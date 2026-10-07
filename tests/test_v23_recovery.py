import json
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
sys.path.insert(0,str(ROOT/'grasp/v23'))
from v23_recovery import Recovery, TransientFault, Checkpoint, RestartEscalation, owner_restart_allowed, unresolved_action
from serial_transport import Transport
from health_server import HealthServer
import v23_demo as demo
from chassis_server import Chassis


def policy(check, repair=None, grace=3):
    now = [0.0]
    r = Recovery(repair or Mock(),clock=lambda:now[0],
                 sleep=lambda s:now.__setitem__(0,now[0]+s),grace_s=grace,cooldown_s=0)
    return r.wait(check), r


def test_transient_recovery_requires_three_good_samples():
    check=Mock(side_effect=[TransientFault('camera','grasp-vision'),True,
                           TransientFault('camera','grasp-vision'),True,True,True])
    repair=Mock()
    _,r=policy(check,repair)
    assert check.call_count==6 and repair.call_count==2


def test_repairs_are_bounded_even_when_failure_persists():
    check=Mock(side_effect=TransientFault('camera','grasp-vision'))
    repair=Mock()
    with pytest.raises(TransientFault,match='exhausted'):policy(check,repair)
    assert repair.call_count==2


def test_hard_fault_is_never_repaired():
    repair=Mock()
    with pytest.raises(RuntimeError,match='wrong pose'):
        policy(Mock(side_effect=RuntimeError('wrong pose')),repair)
    repair.assert_not_called()


def test_serial_fault_never_restarts_owner():
    repair=Mock()
    with pytest.raises(TransientFault):
        policy(Mock(side_effect=TransientFault('serial','grasp-service')),repair)
    repair.assert_not_called()


@pytest.mark.parametrize('change',['inflight','boot','service','plan','pose'])
def test_checkpoint_refuses_ambiguous_or_changed_identity(tmp_path,change):
    p={'version':'v23','steps':[{'action':'home'},{'action':'stow'}]}
    c=Checkpoint(tmp_path/'checkpoint.json',p,'boot1',123)
    c.write(1,'e1',False,change=='inflight')
    if change=='boot':c=Checkpoint(c.path,p,'boot2',123)
    if change=='service':c=Checkpoint(c.path,p,'boot1',456)
    if change=='plan':c=Checkpoint(c.path,{'version':'v23','steps':[{'action':'stow'}]},'boot1',123)
    if change=='pose':c.write(1,'travel',False)
    with pytest.raises(RuntimeError):c.resume(p['steps'])


def test_resume_does_not_replay_completed_home(tmp_path):
    p={'version':'v23','steps':[{'action':'home'},{'action':'stow'}]}
    c=Checkpoint(tmp_path/'checkpoint.json',p,'boot1',123);c.write(1,'e1',False)
    backend=Mock();backend.guard.return_value=None;backend.arm.return_value={'ok':True}
    assert demo.run_plan(p,backend,checkpoint=c,resume=True)['ok']
    backend.arm.assert_called_once_with('stow')


def test_timeout_degrades_without_replaying_or_parking(tmp_path):
    p={'version':'v23','steps':[{'action':'home'},{'action':'stow'}]}
    backend=Mock();backend.guard.return_value=None
    backend.arm.side_effect=TimeoutError('unknown result')
    c=Checkpoint(tmp_path/'checkpoint.json',p,'boot1',123)
    result=demo.run_plan(p,backend,recovery=Recovery(Mock()),checkpoint=c)
    assert result['mode']=='diagnostics' and result['degraded']
    backend.arm.assert_called_once_with('home')
    assert json.loads(c.path.read_text())['inflight']


def test_vision_failure_can_park_without_grasp(tmp_path):
    p={'version':'v23','steps':[{'action':'home'},{'action':'grasp'},{'action':'stow'}]}
    backend=Mock();backend.guard.side_effect=TransientFault('vision')
    backend.guard_control.return_value=None;backend.arm.return_value={'ok':True}
    r=Recovery(Mock(),grace_s=0)
    result=demo.run_plan(p,backend,recovery=r)
    assert result['mode']=='parked' and result['holding'] is False
    backend.arm.assert_called_once_with('stow')


class Serial:
    def __init__(self, chunks=()):self.chunks=list(chunks);self.writes=[]
    def write(self,data):self.writes.append(data);return len(data)
    def read(self,size):
        if self.chunks:
            item=self.chunks.pop(0)
            if isinstance(item,Exception):raise item
            return item
        time.sleep(.001);return b''


def transport(chunks=()):
    device=SimpleNamespace(ser=Serial(chunks),_Rosmaster__parse_data=Mock())
    return device,Transport(device)


def test_receiver_recovers_read_exception_and_parses_checksum():
    packet=bytes([255,251,4,10,7,21])
    device,t=transport([OSError('temporary')]+[bytes([b]) for b in packet])
    t.start()
    try:
        deadline=time.monotonic()+1
        while not t.status()['healthy'] and time.monotonic()<deadline:time.sleep(.01)
        assert t.status()['healthy'] and t.read_errors==1
        device._Rosmaster__parse_data.assert_called_once_with(10,[7,21])
        assert device.ser.writes==[]
    finally:t.close()
    assert not t.thread.is_alive()


def test_receiver_rejects_bad_checksum_and_noise():
    device,t=transport([bytes([b]) for b in [1,255,251,4,10,7,22]])
    t.start();time.sleep(.03);t.close()
    device._Rosmaster__parse_data.assert_not_called()
    assert not t.status()['healthy']


def test_write_timeout_and_short_write_are_latched():
    device,t=transport();t.raw_write=Mock(return_value=0)
    with pytest.raises(OSError,match='short'):t.write(b'packet')
    with pytest.raises(OSError):t.require_write_ok()
    with pytest.raises(OSError):t.write(b'packet')
    assert t.raw_write.call_count==1
    assert device.ser.timeout==.1 and device.ser.write_timeout==.2


def test_write_lock_has_a_deadline():
    _,t=transport();t.lock.acquire()
    before=time.monotonic()
    try:
        with pytest.raises(OSError,match='lock_timeout'):t.write(b'packet')
    finally:t.lock.release()
    assert time.monotonic()-before<1


def test_servo_packets_do_not_make_stale_motion_feedback_fresh():
    _,t=transport();t.thread=Mock();t.thread.is_alive.return_value=True
    t.last_packet=time.monotonic();t.last_motion_packet=time.monotonic()-2
    assert not t.status()['healthy']


def repair_health():
    return dict(ok=True,stack_version='v23',chassis_tcp_ready=True,holding=False,
                model_path='candidate_v23_seed23401_ckpt250000.zip',
                vecnorm_path='candidate_v23_seed23401_ckpt250000_vec.pkl',
                chassis=dict(motors=[0]*4,moving=False,arm_busy=False,client=None,
                             maintenance=False,arm_pose='travel',board_rx_age_s=0),
                odom=dict(valid=True,twist=[0]*3))


def broken_health():
    value=repair_health()
    value.update(ok=False,service_pid=123,serial={'healthy':False})
    value['chassis']['board_rx_age_s']=30
    return value


@pytest.mark.parametrize('field,value',[('holding',True),('holding',None),
    ('arm_busy',True),('moving',True),('motors',[30]*4),('arm_pose','e1'),('client','tcp')])
def test_owner_restart_refuses_uncertain_or_active_state(field,value):
    state=broken_health()
    if field=='holding':state[field]=value
    else:state['chassis'][field]=value
    with pytest.raises(RuntimeError):owner_restart_allowed(state,True,1500)


def test_owner_restart_can_recover_stale_rx_but_not_owner_or_ram():
    owner_restart_allowed(broken_health(),True,1500)
    for owner,ram in [(False,1500),(True,700)]:
        with pytest.raises(RuntimeError):owner_restart_allowed(broken_health(),owner,ram)


def test_restart_lock_blocks_arm_and_velocity_without_a_bus_write():
    board=Mock()
    chassis=Chassis(board,lambda vx,wz:(30,)*4,rx_age=lambda:1,log=lambda _:None)
    chassis.stop();chassis.set_arm_pose('travel');board.set_motor.reset_mock()
    assert chassis.restart_lock()
    assert chassis.begin_arm()=='maintenance'
    assert chassis.command({'action':'velocity','vx':.15})=='maintenance'
    board.set_motor.assert_not_called()
    assert chassis.status()['arm_pose']=='restart_pending'


def test_restart_lock_refuses_a_connected_navigation_client():
    chassis=Chassis(Mock(),lambda vx,wz:(30,)*4,rx_age=lambda:1,log=lambda _:None)
    chassis.stop();chassis.set_arm_pose('travel');chassis.client='tcp'
    assert not chassis.restart_lock()


def test_owner_restart_budget_is_consumed_only_after_atomic_lock(monkeypatch):
    import g2_safety_audit as audit
    import v23_recovery
    backend=demo.ResidentBackend();backend.health=Mock(return_value=broken_health())
    monkeypatch.setattr(audit,'owner_matches',lambda:True)
    monkeypatch.setattr(audit,'available_mb',lambda:1500)
    monkeypatch.setattr(v23_recovery,'unresolved_action',lambda state:False)
    monkeypatch.setattr(demo,'service_request',Mock(return_value={'ok':False}))
    authorize=Mock();shell=Mock();monkeypatch.setattr(audit,'shell',shell)
    with pytest.raises(RuntimeError,match='lock refused'):backend.restart_owner(authorize)
    authorize.assert_not_called();shell.assert_not_called()


def test_owner_restart_never_issues_home_or_replays_motion(monkeypatch):
    import g2_safety_audit as audit
    import v23_recovery
    backend=demo.ResidentBackend();backend.health=Mock(return_value=broken_health())
    monkeypatch.setattr(audit,'owner_matches',lambda:True)
    monkeypatch.setattr(audit,'available_mb',lambda:1500)
    monkeypatch.setattr(v23_recovery,'unresolved_action',lambda state:False)
    request=Mock(return_value={'ok':True});monkeypatch.setattr(demo,'service_request',request)
    shell=Mock(return_value=SimpleNamespace(returncode=0));monkeypatch.setattr(audit,'shell',shell)
    authorize=Mock();backend.restart_owner(authorize)
    request.assert_called_once_with('restart-lock');authorize.assert_called_once_with()
    shell.assert_called_once_with(['sudo','-n','systemctl','restart','grasp-service'])


def test_escalation_waits_and_budget_survives_monitor_restart(tmp_path):
    clock=[0.0];calls=[]
    def restart(authorize):authorize();calls.append('restart')
    budget=tmp_path/'budget.json';marker=tmp_path/'marker.json'
    e=RestartEscalation('boot1',budget,marker,clock=lambda:clock[0])
    assert e.observe(broken_health(),restart)=='waiting' and calls==[]
    clock[0]=15
    assert e.observe(broken_health(),restart)=='restarting'
    assert json.loads(marker.read_text())==dict(boot_id='boot1',no_motion=True)
    new=RestartEscalation('boot1',budget,marker,clock=lambda:clock[0],delay_s=0)
    assert new.observe(broken_health(),restart)=='restart_exhausted'
    assert calls==['restart']


def test_restart_timeout_consumes_attempt_without_replay(tmp_path):
    def timeout(authorize):authorize();raise TimeoutError('systemctl')
    e=RestartEscalation('boot1',tmp_path/'budget',tmp_path/'marker',delay_s=0)
    with pytest.raises(TimeoutError):e.observe(broken_health(),timeout)
    assert e.observe(broken_health(),Mock())=='restart_exhausted'


def test_bad_restart_budget_never_resets_itself(tmp_path):
    budget=tmp_path/'budget';budget.write_text('{}')
    e=RestartEscalation('boot1',budget,tmp_path/'marker',delay_s=0)
    callback=Mock()
    with pytest.raises(RuntimeError):e.observe(broken_health(),callback)
    callback.assert_not_called()


def test_unknown_checkpoint_blocks_restart_of_its_owner(tmp_path):
    path=tmp_path/'checkpoint'
    path.write_text(json.dumps({'service_pid':123,'inflight':True}))
    assert unresolved_action(broken_health(),path)
    path.write_text(json.dumps({'service_pid':123,'inflight':False}))
    assert not unresolved_action(broken_health(),path)


@pytest.mark.parametrize('pose,age,expected',[
    ((90,140,0,0,90,30),0,True),((90,140,0,0,90,100),0,False),
    ((90,74.2,8.6,8.6,90,30),0,False),((90,140,0,0,90,30),2,False)])
def test_recovery_startup_never_homes_or_opens_jaw(pose,age,expected):
    import grasp_service as gs
    from test_travel_pose import Ctl
    ctl=Ctl(pose);svc=gs.GraspService(ctl,None,1,'unused')
    svc.chassis=Mock();svc.chassis.status.return_value={'board_rx_age_s':age}
    assert svc.prepare_startup_pose(True,readonly=True)['ok'] is expected
    assert ctl.moves==[] and ctl.released==0


def test_restart_marker_applies_only_to_current_boot_and_corruption_is_readonly(tmp_path):
    import grasp_service as gs
    marker=tmp_path/'marker'
    assert not gs.restart_without_motion(marker,'boot1')
    marker.write_text(json.dumps({'boot_id':'boot1'}))
    assert gs.restart_without_motion(marker,'boot1')
    assert not gs.restart_without_motion(marker,'boot2')
    marker.write_text('broken')
    assert gs.restart_without_motion(marker,'boot1')


def test_repair_never_restarts_serial_owner():
    backend=demo.ResidentBackend();backend.health=Mock()
    with pytest.raises(RuntimeError,match='not allowed'):backend.repair('grasp-service')
    backend.health.assert_not_called()


def test_repair_uses_verified_maintenance_lock(monkeypatch):
    import g2_safety_audit as audit
    backend=demo.ResidentBackend();backend.health=Mock(return_value=repair_health())
    monkeypatch.setattr(audit,'owner_matches',lambda:True)
    monkeypatch.setattr(audit,'available_mb',lambda:1500)
    shell=Mock(return_value=SimpleNamespace(returncode=0));monkeypatch.setattr(audit,'shell',shell)
    request=Mock(return_value={'ok':True});monkeypatch.setattr(demo,'service_request',request)
    backend.repair('grasp-vision')
    assert [c.args[0] for c in request.call_args_list]==['maintenance-lock','maintenance-unlock']
    shell.assert_called_once_with(['sudo','-n','systemctl','restart','grasp-vision'])


def test_failed_repair_keeps_lock_and_never_moves(monkeypatch):
    import g2_safety_audit as audit
    backend=demo.ResidentBackend();backend.health=Mock(return_value=repair_health())
    monkeypatch.setattr(audit,'owner_matches',lambda:True)
    monkeypatch.setattr(audit,'available_mb',lambda:1500)
    monkeypatch.setattr(audit,'shell',Mock(return_value=SimpleNamespace(returncode=1)))
    request=Mock(return_value={'ok':True});monkeypatch.setattr(demo,'service_request',request)
    with pytest.raises(RuntimeError,match='lock retained'):backend.repair('grasp-vision')
    request.assert_called_once_with('maintenance-lock')


def test_swallowed_driver_error_does_not_claim_motor_stop():
    device,t=transport();device._v23_transport=t
    def motor(*args):
        try:t.write(b'packet')
        except OSError:pass
    device.set_motor=motor;t.raw_write=Mock(side_effect=OSError('broken'))
    chassis=Chassis(device,lambda vx,wz:(30,30,30,30),log=lambda _:None)
    with pytest.raises(OSError):chassis.stop()
    assert chassis.status()['motors'] is None
    assert chassis.begin_arm()=='board_silent'


@pytest.mark.skipif(not hasattr(socket,'AF_UNIX'),reason='Linux socket required')
def test_independent_health_does_not_wait_for_arm(tmp_path):
    server=HealthServer(lambda:{'ok':True,'arm_busy':True},str(tmp_path/'health.sock'))
    server.start()
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(.5);client.connect(server.path);client.sendall(b'health\n')
            assert json.loads(client.recv(1024))['arm_busy']
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(.5);client.connect(server.path);client.sendall(b'home\n')
            assert not json.loads(client.recv(1024))['ok']
    finally:server.close()
