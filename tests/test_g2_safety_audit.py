import sys
import time
from pathlib import Path
from unittest.mock import Mock
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
sys.path.insert(0,str(ROOT/'grasp/v23'))
import g2_safety_audit as audit
from grasp_service import GraspService
from chassis_server import Chassis


def healthy():
    return dict(ok=True,chassis_tcp_ready=True,
                chassis=dict(motors=[0,0,0,0],moving=False,arm_busy=False,
                             arm_pose='travel',board_rx_age_s=0.0,client=None),
                odom=dict(valid=True,twist=[0,0,0]))


def test_healthy_controls():
    assert audit.control_faults(healthy(),True,1024)==[]


@pytest.mark.parametrize('age',[None,float('nan'),float('inf'),-1,0.3])
def test_board_fault_blocks_repair(age):
    status=healthy();status['chassis']['board_rx_age_s']=age
    assert 'board_feedback_stale' in audit.control_faults(status,True,1024)


def test_closed_startup_never_repairable():
    status=healthy();status['chassis_tcp_ready']=False
    status['chassis']['arm_pose']='startup_failed'
    assert 'startup_interlock' in audit.control_faults(status,True,1024)


def test_busy_or_active_client_blocks_repair():
    status=healthy();status['chassis']['arm_busy']=True
    status['chassis']['client']='navigation'
    reasons=audit.control_faults(status,True,1024)
    assert 'arm_not_idle_travel' in reasons
    assert 'navigation_client_present' in reasons


def test_low_ram_and_competing_owner():
    reasons=audit.control_faults(healthy(),False,700)
    assert 'RAM_pressure' in reasons and 'serial_owner_mismatch' in reasons


def test_old_or_missing_topics_are_not_healthy():
    assert len(audit.sensor_faults({},10,10))==3
    scan={'header':{'stamp':{'secs':1,'nsecs':0},'frame_id':'laser'},
          'ranges':[1],'range_min':0.1,'range_max':3}
    assert '/scan_missing_or_stale' in audit.sensor_faults({'/scan':(scan,10)},10,10)


def test_health_never_reads_servos_or_moves():
    svc=GraspService.__new__(GraspService)
    svc.controller=Mock()
    svc.chassis=Mock();svc.chassis.status.return_value=healthy()['chassis']
    svc.odom_bridge=Mock();svc.odom_bridge.status.return_value=healthy()['odom']
    svc.startup_pose={'ok':True};svc.chassis_tcp_ready=True
    svc.started_monotonic=time.monotonic()
    assert svc.cmd_health()['ok']
    svc.controller.servo.read_degrees.assert_not_called()
    svc.chassis.stop.assert_not_called()


def test_maintenance_lock_blocks_wheels_and_arm_atomically():
    board=Mock()
    chassis=Chassis(board,lambda vx,wz:(30,30,30,30),rx_age=lambda:0,log=lambda _:None)
    chassis.stop();chassis.set_arm_pose('travel')
    assert chassis.maintenance(True)
    assert chassis.command({'action':'velocity','vx':0.15,'wz':0})=='maintenance'
    assert chassis.begin_arm()=='maintenance'
    assert board.set_motor.call_args.args==(0,0,0,0)
    assert chassis.status()['maintenance']
    assert chassis.maintenance(False)


def test_maintenance_refuses_competing_client():
    chassis=Chassis(Mock(),lambda vx,wz:(0,0,0,0),rx_age=lambda:0,log=lambda _:None)
    chassis.stop();chassis.set_arm_pose('travel');chassis.client='navigation'
    assert not chassis.maintenance(True)


@pytest.mark.parametrize('vision_active,expected_exit',[(True,0),(False,2)])
def test_boot_gate_only_unlocks_after_complete_readiness(monkeypatch,tmp_path,vision_active,expected_exit):
    clock=[0.0]
    state=healthy()
    state['chassis']['maintenance']=True
    commands=[]
    def diagnostic(command='health'):
        commands.append(command)
        if command=='maintenance-unlock':
            state['chassis']['maintenance']=False
        return state if command=='health' else {'ok':True}
    monkeypatch.setattr(audit,'diagnostic',diagnostic)
    monkeypatch.setattr(audit,'owner_matches',lambda:True)
    monkeypatch.setattr(audit,'available_mb',lambda:1500)
    monkeypatch.setattr(audit,'sensor_faults',lambda *args:[])
    sensors=Mock();sensors.snapshot.return_value=({}, {})
    monkeypatch.setattr(audit,'Sensors',lambda:sensors)
    monkeypatch.setattr(audit,'shell',lambda args:Mock(stdout=(
        'inactive' if args[-1]=='grasp-vision' and not vision_active else 'active')))
    monkeypatch.setattr(audit.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(audit.time,'sleep',lambda secs:clock.__setitem__(0,clock[0]+secs))
    monkeypatch.setattr(sys,'argv',['audit','--boot-gate','--seconds','10',
                                   '--output',str(tmp_path/'audit.json')])
    with pytest.raises(SystemExit) as result:
        audit.main()
    assert result.value.code==expected_exit
    assert commands.count('maintenance-lock')==1
    assert ('maintenance-unlock' in commands)==vision_active
    assert all(command in ['health','maintenance-lock','maintenance-unlock'] for command in commands)
    sensors.close.assert_called_once()


def test_disconnected_health_client_does_not_kill_service(monkeypatch,tmp_path):
    import grasp_service as gs
    import health_server
    monkeypatch.setattr(health_server,'HealthServer',lambda *args:Mock())
    import io
    svc=GraspService.__new__(GraspService)
    svc.sock_path=str(tmp_path/'service.sock')
    svc.chassis=svc.chassis_server=svc.odom_bridge=None
    svc.chassis_tcp_ready=False
    svc.stop_requested=False
    events=[]
    svc.prepare_startup_pose=lambda enabled:(events.append('park') or {'ok':True})
    svc.cmd_health=lambda:{'ok':True}
    disconnected=Mock();disconnected.makefile.return_value=io.StringIO('health\n')
    disconnected.sendall.side_effect=BrokenPipeError()
    quit_client=Mock();quit_client.makefile.return_value=io.StringIO('quit\n')
    server=Mock();server.accept.side_effect=[(disconnected,None),(quit_client,None)]
    server.listen.side_effect=lambda backlog:events.append('listen')
    monkeypatch.setattr(gs.socket,'socket',lambda *args:server)
    monkeypatch.setattr(gs.socket,'AF_UNIX',1,raising=False)
    monkeypatch.setattr(gs.os,'chmod',lambda *args:None)
    svc.serve()
    assert events==['park','listen']
    assert server.accept.call_count==2
    disconnected.close.assert_called_once()
    server.close.assert_called_once()
