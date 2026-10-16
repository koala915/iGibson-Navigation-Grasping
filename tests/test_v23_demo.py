import json
from pathlib import Path
import sys
from unittest.mock import Mock
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
sys.path.insert(0,str(ROOT/'grasp/v23'))
import v23_demo as demo
import g2_safety_audit as audit
from grasp_service import GraspService


def plan(*actions):
    return {'version':'v23','steps':[{'action':a} for a in actions]}


class Backend:
    def __init__(self, fail=None):
        self.pose='travel';self.holding=False;self.calls=[];self.fail=fail
    def guard(self,pose,holding):
        assert (pose,holding)==(self.pose,self.holding)
    def arm(self,command):
        self.calls.append(command)
        if command==self.fail:
            return {'ok':False,'confirmed':False}
        if command=='home':self.pose='e1'
        elif command=='grasp':self.holding=True
        elif command=='stow':self.pose='travel'
        elif command=='release':self.pose='e1';self.holding=False
        return {'ok':True,'confirmed':True}
    def drive(self,distance):
        assert self.pose=='travel'
        self.calls.append('drive')


def test_pick_carry_drive_place_uses_one_backend():
    p=plan('home','grasp','stow','drive','release','stow')
    p['steps'][3]['distance_m']=0.15;p['orientation_evidence']='measured.json'
    backend=Backend()
    assert demo.run_plan(p,backend)['holding'] is False
    assert backend.calls==['home','grasp','stow','drive','release','stow']


def test_failed_grasp_never_drives_or_retries():
    backend=Backend(fail='grasp')
    with pytest.raises(RuntimeError):demo.run_plan(plan('home','grasp','stow'),backend)
    assert backend.calls==['home','grasp']


def test_ambiguous_timeout_never_replays_or_releases():
    backend=Backend();backend.arm=Mock(side_effect=TimeoutError('still running'))
    with pytest.raises(TimeoutError):demo.run_plan(plan('home','grasp','stow'),backend)
    assert backend.arm.call_count==1


@pytest.mark.parametrize('p',[
    {'version':'v21','steps':[{'action':'stow'}]},
    plan('grasp','stow'),plan('home','grasp','home','stow'),
    plan('release','stow'),plan('home'),plan('quit'),
    {'version':'v23','steps':[{'action':'drive','distance_m':1}]},
    {'version':'v23','steps':[{'action':'drive','distance_m':float('nan')}],
     'orientation_evidence':'measured.json'},
])
def test_invalid_plan_rejected_before_backend(p):
    with pytest.raises(ValueError):demo.validate_plan(p)


def test_guard_failure_sends_no_arm_commands():
    backend=Backend();backend.guard=Mock(side_effect=RuntimeError('RAM_pressure'))
    with pytest.raises(RuntimeError):demo.run_plan(plan('stow'),backend)
    assert backend.calls==[]


def test_health_refuses_wrong_model_stack_or_maintenance():
    state={'ok':True,'stack_version':'v21','chassis_tcp_ready':True}
    with pytest.raises(RuntimeError):demo.check_health(state,'travel',False)


@pytest.mark.parametrize('age,pid',[(4,20),(-1,20),(1,21)])
def test_vision_stale_future_or_previous_process_rejected(tmp_path,age,pid):
    f=tmp_path/'health.json'
    f.write_text(json.dumps(dict(pid=20,monotonic=10-age,width=640,height=480,inference_s=.2)))
    assert audit.vision_faults(str(f),10,pid)


def test_vision_fresh_without_detected_object_is_healthy(tmp_path):
    f=tmp_path/'health.json'
    f.write_text(json.dumps(dict(pid=20,monotonic=10,width=640,height=480,inference_s=.2)))
    assert audit.vision_faults(str(f),11,20)==[]


def test_detection_timeout_survives_wall_clock_jump(monkeypatch):
    import grasp_service as gs
    svc=GraspService.__new__(GraspService)
    svc.detection=lambda:{'fresh':False}
    clock=[100.0]
    monkeypatch.setattr(gs.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(gs.time,'time',lambda:-1000000)
    monkeypatch.setattr(gs.time,'sleep',lambda sec:clock.__setitem__(0,clock[0]+sec))
    assert svc.wait_for_detection(1) is None
    assert 101 <= clock[0] <= 101.3


def test_arm_low_memory_refuses_before_serial_or_motion(monkeypatch):
    svc=GraspService.__new__(GraspService)
    monkeypatch.setenv('GRASP_SERVICE_MIN_AVAILABLE_MB','768')
    monkeypatch.setattr(Path,'read_text',lambda *args,**kw:'MemAvailable: 1024 kB\n')
    fn=Mock()
    assert svc.run_arm_command(fn)['reason']=='RAM_pressure'
    fn.assert_not_called()


def test_default_preview_does_not_construct_backend(monkeypatch,tmp_path):
    f=tmp_path/'plan.json';f.write_text(json.dumps(plan('stow')))
    factory=Mock(side_effect=AssertionError('hardware backend created'))
    monkeypatch.setattr(demo,'ResidentBackend',factory)
    assert demo.main(['--plan',str(f)])==0
    factory.assert_not_called()


def resident_health():
    return dict(ok=True,stack_version='v23',chassis_tcp_ready=True,holding=False,
                model_path='candidate_v23_seed23401_ckpt250000.zip',
                vecnorm_path='candidate_v23_seed23401_ckpt250000_vec.pkl',
                chassis=dict(motors=[0,0,0,0],moving=False,arm_busy=False,client=None,
                             maintenance=False,arm_pose='travel',board_rx_age_s=0),
                odom=dict(valid=True,twist=[0,0,0]))


def test_health_accepts_only_idle_v23_pair_and_expected_holding():
    state=resident_health()
    demo.check_health(state,'travel',False)
    state['model_path']='candidate_v24.zip'
    with pytest.raises(RuntimeError):demo.check_health(state,'travel',False)
    state=resident_health();state['holding']=True
    with pytest.raises(RuntimeError):demo.check_health(state,'travel',False)
    state=resident_health();state['chassis']['maintenance']=True
    with pytest.raises(RuntimeError):demo.check_health(state,'travel',False)


def test_frame_heartbeat_is_atomic_and_checks_current_process(tmp_path):
    import os
    from vision_grasp_bridge import write_frame_health
    target=tmp_path/'vision.json'
    write_frame_health(str(target),(480,640,3),0.2)
    sample=json.loads(target.read_text())
    assert audit.vision_faults(str(target),sample['monotonic']+0.1,os.getpid())==[]
    assert list(tmp_path.glob('*.tmp'))==[]


def test_ui_arm_preparation_never_advertises_chassis_permission():
    from mission_status import SimpleReporter
    reporter=SimpleReporter('127.0.0.1:8099',mode='D')
    reporter.emitter.send=Mock()
    try:
        home=demo.emit_telemetry(reporter,0,'home','running')
        assert home['arm_allowed'] is True
        assert home['chassis_allowed'] is False
        completed=demo.emit_telemetry(reporter,0,'home','complete')
        # Force a new state so telemetry throttling is exercised normally.
        stop=demo.emit_telemetry(reporter,-1,'abort','interrupted')
        assert stop['chassis_allowed'] is False and stop['arm_allowed'] is False
    finally:
        reporter.close()
