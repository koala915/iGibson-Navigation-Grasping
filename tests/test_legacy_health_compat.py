import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'grasp/v23'))
sys.path.insert(0,str(ROOT/'integration'))
from legacy_health_compat import health_snapshot
from frame_health import write_frame_health


def test_health_does_not_read_servos_and_does_not_invent_transport_health():
    class Servo:
        device=None
        def read_degrees(self):
            raise AssertionError('health must never read hardware')
    service=NS(controller=NS(servo=Servo(),cfg=NS(model_path='model',vecnorm_path='norm')),
               chassis=NS(status=lambda:{'arm_busy':True}),odom_bridge=None,
               startup_pose={},chassis_tcp_ready=True,started_monotonic=0)
    result=health_snapshot(service,123.45)
    assert result['chassis']['arm_busy'] is True
    assert result['serial'] is None and result['holding'] is False
    assert result['capabilities']['owner_restart'] is False


def test_real_transport_failure_not_hidden():
    device=NS(_v23_transport=NS(status=lambda:{'healthy':False}))
    service=NS(controller=NS(servo=NS(device=device),cfg=NS(model_path='m',vecnorm_path='v'),_grip_hold_rad=.1),
               chassis=None,odom_bridge=None,startup_pose={},chassis_tcp_ready=False,started_monotonic=0)
    result=health_snapshot(service,1)
    assert result['ok'] is False and result['holding'] is True


def test_frame_heartbeat_contains_real_inference_fields(tmp_path):
    path=tmp_path/'health.json'
    write_frame_health(path,(480,640,3),.25)
    result=json.loads(path.read_text())
    assert result['width']==640 and result['height']==480 and result['inference_s']==.25
    assert result['pid']>0 and result['monotonic']>0
    assert len(list(tmp_path.iterdir()))==1


def test_invalid_frame_cannot_refresh_heartbeat(tmp_path):
    path=tmp_path/'health.json'
    with pytest.raises(ValueError):write_frame_health(path,(0,640),.1)
    assert not path.exists()


def test_patch_rejects_unknown_serve_loop():
    spec=importlib.util.spec_from_file_location('compat_patch',ROOT/'deploy/patches/add_independent_health_legacy.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    with pytest.raises(ValueError):module.patch_service('class Service: pass\n')


def test_patch_preserves_arm_and_team_specific_methods():
    import ast
    spec=importlib.util.spec_from_file_location('compat_patch',ROOT/'deploy/patches/add_independent_health_legacy.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source='''class Service:
    def cmd_health(self):
        return {"ok": True}
    def team_specific_arm(self):
        return "keep exact behavior"
    def serve(self):
        srv.bind(self.sock_path)
        try:
            self.team_specific_arm()
        finally:
            srv.close()
'''
    patched=module.patch_service(source)
    def method(text,name):
        return next(n for n in ast.walk(ast.parse(text)) if isinstance(n,ast.FunctionDef) and n.name==name)
    assert ast.dump(method(source,'team_specific_arm'))==ast.dump(method(patched,'team_specific_arm'))
    assert 'health_server.start()' in patched and 'health_server.close()' in patched
    with pytest.raises(ValueError):module.patch_service(patched)
