import importlib.util
import math
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('entry_patrol', ROOT / 'deploy/ros/g2_map_patrol.py')
patrol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patrol)
from g2_patrol_entry import EntryGeometry, BODY_RADIUS
from g2_body_geometry import BODY_RECTANGLES
from g2_navigation_goals import resolve_destination, nearest_route_entry


def policy():
    return patrol.Policy(str(ROOT / 'tests/fixtures/patrol_entry_20261029'))


def runner_at(pose):
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.policy = policy()
    runner.report = {'pulses': 0, 'trace': []}
    runner.sock = None
    runner.end = 1000.
    clock = [100.]
    plant = {'pose': list(pose), 'vx': 0., 'wz': 0., 'commands': []}
    class Socket:
        def sendall(self, data):
            import json
            c = json.loads(data)
            plant['commands'].append(c)
            plant['vx'], plant['wz'] = c.get('vx', 0.), c.get('wz', 0.)
    runner.sock = Socket()
    def state(**kwargs):
        return {'pose': list(plant['pose']), 'odom': list(plant['pose']),
                'odom_stamp': clock[0], 'vx': plant['vx'], 'wz': plant['wz'],
                'front': 2., 'cov': [.00001, .00001, .00001]}
    runner.state = state
    runner.ready = state
    runner.settle = lambda: (runner.stop(), state())[1]
    def sleep(dt):
        clock[0] += dt
        plant['pose'][0] += dt * plant['vx'] * math.cos(plant['pose'][2])
        plant['pose'][1] += dt * plant['vx'] * math.sin(plant['pose'][2])
        plant['pose'][2] += dt * plant['wz']
    return runner, plant, clock, sleep


@pytest.mark.parametrize('x,y,yaw', [(1.04,.37,87), (1.03,.45,114), (1.06,.40,75)])
def test_recorded_startups_reach_turn_clearance_and_route(x,y,yaw):
    r, plant, clock, sleep = runner_at([x,y,math.radians(yaw)])
    if yaw != 87:assert not r.mask_rotation_allowed(plant['pose'])
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), patch.object(patrol.time, 'sleep', side_effect=sleep):
        r.enter_turn_reserve()
        assert r.rotation_allowed(plant['pose'])
        r.pulse(0., 1., .08)
    assert resolve_destination(r.policy, plant['pose'][:2], 'patrol-cycle')
    assert not r._entry_active
    assert r._entry_reverse_used <= .10
    assert all(c.get('vx', 0.) >= -.08 for c in plant['commands'])
    if yaw == 114:
        assert r._entry_reverse_used > 0.
        expected = nearest_route_entry(r.policy, [x,y])
        assert r.report['startup_route_entry']['point'] == expected['point']


@pytest.mark.parametrize('xy', [(.5,.5),(.40,1.18)])
def test_central_start_is_unchanged(xy):
    r, plant, _, _ = runner_at(list(xy)+[0.])
    assert r.mask_rotation_allowed(plant['pose'])
    r.enter_turn_reserve()
    assert not any(c['action'] == 'velocity' for c in plant['commands'])


def test_body_matches_survey_and_full_sweep_rejects_box_and_wall():
    import json
    profile = json.loads((ROOT/'deploy/ros/lidar_self_returns_2026-10-18.json').read_text())
    assert tuple(tuple(p['bounds_xyxy']) for p in profile['body_rectangles_base_m']) == BODY_RECTANGLES
    assert BODY_RADIUS == pytest.approx(.308,abs=.0002)
    g = EntryGeometry(policy())
    assert not g.turn_allowed([1.25,.6,0.])
    assert not g.turn_allowed(policy().layout['room_polygon'][0]+[0.])


def test_reverse_limits_blocked_path_and_no_global_reverse():
    r, _, _, _ = runner_at([.5,.5,0.])
    with pytest.raises(RuntimeError, match='startup'):
        r.pulse(-.08,0.,.08)
    g = EntryGeometry(r.policy)
    assert g.reverse_problem([.5,.5,0.],.101)
    assert g.reverse_problem([1.25,.6,0.],.08)
    # The 114-degree forward direction would sweep into the box reserve.
    assert g.translation_problem([1.03,.45,math.radians(114)],.088)


def test_uncertainty_brake_and_stale_state_never_send_velocity():
    for fault in ('uncertainty', 'brake', 'stale'):
        r, plant, clock, sleep = runner_at([1.06,.40,math.radians(75)])
        original = r.state
        def state(**kwargs):
            if fault == 'stale':raise patrol.Transient('stale scan')
            s = original()
            if fault == 'uncertainty':s['cov'] = [.01,.01,.01]
            if fault == 'brake':s['front'] = .1
            return s
        r.state = state
        r.ready = state
        with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), patch.object(patrol.time, 'sleep', side_effect=sleep):
            with pytest.raises(patrol.Transient):
                if fault == 'brake':r.pulse(.08,0.,.08)
                else:r.pulse(0.,1.,.08)
        assert not any(c['action'] == 'velocity' for c in plant['commands'])


def test_reverse_cumulative_reserve_rejects_before_output():
    r, plant, clock, sleep = runner_at([1.03,.45,math.radians(114)])
    r._entry_active = True
    r._entry_reverse_used = .02
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), patch.object(patrol.time, 'sleep', side_effect=sleep):
        with pytest.raises(RuntimeError, match='total distance'):
            r.pulse(-.08,0.,.08)
    assert not any(c['action']=='velocity' for c in plant['commands'])


def test_away_from_bin_wall_reserve_is_unchanged():
    r, _, _, _ = runner_at([.5,.5,0.])
    r.policy.allowed = lambda x,y: .4 <= x <= 1.6 and .4 <= y <= 1.6
    # No physical fallback is allowed for an unrelated failed wall/unknown ring.
    assert not r.rotation_allowed([.41,.8,0.])


def test_turn_drift_and_midpulse_sensor_loss_stop_output():
    for failure in ('drift','stale'):
        r, plant, clock, sleep = runner_at([1.06,.4,math.radians(75)])
        original = r.state
        def state(**kwargs):
            s = original()
            if any(c['action']=='velocity' for c in plant['commands']):
                if failure=='stale':raise patrol.Transient('stale scan')
                s['odom'][0] += .011
            return s
        r.state = state
        with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), patch.object(patrol.time, 'sleep', side_effect=sleep):
            with pytest.raises(RuntimeError):r.pulse(0.,1.,.08)
        assert plant['commands'][-1]['action']=='stop'
