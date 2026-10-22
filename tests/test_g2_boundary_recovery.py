import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('patrol_escape', ROOT / 'deploy/ros/g2_map_patrol.py')
patrol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patrol)
RecoveryPolicy = patrol.RecoveryPolicy


class Scene:
    layout = {'room_heading_rad': 0., 'room_polygon': [(0., 0.), (2., 0.), (2., 2.), (0., 2.)],
              'boundary_recovery': {'enabled': True, 'environment': 'walls_and_fixed_bin',
                                    'body_radius_m': .25, 'error_budget_m': .10},
              'navigation_mask_validation': {'envelope_room_uv': [1.7, 1.9, 1.7, 1.9]}}

    def allowed(self, x, y):
        return (.4 <= x <= 1.6 and .4 <= y <= 1.6
                and math.hypot(max(1.7 - x, 0., x - 1.9), max(1.7 - y, 0., y - 1.9)) >= .4)


def test_soft_forbidden_point_has_a_physical_escape_corridor():
    scene = Scene()
    recovery = RecoveryPolicy(scene)
    pose = (.38, 1., 0.)
    assert not scene.allowed(*pose[:2])
    target = recovery.target(pose)
    assert recovery.recovered(target)
    assert recovery.segment(pose, target)
    assert recovery.command(pose, target) == (.08, 0.)


def test_a_target_behind_is_faced_by_turning_never_by_reversing():
    # Reversing is allowed only in the enoch approach (koala, 2026-10-12).
    recovery = RecoveryPolicy(Scene())
    pose = (.38, 1., math.pi)
    vx, wz = recovery.command(pose, recovery.target(pose))
    assert vx == 0. and abs(wz) == 1.


def test_parallel_heading_uses_bounded_turn_in_physical_clearance():
    recovery = RecoveryPolicy(Scene())
    pose = (.38, 1., math.pi / 2)
    vx, wz = recovery.command(pose, recovery.target(pose))
    assert vx == 0. and abs(wz) == 1.


def test_real_wall_and_bin_cannot_be_escaped_through():
    recovery = RecoveryPolicy(Scene())
    for pose in ((.20, 1., 0.), (1.6, 1.65, 0.), (float('nan'), 1., 0.)):
        with pytest.raises(ValueError, match='physical'):
            recovery.target(pose)
    assert not recovery.segment((1.2, 1.65), (1.65, 1.2))


def simulated_runner(pose):
    clock = [100.]
    plant = {'pose': list(pose), 'vx': 0., 'wz': 0., 'commands': [], 'fault': None}
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.policy = Scene()
    runner.boundary_recovery = True
    runner.report = {'pulses': 0, 'trace': []}
    runner.end = 1000.

    class Socket:
        def sendall(self, data):
            command = json.loads(data)
            plant['commands'].append(command)
            plant['vx'] = command.get('vx', 0.)
            plant['wz'] = command.get('wz', 0.)

    runner.sock = Socket()

    def state(check_vision=True):
        if plant['fault']:
            raise patrol.Transient(plant['fault'])
        if not runner.policy.allowed(*plant['pose'][:2]) and not getattr(runner, '_boundary_recovery_active', False):
            raise patrol.BoundaryRequired('current centre outside navigation mask')
        return {'pose': list(plant['pose']), 'odom': list(plant['pose']), 'odom_stamp': clock[0],
                'vx': plant['vx'], 'wz': plant['wz'], 'cov': [.000025, .000025, .000025],
                'front': .01}  # body return: NOT a wall or an obstacle at 1cm

    def sleep(dt):
        clock[0] += dt
        plant['pose'][0] += dt * plant['vx'] * math.cos(plant['pose'][2])
        plant['pose'][1] += dt * plant['vx'] * math.sin(plant['pose'][2])
        plant['pose'][2] += dt * plant['wz']

    runner.state = state
    return runner, plant, clock, sleep


@pytest.mark.parametrize('yaw', [0., math.pi, math.pi / 2])
def test_runner_recovers_then_normal_ready_works_despite_body_return(yaw):
    runner, plant, clock, sleep = simulated_runner((.38, 1., yaw))
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep):
        runner.recover_boundary()
        state = runner.ready()
    assert RecoveryPolicy(runner.policy).recovered(state['pose'])
    assert runner.report['boundary_escapes'][-1]['ok']
    assert any(c['action'] == 'velocity' for c in plant['commands'])
    assert plant['commands'][-1]['action'] == 'stop'
    assert not runner._boundary_recovery_active


def test_stale_state_never_sends_recovery_velocity():
    runner, plant, clock, sleep = simulated_runner((.38, 1., 0.))
    plant['fault'] = 'stale scan/odom'
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep), \
            pytest.raises(patrol.Transient, match='stale'):
        runner.recover_boundary()
    assert not any(c['action'] == 'velocity' for c in plant['commands'])


def test_overconfident_soft_escape_does_not_ignore_covariance_budget():
    runner, plant, clock, sleep = simulated_runner((.38, 1., 0.))
    original = runner.state
    runner.state = lambda **kw: dict(original(**kw), cov=[.01, .01, .01])
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep), \
            pytest.raises(RuntimeError, match='uncertainty'):
        runner.recover_boundary()
    assert not any(c['action'] == 'velocity' for c in plant['commands'])
    assert plant['commands'][-1]['action'] == 'stop'


def test_ready_invokes_recovery_instead_of_terminating_outside_mask():
    runner, plant, clock, sleep = simulated_runner((.38, 1., 0.))
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep):
        state = runner.ready()
    assert runner.policy.allowed(*state['pose'][:2])
    assert runner.report['boundary_escapes'][-1]['ok']


def test_fault_during_escape_stops_and_clears_exception_mode():
    runner, plant, clock, sleep = simulated_runner((.38, 1., 0.))
    original = runner.state
    def state(check_vision=True):
        if any(c['action'] == 'velocity' for c in plant['commands']):
            raise patrol.Transient('stale scan/odom')
        return original(check_vision)
    runner.state = state
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep), \
            pytest.raises(patrol.Transient, match='stale'):
        runner.recover_boundary()
    assert plant['commands'][-1]['action'] == 'stop'
    assert not runner._boundary_recovery_active


def test_the_laser_brake_stays_in_patrol_pulses_with_boundary_recovery():
    # The operator stands in the room during a mission and only the laser sees
    # a person: the map geometry is added to the brake, never put in its place.
    runner, plant, clock, sleep = simulated_runner((.6, 1., 0.))
    runner.policy.stopping_path_allowed = lambda pose, distance: True
    runner.ready = lambda: runner.state()
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep), \
            pytest.raises(patrol.Transient, match='front brake'):
        runner.pulse(.15, 0., .08)
    assert not any(c['action'] == 'velocity' for c in plant['commands'])


def test_empty_room_mode_still_rejects_real_wall_even_with_bad_soft_mask():
    runner, plant, clock, sleep = simulated_runner((1.59, 1., 0.))
    runner.policy.stopping_path_allowed = lambda pose, distance: True
    original = runner.state
    runner.state = lambda check_vision=True: dict(original(check_vision), front=2.)
    runner.ready = lambda: runner.state()
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep), \
            pytest.raises(patrol.Transient, match='physical wall'):
        runner.pulse(.15, 0., .08)
    assert not any(c['action'] == 'velocity' for c in plant['commands'])
    assert plant['commands'][-1]['action'] == 'stop'


def test_stop_can_be_confirmed_outside_soft_mask_without_moving():
    runner, plant, clock, sleep = simulated_runner((.38, 1., 0.))
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
            patch.object(patrol.time, 'sleep', side_effect=sleep):
        stopped = runner.settle()
    assert stopped['vx'] == 0.
    assert not runner.policy.allowed(*stopped['pose'][:2])
    assert not any(c['action'] == 'velocity' for c in plant['commands'])
    assert not runner._boundary_recovery_active
