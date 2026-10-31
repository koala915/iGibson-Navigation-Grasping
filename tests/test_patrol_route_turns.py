"""The real patrol drives the real route on a noisy plant: every turn must be allowed."""
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'integration'), str(ROOT/'deploy/ros')]
from g2_navigation_goals import patrol_looks, resolve_destination  # noqa: E402
from g2_patrol_entry import BODY_RADIUS, TURN_MARGIN, EntryGeometry  # noqa: E402
from g2_patrol_policy import Policy  # noqa: E402

SCENE = ROOT/'maps/demo_box_20261018_patrol'
spec = importlib.util.spec_from_file_location('route_patrol', ROOT/'deploy/ros/g2_map_patrol.py')
patrol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patrol)


def drive(policy, start, seed, sigma=.01, heading_bias=.02, slip=.02):
    """One whole patrol-cycle with AMCL noise, a wheel heading bias and slip."""
    rng = random.Random(seed)
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.policy, runner.sock, runner.end = policy, None, 1e9
    runner.report = {'pulses': 0, 'trace': [], 'arrivals': [], 'recoveries': [], 'planned_paths': []}
    clock = [100.]
    plant = {'pose': list(start), 'vx': 0., 'wz': 0.}

    class Socket:
        def sendall(self, data):
            for line in data.decode().splitlines():
                command = json.loads(line)
                plant['vx'], plant['wz'] = command.get('vx', 0.), command.get('wz', 0.)
    runner.sock = Socket()

    def state(**_):
        pose = plant['pose']
        return {'pose': [pose[0]+rng.gauss(0, sigma), pose[1]+rng.gauss(0, sigma),
                         pose[2]+rng.gauss(0, math.radians(.4))],
                'odom': list(pose), 'odom_stamp': clock[0], 'vx': plant['vx'], 'wz': plant['wz'],
                'front': 2., 'cov': [sigma**2, sigma**2, 1e-4]}

    def sleep(dt):
        clock[0] += dt
        x, y, yaw = plant['pose']
        v = plant['vx'] * (1 + rng.gauss(0, slip))
        plant['pose'] = [x+dt*v*math.cos(yaw), y+dt*v*math.sin(yaw),
                         yaw+dt*(plant['wz']*(1+rng.gauss(0, slip)) + heading_bias*(plant['vx'] != 0))]
    def ready(*_, **__):                 # the real ready() ends a pause the same way
        episode = getattr(runner, '_task_episode', None)
        if episode is not None and episode.started is not None:
            episode.resume()
        return state()
    runner.state, runner.ready = state, ready
    runner.settle = lambda: (plant.update(vx=0., wz=0.), state())[1]
    runner.stop = lambda: plant.update(vx=0., wz=0.)
    goals = resolve_destination(policy, list(start[:2]), 'patrol-cycle')
    looks = patrol_looks(policy)
    runner.headings = []
    with patch.object(patrol.time, 'time', side_effect=lambda: clock[0]), \
         patch.object(patrol.time, 'sleep', side_effect=sleep):
        for index, (name, goal) in enumerate(goals):
            runner.goto(goal, name)
            before = len(runner.report['trace'])
            runner.look_from(goals, index, looks)
            if name in looks and index+1 < len(goals):
                runner.headings = [s['odom'][2] for s in runner.report['trace'][before:]]
                runner.look_at = clock[0]
    return runner


@pytest.mark.parametrize('seed', range(12))
def test_a_whole_lap_turns_where_it_stops(seed):
    start = [(.5, .5, 0.), (.40, 1.0, -.4), (.7, .1, .6), (.9, .3, 1.1)][seed % 4]
    runner = drive(Policy(str(SCENE)), start, seed)
    assert len(runner.report['arrivals']) >= 6
    assert len(runner.report['recoveries']) <= 3      # a stop-and-go, well under the 12 allowed


@pytest.mark.parametrize('seed', range(6))
def test_the_far_corner_shows_the_camera_the_room_centre_once_a_lap(seed):
    policy = Policy(str(SCENE))
    runner = drive(policy, [(.5, .5, 0.), (.40, 1.0, -.4)][seed % 2], seed)
    assert [look['label'] for look in runner.report['looks']] == ['outer_ccw_0']
    route = {item['name']: item['point'] for item in json.loads((SCENE/'outer_patrol_goals.json').read_text())}
    one, two = route['outer_ccw_0'], route['outer_ccw_1']
    leg = math.atan2(two[1]-one[1], two[0]-one[0])
    assert len(runner.headings) > 5
    last = runner.report['looks'][0]['pose'][2]
    assert abs(patrol.wrap(last-leg-math.radians(60))) < math.radians(9)
    # it got there turning left, across the next leg's heading, not around the back
    assert all(0 < patrol.wrap(after-before) < math.radians(20) for before, after in
               zip(runner.headings, runner.headings[1:]) if abs(patrol.wrap(after-before)) > .001)
    assert len(runner.report['arrivals']) >= 6


def test_a_look_is_only_taken_with_a_leg_to_go_on_to():
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.face_yaw = lambda *_: pytest.fail('turned at the end of the route')
    runner.look_from([('a', [0, 0]), ('outer_ccw_0', [1, 0])], 1, {'outer_ccw_0': 60.})
    runner.look_from([('a', [0, 0]), ('b', [1, 0])], 0, {'outer_ccw_0': 60.})


def test_the_route_leaves_room_for_an_arrival_error_at_every_turn():
    policy = Policy(str(SCENE))
    route = [item['point'] for item in json.loads((SCENE/'outer_patrol_goals.json').read_text())]
    geometry = EntryGeometry(policy)
    need = BODY_RADIUS + TURN_MARGIN + .065
    for a, b in zip(route, route[1:]):
        for t in range(41):
            point = (a[0]+(b[0]-a[0])*t/40, a[1]+(b[1]-a[1])*t/40)
            assert geometry.box_distance(point) >= need
