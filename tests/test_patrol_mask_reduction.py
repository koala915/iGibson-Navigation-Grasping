"""Offline replay of run 19 plus collision-boundary checks for newly freed cells."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration.g2_patrol_policy import Policy
from integration.rebuild_patrol_mask import build
from integration import mission_loop
from integration import placement_dock
from unittest.mock import Mock, patch

BEFORE = ROOT/'docs/operations/patrol_mask_20261021/before'
SCENE = BEFORE/'maps/demo_box_20261018_patrol'


class MaskReductionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)/'scene'
        self.report = build(SCENE, self.output)
        self.old = Policy(str(SCENE))
        self.new = Policy(str(self.output))

    def ring(self, policy, pose):
        return all(policy.allowed(pose[0]+.06*math.cos(i*math.pi/360),
                                  pose[1]+.06*math.sin(i*math.pi/360)) for i in range(720))

    def test_run19_turn_refusal_is_resolved_without_any_motion(self):
        record = json.loads((BEFORE/'mission_patrol.json').read_text())
        self.assertEqual(record['error'], 'cannot enter turn reserve')
        self.assertEqual(record['pulses'], 0)
        pose = record['initial']['pose']
        self.assertTrue(self.old.allowed(*pose[:2]))
        self.assertFalse(self.ring(self.old, pose))
        self.assertTrue(self.ring(self.new, pose))
        self.assertTrue(self.new.segment_allowed(pose[:2], record['requested_goals'][0]['point']))

    def test_exit_planner_accounts_for_run19_turn_reserve_even_with_old_mask(self):
        pose = json.loads((BEFORE/'mission_patrol.json').read_text())['initial']['pose']
        self.assertTrue(self.old.allowed(*pose[:2]))
        self.assertFalse(mission_loop.patrol_start_allowed(self.old, pose[:2]))
        distance = mission_loop.mask_exit(self.old, pose, .01)
        self.assertIsNotNone(distance)
        self.assertLessEqual(distance, mission_loop.ROOM_EXIT_MAX_M)
        point = (pose[0]-distance*math.cos(pose[2]), pose[1]-distance*math.sin(pose[2]))
        self.assertTrue(mission_loop.patrol_start_allowed(self.old, point))
        self.assertTrue(self.ring(self.old, point))
        self.assertTrue(self.old.segment_allowed(point, json.loads(
            (BEFORE/'mission_patrol.json').read_text())['requested_goals'][0]['point']))

    def test_postplacement_exit_and_relocalized_restart_share_turn_clearance(self):
        fixture = json.loads((BEFORE.parent/'postplace_fixture.json').read_text())
        self.assertEqual(fixture['run19_odom'], fixture['run20_odom'])
        retreat = sum(fixture['retreat_steps_m'])
        obstacle = placement_dock.bin_obstacle(placement_dock.box_from_layout(self.new.layout))
        for key, other in [('run19_pose','run20_pose'), ('run20_pose','run19_pose')]:
            stopped, relocated = fixture[key], fixture[other]
            position = [stopped[0]+retreat*math.cos(stopped[2]),
                        stopped[1]+retreat*math.sin(stopped[2]), stopped[2]]
            moves = []

            def back(host, metres):
                self.assertIsNone(placement_dock.back_path_problem(
                    position, metres, self.new.layout['room_polygon'],
                    placement_dock.BACK_WALL_MARGIN_M, [obstacle]))
                actual = metres-.03 if not moves else metres
                position[0] -= actual*math.cos(position[2])
                position[1] -= actual*math.sin(position[2])
                moves.append(actual)
                return actual

            room = mission_loop.Room('fake', mask=self.new, back=back,
                                     locate=lambda host: tuple(position))
            with self.subTest(basis=key), patch('builtins.print'):
                room.leave()
            self.assertTrue(mission_loop.patrol_exit_allowed(self.new, position))
            self.assertLessEqual(sum(moves), mission_loop.ROOM_EXIT_MAX_M)
            # Apply the observed change in map position/heading after fresh
            # localization. It must still pass the patrol's admission test.
            shifted = [position[0]+relocated[0]-stopped[0],
                       position[1]+relocated[1]-stopped[1], relocated[2]]
            self.assertTrue(mission_loop.patrol_start_allowed(self.new, shifted[:2]))
            goal = json.loads((SCENE/'outer_patrol_goals.json').read_text())[2]['point']
            self.assertTrue(self.new.plan(shifted[:2], goal))

    def test_two_deliveries_continue_to_next_loop_without_relocalizing(self):
        fixture = json.loads((BEFORE.parent/'postplace_fixture.json').read_text())
        stopped = fixture['run20_pose']
        retreat = sum(fixture['retreat_steps_m'])
        release = [stopped[0]+retreat*math.cos(stopped[2]),
                   stopped[1]+retreat*math.sin(stopped[2]), stopped[2]]
        position, events, back_count = release[:], [], [0]
        obstacle = placement_dock.bin_obstacle(placement_dock.box_from_layout(self.new.layout))
        calls = [0]

        def back(host, metres):
            self.assertIsNone(placement_dock.back_path_problem(
                position, metres, self.new.layout['room_polygon'],
                placement_dock.BACK_WALL_MARGIN_M, [obstacle]))
            # Model each delivery's first retreat stopping 3cm short.
            actual = metres-.03 if back_count[0] == 0 else metres
            back_count[0] += 1
            position[0] -= actual*math.cos(position[2])
            position[1] -= actual*math.sin(position[2])
            events.append('back')
            return actual

        room = mission_loop.Room('fake', mask=self.new, back=back,
                                 locate=lambda host: tuple(position))

        def patrol(*args, **kwargs):
            calls[0] += 1
            events.append('patrol')
            if calls[0] > 1:
                self.assertTrue(mission_loop.patrol_exit_allowed(self.new, position))
                self.assertTrue(mission_loop.patrol_start_allowed(self.new, position[:2]))
                self.assertEqual(room.extra, 0)  # exit was ready before restarting patrol
            return {'point': (1.,.1), 'map': (0.,0.)} if calls[0] <= 2 else None

        def place(*args, **kwargs):
            events.append('place')
            position[:] = release
            back_count[0] = 0

        args = mission_loop.argparse.Namespace(host='fake', scene='fixture',
            homography=Path('fake'), gstreamer=None, wall_m=None, max_deliveries=None)
        with tempfile.TemporaryDirectory() as folder, patch('builtins.print'), \
             patch.object(mission_loop.dock, 'fetch_layout', return_value=self.new.layout), \
             patch.object(mission_loop, 'stream_running', return_value=True), \
             patch.object(mission_loop, 'SugarboxWatcher'), \
             patch('integration.rear_stream.StreamReader'), \
             patch.object(mission_loop, 'patrol_until_sugarbox', side_effect=patrol), \
             patch.object(mission_loop, 'fetch_sugarbox', return_value='held'), \
             patch.object(mission_loop, 'place_held', side_effect=place), \
             patch.object(mission_loop.runner, 'stop'), \
             patch.object(mission_loop, 'relocalize') as reloc:
            code = mission_loop.patrol_mission(args, 'fake', Path(folder), room=room,
                                              new_link=lambda host: Mock())
        self.assertEqual(code, 0)
        self.assertEqual(calls[0], 3)
        self.assertEqual(events.count('place'), 2)
        self.assertGreaterEqual(events.count('back'), 4)
        reloc.assert_not_called()

    def test_old_route_and_allowed_cells_are_preserved(self):
        old = np.asarray(self.old.cells)
        new = np.asarray(self.new.cells)
        self.assertTrue(np.all(new[old > 250] > 250))
        route = json.loads((SCENE/'outer_patrol_goals.json').read_text())
        for a, b in zip(route, route[1:]+route[:1]):
            self.assertTrue(self.new.segment_allowed(a['point'], b['point']))
        self.new.require_motion_ready(.10)
        self.assertEqual(self.new.patrol_points(.70), self.old.patrol_points(.70))

    def test_every_added_cell_keeps_wall_bin_and_raw_map_clearance(self):
        meta = yaml.safe_load((SCENE/'demo_box.yaml').read_text())
        raw = cv2.imread(str(SCENE/meta['image']), 0)
        old = np.asarray(self.old.cells).reshape(raw.shape)
        new = np.asarray(self.new.cells).reshape(raw.shape)
        added = np.argwhere((new > 250) & (old <= 250))
        self.assertGreater(len(added), 0)
        a = self.new.layout['room_heading_rad']
        c, s = math.cos(a), math.sin(a)
        poly = [(x*c+y*s, -x*s+y*c) for x,y in self.new.layout['room_polygon']]
        env = self.new.layout['navigation_mask_validation']['envelope_room_uv']
        resolution = self.new.resolution
        for row, col in added:
            self.assertGreater(raw[row, col], 255*(1-meta['free_thresh']))
            # Independent exact distance to nearby closed occupied/unknown
            # squares; evaluate all nine corners/edge-centres/centre points.
            r0, r1 = max(0, row-20), min(raw.shape[0], row+21)
            c0, c1 = max(0, col-20), min(raw.shape[1], col+21)
            bad = np.argwhere(raw[r0:r1, c0:c1] <= 255*(1-meta['free_thresh']))
            for dr in (0, .5, 1):
                for dc in (0, .5, 1):
                    x = meta['origin'][0]+(col+dc)*resolution
                    y = meta['origin'][1]+(raw.shape[0]-row-dr)*resolution
                    u, v = x*c+y*s, -x*s+y*c
                    wall = min(u-min(p[0] for p in poly), max(p[0] for p in poly)-u,
                               v-min(p[1] for p in poly), max(p[1] for p in poly)-v)
                    self.assertGreaterEqual(wall+1e-7, .35)
                    box = math.hypot(max(env[0]-u, 0, u-env[1]), max(env[2]-v, 0, v-env[3]))
                    self.assertGreaterEqual(box+1e-7, self.report['bin_buffer_m'])
                    if len(bad):
                        dx = np.maximum(np.abs(col+dc-(bad[:,1]+c0+.5))-.5, 0)
                        dy = np.maximum(np.abs(row+dr-(bad[:,0]+r0+.5))-.5, 0)
                        self.assertGreaterEqual(float(np.hypot(dx,dy).min())*resolution+1e-7, .35)

    def test_mask_cannot_be_reduced_below_floor_or_overwrite_source(self):
        for margin in (.0, .30, .349, float('nan')):
            with self.assertRaises(ValueError):
                build(SCENE, self.output, margin)
        with self.assertRaises(ValueError):
            build(SCENE, SCENE)

    def test_reduced_scene_needs_matching_approval_and_image(self):
        path = self.output/'navigation_center_mask.pgm'
        data = bytearray(path.read_bytes()); data[-1] ^= 1; path.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'approval/geometry'):
            Policy(str(self.output))
        shutil.copy2(SCENE/'navigation_center_mask.pgm', path)
        with self.assertRaises(ValueError):
            Policy(str(self.output))

    def test_missing_approval_and_below_floor_are_rejected(self):
        path = self.output/'scene_layout.json'
        layout = json.loads(path.read_text())
        del layout['navigation_mask_reduction_20261021']
        path.write_text(json.dumps(layout))
        with self.assertRaises(ValueError):
            Policy(str(self.output))
        layout['center_wall_margin_m'] = .30
        path.write_text(json.dumps(layout))
        with self.assertRaises(ValueError):
            Policy(str(self.output))

    def test_rebuild_is_reproducible_and_source_is_unchanged(self):
        actual = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        self.assertEqual(actual(SCENE/'navigation_center_mask.pgm'), self.report['before_sha256'])
        self.assertEqual(actual(self.output/'navigation_center_mask.pgm'), self.report['sha256'])
        other = build(SCENE, Path(self.temp.name)/'again')
        self.assertEqual(other, self.report)


if __name__ == '__main__':
    unittest.main(verbosity=2)
