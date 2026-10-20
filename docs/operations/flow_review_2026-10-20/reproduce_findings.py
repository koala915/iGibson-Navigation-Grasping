"""Read-only reproductions of flow review findings; no robot or network access.

Assertions confirm the problematic current behavior, not an acceptance pass.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('flow_review_fakes', ROOT / 'tests/test_mission_loop.py')
fakes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fakes)
mission = fakes.mission


def rejected_candidate_allows_patrol():
    clock, starts = [100.0], []

    class Watcher:
        calls = 0

        def look(self, frame):
            self.calls += 1
            return (1.0, 0.0), False

    class StartRequested(Exception):
        pass

    watcher = Watcher()

    def start(*args):
        starts.append(watcher.calls)
        raise StartRequested()

    with patch.object(mission, 'confirm_parked', return_value=None), \
            patch.object(mission, 'start_patrol', side_effect=start), patch('builtins.print'):
        try:
            mission.patrol_until_sugarbox(
                'mock', 'scene', fakes.ClockReader(clock), watcher, fakes.FakeLink(),
                now=lambda: clock[0])
        except StartRequested:
            pass
    assert starts == [6]
    return {'start_requested_after_frames': starts[0], 'box_visible_in_every_frame': True}


def search_does_not_check_turn_clearance():
    clock, turns = [100.0], []

    class Watcher:
        def look(self, frame):
            return ((1.5, 0.0), False) if turns else None

    with patch.object(mission.runner.dock, 'read_localization', return_value={
            'ready': True, 'age': 0.0,
            'state': {'pose': [0.0, 0.0, 0.0], 'turn_clear': False}}) as localization, \
            patch('builtins.print'):
        found = mission.search_placement(
            'mock', fakes.ClockReader(clock), Watcher(),
            turn=lambda host, angle: turns.append(angle), now=lambda: clock[0])
    assert found and len(turns) == 1 and localization.call_count == 0
    return {'turns_requested': len(turns), 'turn_clear': False,
            'localization_checks': localization.call_count}


def pose_and_frame_timestamp_mismatch():
    walls = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    relative_sighting = (0.8, 0.4)
    at_capture = mission.map_point((0.0, 0.2, 0.0), relative_sighting)
    at_processing = mission.map_point((0.0, 0.2, 0.5), relative_sighting)
    capture_ignored = mission.passed_over(at_capture, None, (), (), walls)
    processing_ignored = mission.passed_over(at_processing, None, (), (), walls)
    assert not capture_ignored and processing_ignored
    return {'capture_pose_yaw_rad': 0.0, 'processing_pose_yaw_rad': 0.5,
            'actual_map_target': at_capture, 'incorrect_map_target': at_processing,
            'correct_target_ignored': capture_ignored,
            'incorrect_target_ignored_as_wall': processing_ignored}


def custom_assets_not_forwarded():
    args = argparse.Namespace(host='mock', wall_m=None,
                              homography=Path('CUSTOM_CALIBRATION.json'),
                              gstreamer=Path('CUSTOM_GST.exe'))
    calls = []
    with patch.object(mission, 'ensure_arm_camera', return_value=None), \
            patch.object(mission, 'unlock'), \
            patch.object(mission, 'approach_seed', return_value=(1.0, 0.0)), \
            patch.object(mission, 'run_stage', side_effect=lambda host, argv, log=None:
                         calls.append(argv) or 0), \
            patch.object(mission, 'arm_state', return_value='holding'):
        assert mission.fetch_sugarbox(args, {'point': (1.0, 0.0), 'map': None}, OUT, 1) == 'held'
    assert '--homography' not in calls[0] and '--gstreamer' not in calls[0]
    return {'requested_homography': str(args.homography),
            'requested_gstreamer': str(args.gstreamer), 'stage_arguments': calls[0]}


def resume_place_does_not_unlock():
    args = argparse.Namespace(host='mock', wall_m=None, resume_at='place')
    unlocked = []

    def ctl(host, command):
        if command == 'status':
            return {'chassis': {'arm_pose': 'travel', 'maintenance': not unlocked}}
        return {'holding': True}

    with tempfile.TemporaryDirectory(dir=OUT) as folder, \
            patch.object(mission, 'stream_running', return_value=True), \
            patch.object(mission, 'run_stage', side_effect=lambda *args: 0 if unlocked else 2), \
            patch.object(mission.runner, 'ctl', side_effect=ctl), \
            patch.object(mission.runner, 'service_command',
                         side_effect=lambda host, cmd: unlocked.append(cmd)), \
            patch.object(mission.runner, 'stop'), patch('builtins.print'):
        code = mission.grasp_and_place(args, 'mock', Path(folder))
    assert code == 2 and not unlocked
    return {'initial_maintenance_lock': True, 'unlock_commands': unlocked, 'mission_exit': code}


def checkout_service_protocol_gap():
    spec = importlib.util.spec_from_file_location('review_graspctl', ROOT / 'grasp/v23/graspctl.py')
    client = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(client)
    missing = sorted(set(mission.runner.ARM_COMMANDS) - set(client.TIMEOUTS))
    with patch('sys.stderr'):
        code = client.main(['graspctl.py', 'health'])
    assert code == 2 and 'health' in missing
    return {'runner_commands_missing_in_checkout_client': missing, 'health_exit': code,
            'scope': 'checkout deployment; deployed ncu service may provide these commands'}


if __name__ == '__main__':
    checks = [rejected_candidate_allows_patrol, search_does_not_check_turn_clearance,
              pose_and_frame_timestamp_mismatch, custom_assets_not_forwarded,
              resume_place_does_not_unlock, checkout_service_protocol_gap]
    result = {fn.__name__: fn() for fn in checks}
    result['source_sha256'] = {
        name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        for name in ['integration/mission_loop.py', 'integration/sugarbox_approach_grasp.py',
                     'integration/placement_dock.py', 'grasp/v23/graspctl.py']}
    (OUT / 'reproductions.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
