"""Feedback-based post-placement exit; standard library only, no hardware on import.

The close-to-bin leg deliberately stays straight. Upstream back_path_problem
and back_straight retain their directional wall/bin checks there. A rotation is
not requested until the stopped centre and its 7cm ring fit the patrol mask.
"""
import json
import math
import subprocess
import time
from pathlib import Path

TURN_MARGIN = .07
MAX_RETREAT = .60
MAX_STEP = .10


def turn_ready(mask, point, margin=TURN_MARGIN):
    return mask.allowed(*point[:2]) and all(
        mask.allowed(point[0] + radius * math.cos(k * math.pi / 8),
                     point[1] + radius * math.sin(k * math.pi / 8))
        for radius in (.06, margin) for k in range(16))


def exit_distance(mask, pose, minimum=0., limit=MAX_RETREAT):
    """A destination with a full turn-clearance ring, not just a legal centre."""
    d = max(0., minimum)
    while d <= limit + 1e-9:
        point = (pose[0] - d * math.cos(pose[2]), pose[1] - d * math.sin(pose[2]))
        if turn_ready(mask, point):
            return d
        d += .01
    return None


def retreat_to_patrol(mask, read, back, path_problem, stop, minimum=.20,
                      limit=MAX_RETREAT, clock=time.monotonic, report=None):
    """Back in <=10cm commands, then verify fresh stopped feedback at the exit.

    read() must validate localization age/covariance, owner/boot/map, empty
    gripper, arm travel, zero wheel feedback and an unowned TCP client. Returned
    positions, not requested distances, determine progress and completion.
    """
    report = {} if report is None else report
    report.update(ok=False, turn_margin_m=TURN_MARGIN, minimum_back_m=minimum,
                  distance_m=0., trace=[])
    if not 0 <= minimum <= limit <= MAX_RETREAT:
        raise ValueError('invalid bounded reentry distance')
    stop()
    try:
        initial = read()
        context = initial['context']
        start = initial['odom']
        previous = initial
        deadline = clock() + 90.
        stable = 0
        stalled = 0
        for _ in range(50):
            state = previous
            if clock() > deadline:
                raise RuntimeError('patrol reentry timed out')
            if state['context'] != context:
                raise RuntimeError('owner/boot/map changed during patrol reentry')
            pose = state['pose']
            odom = state['odom']
            dx, dy = start[0] - odom[0], start[1] - odom[1]
            travelled = dx * math.cos(start[2]) + dy * math.sin(start[2])
            lateral = abs(-dx * math.sin(start[2]) + dy * math.cos(start[2]))
            yaw_error = abs((odom[2] - start[2] + math.pi) % (2 * math.pi) - math.pi)
            if lateral > .04 or yaw_error > math.radians(5) or travelled < -.02:
                raise RuntimeError('straight dock exit deviated; no blind turn or further backing')
            # Allow bounded coast after the last command; never request another
            # command past the nominal budget merely to make up a tiny rest.
            if report['distance_m'] > limit + .05:
                raise RuntimeError('patrol reentry actual-distance budget exceeded')
            report['trace'].append({'pose': pose, 'reverse_m': travelled,
                                    'turn_ready': turn_ready(mask, pose), 'stamp': state['stamp']})
            if travelled >= minimum - .005 and turn_ready(mask, pose):
                stable += 1
                if stable >= 3:
                    report.update(ok=True, final_pose=pose, next_stage='PATROL_REENTRY_READY')
                    return report
                fresh = read()
            else:
                stable = 0
                remaining = limit - report['distance_m']
                needed = exit_distance(mask, pose, max(0., minimum - travelled), remaining)
                if needed is None:
                    raise RuntimeError('no straight exit into a turn-clear patrol region within budget')
                problem = path_problem(pose, needed)
                if problem:
                    raise RuntimeError('patrol reentry corridor refused: ' + str(problem))
                step = min(MAX_STEP, needed)
                if step < .005:
                    raise RuntimeError('no measurable retreat step toward patrol region')
                back(step)
                fresh = read()
                moved = math.hypot(fresh['odom'][0] - odom[0], fresh['odom'][1] - odom[1])
                report['distance_m'] += moved
                stalled = stalled + 1 if moved < .005 else 0
                if stalled >= 3:
                    raise RuntimeError('patrol reentry made no measured progress')
            if fresh['stamp'] <= state['stamp']:
                raise RuntimeError('patrol reentry requires a new localization/odometry sample')
            previous = fresh
        raise RuntimeError('patrol reentry iteration budget exhausted')
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'interrupted'
        raise
    finally:
        stop()


def bind_mission(mission, scene, report_folder, health, recovery=None, placement=None):
    """Install the adapter on a coherent recent koala Room, leaving PPO unchanged."""
    required = [(mission, 'Room'), (mission.dock, 'fetch_nav_mask'),
                (mission, 'place_held'), (mission.dock, 'back_path_problem'),
                (mission.runner, 'back_straight')]
    if any(not hasattr(obj, name) for obj, name in required):
        raise RuntimeError('post-place reentry requires koala loop-opt-2026-10-21 or newer')
    original = mission.Room
    original_place = mission.place_held
    post_place = {'pending': False}
    rooms = {}
    folder = Path(report_folder)

    class FeedbackRoom(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            if self.mask is None:
                raise RuntimeError('post-place reentry requires the active navigation mask')
            self.reentry_used = 0.
            self.reentry_reports = []
            self._last_snapshot = None
            self._anchor = None
            self.recovery_events = []
            rooms[self.host] = self
            if recovery is not None:
                self._snapshot()

        def _fault(self, code, text, retryable=False, record=None):
            if recovery is None:
                return RuntimeError(text)
            fault = recovery.localization_fault(record) if record else None
            return recovery.RecoveryFault(code, text, retryable,
                fault.elapsed if fault else 0., fault.context if fault else None)

        def _snapshot_once(self, holding=False):
            try:
                h = health(self.host)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                raise self._fault('HEALTH_UNAVAILABLE', str(exc), True)
            ch = h.get('chassis', {})
            if h.get('holding') is not holding or ch.get('client') is not None:
                raise self._fault('ACTION_STATE_UNKNOWN', 'reentry requires known holding state and no other client')
            if (h.get('ok') is not True or ch.get('arm_pose') != 'travel'
                    or ch.get('arm_busy') is not False or ch.get('moving') is not False
                    or ch.get('motors') != [0, 0, 0, 0]):
                raise self._fault('TASK_NOT_STOPPED', 'post-place reentry requires stopped travel state', True)
            try:
                loc = mission.dock.read_localization(self.host)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                raise self._fault('SUPERVISOR_STALE', str(exc), True)
            if (not isinstance(loc, dict) or not isinstance(loc.get('age'), (int, float))
                    or not 0 <= loc['age'] <= .4):
                raise self._fault('SUPERVISOR_STALE', 'post-place reentry localization is stale', True)
            if loc.get('ready') is not True:
                if recovery is None:
                    raise RuntimeError('post-place reentry waiting for localization')
                raise recovery.localization_fault(loc)
            pose = mission.dock.localization_pose(loc)
            state = loc.get('state', {})
            cov = state.get('cov')
            if (not isinstance(cov, (tuple, list)) or len(cov) != 3
                    or any(not math.isfinite(float(v)) or v < 0 for v in cov)
                    or max(cov[:2]) > (.04 / 3) ** 2
                    or loc.get('age', float('inf')) > .4):
                raise self._fault('AMCL_UNCERTAIN', 'post-place reentry localization exceeds freshness/uncertainty budget', True, loc)
            stamp = state.get('odom_stamp')
            odom = state.get('odom')
            vx, wz = state.get('vx', float('inf')), state.get('wz', float('inf'))
            if (not isinstance(stamp, (float, int)) or not math.isfinite(stamp)
                    or not isinstance(odom, (tuple, list)) or len(odom) != 3
                    or any(not math.isfinite(float(v)) for v in odom)
                    or not loc.get('boot_id') or not loc.get('map_hash')
                    or h.get('service_pid') is None or loc.get('owner_pid') != h['service_pid']
                    or not math.isfinite(vx) or not math.isfinite(wz)
                    or abs(vx) >= .01 or abs(wz) >= .02):
                raise self._fault('CONTEXT_OR_FEEDBACK_INVALID', 'post-place reentry owner/odometry/stop evidence invalid')
            return {'pose': pose, 'odom': odom, 'stamp': stamp,
                    'context': (loc['boot_id'], loc['owner_pid'], loc['map_hash'])}

        def _snapshot(self, holding=False):
            if recovery is None:
                return self._snapshot_once(holding=holding)
            value = recovery.wait_snapshot(lambda: self._snapshot_once(holding=holding), lambda: mission.runner.stop(self.host),
                                           previous=self._last_snapshot, events=self.recovery_events,
                                           anchor=self._anchor)
            self._last_snapshot = self._anchor = value
            return value

        def _leave_ready(self, minimum, holding=False):
            # The grasp/placement controllers moved the robot since the last
            # snapshot: a jump test against it would only measure their drift.
            self._anchor = None
            result = {}
            try:
                retreat_to_patrol(
                    self.mask, lambda: self._snapshot(holding=holding), lambda metres: self._back(metres),
                    lambda pose, metres: mission.dock.back_path_problem(
                        pose, metres, self.walls, obstacles=self.obstacles),
                    lambda: mission.runner.stop(self.host), minimum=minimum,
                    limit=MAX_RETREAT - self.reentry_used, report=result)
            finally:
                self.reentry_used += result.get('distance_m', 0.)
                result['recovery_events'] = list(self.recovery_events)
                self.reentry_reports.append(result)
                folder.mkdir(parents=True, exist_ok=True)
                (folder / 'post_place_reentry.json').write_text(
                    json.dumps(self.reentry_reports, indent=2), encoding='utf-8')
            self.backed, self.extra = self.reentry_used, 0.
            print('[REENTRY] centre and 7cm turn reserve ready; actual retreat {:.2f}m'.format(
                result['distance_m']), flush=True)
            return result

        def leave(self, metres=None):
            # Successful placement has a geometric completion condition, with
            # no mandatory 20cm. Explicit failed-grasp visibility distances and
            # the existing failed-grasp default remain intact.
            minimum = metres if metres is not None else (0. if post_place['pending'] else mission.BACK_OUT_M)
            result = self._leave_ready(minimum)
            post_place['pending'] = False
            return result

        def make(self):
            if self.backed is None:
                return False
            return self._leave_ready(0.)['distance_m'] > .005

        def reset(self):
            super().reset()
            self._anchor = None
            self.reentry_used = 0.
            post_place['pending'] = False
            if recovery is not None:
                # Capture a trusted context before the next grasp/release,
                # rather than treating a changed owner after a fault as normal.
                self._snapshot()

    def place_held(*args, **kwargs):
        result = (supervisor.execute(*args, **kwargs) if placement is not None
                  else original_place(*args, **kwargs))
        post_place['pending'] = True
        return result

    mission.Room = FeedbackRoom
    mission.place_held = place_held
    supervisor = placement.Supervisor(mission, rooms, scene, folder) if placement is not None else None
    patrol_recovery = recovery.bind_patrol(mission, rooms, folder) if recovery is not None else False
    return {'adapter': 'g2_patrol_reentry', 'turn_margin_m': TURN_MARGIN,
            'max_step_m': MAX_STEP, 'max_retreat_m': MAX_RETREAT,
            'completion': 'three fresh stopped poses inside mask with turn reserve', 'scene': scene,
            'patrol_recovery_installed': patrol_recovery, 'task_recovery_wait_s': 30 if recovery else 0,
            'placement_corridor_installed': supervisor is not None}
