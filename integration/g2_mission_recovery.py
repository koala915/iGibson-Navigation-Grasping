"""Bounded task-side waits while the independent AMCL supervisor recovers.

No arm commands or service restarts. A failed sensor read never restarts an
entire grasp/release stage or replays its last relative velocity command.
"""
import math
import json
from pathlib import Path
import time


class RecoveryFault(RuntimeError):
    def __init__(self, code, detail, retryable=False, elapsed=0., context=None):
        super().__init__('{}: {}'.format(code, detail))
        self.code, self.retryable = code, retryable
        self.elapsed, self.context = max(0., elapsed), context


def continuity(previous, current):
    if previous is None:
        return
    if 'context' in previous and previous['context'] != current['context']:
        raise RecoveryFault('CONTEXT_CHANGED', 'owner/boot/map changed; re-anchor the task')
    old, new = previous['odom'], current['odom']
    p, q = previous['pose'], current['pose']
    angle = p[2] - old[2]
    dx, dy = new[0] - old[0], new[1] - old[1]
    predicted = (p[0] + math.cos(angle) * dx - math.sin(angle) * dy,
                 p[1] + math.sin(angle) * dx + math.cos(angle) * dy,
                 p[2] + new[2] - old[2])
    yaw = abs((predicted[2] - q[2] + math.pi) % (2 * math.pi) - math.pi)
    if math.hypot(predicted[0] - q[0], predicted[1] - q[1]) > .15 or yaw > math.radians(10):
        raise RecoveryFault('REFERENCE_REQUIRED', 'AMCL/odometry continuity lost; no automatic task replay')


def wait_snapshot(read, stop, previous=None, seconds=30., clock=time.monotonic,
                  sleep=time.sleep, events=None, anchor=None):
    """Normally one read; after a transient outage require five new stable poses.

    Count the supervisor's existing outage time against this same episode. A
    daemon already waiting 20s gets at most another 10s, not a fresh 30s.

    `previous` binds the owner/boot/map context. The AMCL/odometry jump test runs
    only across an outage, against `anchor`: the last good stopped state just
    before it. Against an older snapshot (before a grasp, a placement or a patrol
    lap) ordinary odometry drift alone exceeds 15cm/10deg and would stop a
    healthy task.
    """
    if not 0 < seconds <= 60:
        raise ValueError('recovery wait must be 0..60 seconds')
    started, offset = clock(), 0.
    waiting, stable, last, first = False, 0, None, None
    expected = previous['context'] if previous else None
    history = [] if events is None else events
    for _ in range(1000):
        try:
            value = read()
            if expected is not None and value['context'] != expected:
                raise RecoveryFault('CONTEXT_CHANGED', 'owner/boot/map changed while stopped')
            if not waiting:
                return value
            continuity(anchor, value)
            if clock() - started + offset >= seconds:
                raise RecoveryFault('RECOVERY_TIMEOUT', 'localization recovery budget exhausted')
            if last is not None and value['stamp'] < last['stamp']:
                raise RecoveryFault('ODOM_TIME_REVERSED', 'odometry clock changed during recovery')
            if first is not None and (math.hypot(value['pose'][0] - first['pose'][0],
                                                value['pose'][1] - first['pose'][1]) > .03
                    or abs((value['pose'][2] - first['pose'][2] + math.pi) % (2 * math.pi) - math.pi)
                    > math.radians(3)):
                stable, first = 0, None
            if last is None or value['stamp'] > last['stamp']:
                stable += 1
                first = first or value
            last = value
            if stable >= 5:
                history.append({'status': 'READY_TO_REPLAN', 'stable_samples': stable,
                                'elapsed_s': clock() - started + offset})
                return value
        except RecoveryFault as exc:
            if not exc.retryable:
                raise
            if expected is not None and exc.context is not None and exc.context != expected:
                raise RecoveryFault('CONTEXT_CHANGED', 'supervisor context changed during recovery')
            expected = expected or exc.context
            if not waiting:
                stop()
                waiting = True
                history.append({'status': 'WAIT_LOCALIZATION', 'reason_code': exc.code})
            elapsed = clock() - started
            offset = max(offset, exc.elapsed - elapsed)
            stable, last, first = 0, None, None
            if elapsed + offset >= seconds:
                raise RecoveryFault('RECOVERY_TIMEOUT', '{}; supervisor/task wait exceeded {}s'.format(exc.code, seconds))
        sleep(.1)
    raise RecoveryFault('RECOVERY_TIMEOUT', 'recovery polling budget exhausted')


TRANSIENT_CODES = frozenset(('AMCL_STARTING', 'AMCL_STALE', 'AMCL_UNCERTAIN', 'SCAN_STALE',
    'ODOM_STALE', 'TF_STALE', 'BOARD_STALE', 'SUPERVISOR_FAULT', 'LOCALIZATION_NOT_READY',
    'RESTART_BACKOFF', 'RAM_PRESSURE'))


def localization_fault(record):
    record = record or {}
    code = record.get('reason_code') or 'LOCALIZATION_NOT_READY'
    if code == 'REFERENCE_REQUIRED' or record.get('status') == 'needs_reference':
        code = 'REFERENCE_REQUIRED'
    elapsed = record.get('wait_elapsed_s', 0.)
    if not isinstance(elapsed, (float, int)) or not math.isfinite(elapsed):
        elapsed = 30.
    context = (record.get('boot_id'), record.get('owner_pid'), record.get('map_hash'))
    if not all(v is not None for v in context):
        context = None
    return RecoveryFault(code, record.get('reason', 'waiting for supervisor'),
                         code in TRANSIENT_CODES, elapsed, context)


def bind_patrol(mission, rooms, folder):
    """Retry a stopped patrol once; never restart the whole delivery mission."""
    if not hasattr(mission, 'patrol_until_sugarbox'):
        return False
    original = mission.patrol_until_sugarbox
    events = []

    def patrol(*args, **kwargs):
        host = args[0] if args else kwargs['host']
        call_args, call_kwargs = list(args), dict(kwargs)
        temporary_link = None
        try:
            for attempt in range(2):
                try:
                    return original(*call_args, **call_kwargs)
                except mission.MissionError as exc:
                    message = str(exc).lower()
                    link_failure = 'vision permit link to the robot ended' in message
                    localization = any(token in message for token in (
                        'localiz', 'stale amcl', 'stale map tf', 'stale scan/odom',
                        'missing ros data', 'map tf:', 'amcl uncertainty',
                        'amcl no-motion update worker stuck'))
                    if attempt or not (link_failure or localization):
                        raise
                    room = rooms.get(host)
                    if room is None:
                        raise RecoveryFault('CHECKPOINT_MISSING', 'no trusted patrol checkpoint')
                    # The upstream finally block has stopped and joined its own
                    # native patrol. Do not create a second motion owner.
                    report = mission.patrol_report(host)
                    if not isinstance(report, dict):
                        raise RecoveryFault('CHECKPOINT_INVALID', 'patrol report is not an object')
                    state = report.get('last_localization') or {}
                    if not isinstance(state, dict):
                        raise RecoveryFault('CHECKPOINT_INVALID', 'localization report is not an object')
                    elapsed = max((v for v in (report.get('sensor_wait_elapsed_s'), state.get('wait_elapsed_s'))
                                   if isinstance(v, (float, int)) and math.isfinite(v) and v >= 0), default=0.)
                    if state.get('reason_code') == 'REFERENCE_REQUIRED' or state.get('status') == 'needs_reference':
                        raise localization_fault(state)
                    first = [True]
                    def read():
                        if first[0]:
                            first[0] = False
                            raise RecoveryFault('AMCL_STALE' if localization else 'PERMIT_LINK_FAILED',
                                                str(exc), True, elapsed)
                        return room._snapshot_once()
                    trace = report.get('trace')
                    last = trace[-1] if isinstance(trace, list) and trace else None
                    anchor = (last if isinstance(last, dict) and 'pose' in last
                              and 'odom' in last else None)
                    value = wait_snapshot(read, lambda: mission.runner.stop(host),
                                          previous=room._last_snapshot, events=events,
                                          anchor=anchor)
                    room._last_snapshot = value
                    link = call_args[4] if len(call_args) > 4 else call_kwargs['link']
                    if link_failure or not link.alive():
                        link.close()
                        temporary_link = mission.VisionLink(host)
                        if len(call_args) > 4:
                            call_args[4] = temporary_link
                        else:
                            call_kwargs['link'] = temporary_link
                    events.append({'status': 'PATROL_REDISPATCH', 'attempt': attempt + 1,
                                   'pose': value['pose'], 'arm_actions_replayed': False})
        finally:
            if temporary_link is not None:
                temporary_link.close()
            if events:
                Path(folder).mkdir(parents=True, exist_ok=True)
                (Path(folder) / 'patrol_recovery.json').write_text(json.dumps(events, indent=2), encoding='utf-8')

    mission.patrol_until_sugarbox = patrol
    return True
