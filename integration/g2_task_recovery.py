"""Shared, hardware-free recovery contracts (also importable by ROS Python 2).

Waiting is charged across calls until a leg AND its observation are complete.
Nothing here authorizes a motor command, starts a service, or replays an action.
"""
from __future__ import division, print_function
import io
import json
import math
import os
import time
from collections import deque

TRANSIENT = frozenset(('AMCL_STARTING', 'AMCL_STALE', 'AMCL_UNCERTAIN',
    'LOCALIZATION_NOT_READY', 'SCAN_STALE', 'ODOM_STALE', 'TF_STALE',
    'SUPERVISOR_STALE', 'SUPERVISOR_FAULT', 'RESTART_BACKOFF', 'PERMIT_LINK_FAILED'))


def monotonic():
    if hasattr(time, 'monotonic'):
        return time.monotonic()
    # Melodic's Python 2 has no time.monotonic. Do not use the wall clock for
    # recovery deadlines (NTP corrections must not extend a motion lease).
    import ctypes
    class Timespec(ctypes.Structure):
        _fields_ = [('seconds', ctypes.c_long), ('nanos', ctypes.c_long)]
    value = Timespec()
    if ctypes.CDLL(None).clock_gettime(1, ctypes.byref(value)):
        raise RuntimeError('monotonic clock unavailable')
    return value.seconds + value.nanos / 1e9


def finite(values):
    return all(isinstance(v, (int, float)) and not math.isnan(v) and not math.isinf(v) for v in values)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Fault(RuntimeError):
    def __init__(self, code, detail, retryable=False, elapsed=0., context=None):
        RuntimeError.__init__(self, '{}: {}'.format(code, detail))
        self.code, self.retryable = code, retryable
        self.elapsed, self.context = max(0., elapsed), context


def continuity(previous, current):
    if previous is None:
        return
    if previous.get('context') is not None and tuple(previous['context']) != tuple(current['context']):
        raise Fault('CONTEXT_CHANGED', 'boot/owner/map changed')
    p, q, a, b = previous['pose'], current['pose'], previous['odom'], current['odom']
    angle = p[2] - a[2]
    dx, dy = b[0] - a[0], b[1] - a[1]
    expected = (p[0] + math.cos(angle) * dx - math.sin(angle) * dy,
                p[1] + math.sin(angle) * dx + math.cos(angle) * dy, p[2] + b[2] - a[2])
    if (math.hypot(expected[0] - q[0], expected[1] - q[1]) > .15
            or abs(wrap(expected[2] - q[2])) > math.radians(10)):
        raise Fault('REFERENCE_REQUIRED', 'AMCL/odometry continuity lost')


def atomic(path, value):
    temporary = str(path) + '.tmp'
    with io.open(temporary, 'w', encoding='utf-8') as stream:
        stream.write(json.dumps(value, allow_nan=False))
    for attempt in range(5):
        try:
            getattr(os, 'replace', os.rename)(temporary, str(path))
            return
        except OSError:
            if attempt == 4:
                raise
            time.sleep(.002)


class Episode:
    """One cumulative wait allowance per unfinished task leg, not per read."""
    def __init__(self, seconds=30., clock=monotonic, emit=None):
        if not 0 < seconds <= 30:
            raise ValueError('recovery allowance must be 0..30 seconds')
        self.limit, self.clock, self.emit = seconds, clock, emit
        self.used, self.remote, self.started, self.inherited = 0., 0., None, 0.
        self.pending = False
        self.number = 0
        self.status = 'RUNNING'

    def event(self, status, **fields):
        self.status = status
        if self.emit:
            self.emit(dict(status=status, episode=self.number, wait_used_s=self.elapsed(), **fields))

    def elapsed(self):
        return self.used + (max(self.clock() - self.started, self.inherited) if self.started is not None else 0.)

    def check(self):
        if self.elapsed() >= self.limit:
            self.event('RECOVERY_TIMEOUT')
            raise Fault('RECOVERY_TIMEOUT', 'shared localization/observation allowance exhausted')

    def pause(self, code, elapsed=0., began=None):
        if self.started is None:
            self.started = self.clock() if began is None else min(self.clock(), began)
            self.inherited = 0.
            self.number += 1
            self.pending = True
            self.event('WAIT_LOCALIZATION', reason_code=code)
        if not finite([elapsed]) or elapsed < 0:
            raise Fault('CHECKPOINT_INVALID', 'invalid supervisor wait time')
        self.inherited = max(self.inherited, elapsed - self.remote)
        self.check()

    def resume(self):
        self.check()
        self.used = self.elapsed()
        self.remote += self.inherited
        self.started, self.inherited = None, 0.
        self.event('REVALIDATE_PATH')

    def complete(self, navigation, observation):
        if not navigation or not observation or self.started is not None:
            raise Fault('CHECKPOINT_INCOMPLETE', 'leg and observation must both finish before resetting recovery')
        self.used = self.remote = 0.
        self.pending = False
        self.event('RUNNING')


class Checkpoint:
    def __init__(self, stage, target=None, holding=None, context=None, path=None, emit=None):
        self.path, self.emit = path, emit
        if path:
            directory = os.path.dirname(os.path.abspath(str(path)))
            if not os.path.isdir(directory):
                os.makedirs(directory)
        self.data = dict(schema=1, stage=stage, target=target, route=None, leg_index=0,
            navigation_complete=False, observation_complete=False, start_odom=None,
            motion_target_m=None, progress_m=0., holding=holding, arm_outcome='not_started',
            last_frame=None, stopped=None, context=list(context) if context else None,
            status='RUNNING', wait_used_s=0., paused_total_s=0., paused_at=None)

    def save(self, **fields):
        previous = self.data.get('status')
        self.data.update(fields)
        if self.path:
            atomic(self.path, self.data)
        if self.emit and 'status' in fields and self.data['status'] != previous:
            self.emit(dict(self.data))

    def event(self, value):
        self.save(**value)


def hull(points):
    points = sorted(set(tuple(p) for p in points))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for part, data in ((lower, points), (upper, reversed(points))):
        for p in data:
            while len(part) >= 2 and cross(part[-2], part[-1], p) <= 0:
                part.pop()
            part.append(p)
    return lower[:-1] + upper[:-1]


def area(poly):
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(poly, poly[1:]+poly[:1]))) / 2 if len(poly) > 2 else 0.


def clip(poly, a, b, inside=True, margin=0.):
    if not poly:
        return []
    length = math.hypot(b[0]-a[0], b[1]-a[1])
    def side(p):
        return ((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])) / length - margin
    result = []
    for p, q in zip(poly, poly[1:]+poly[:1]):
        u, v = side(p), side(q)
        pin, qin = (u >= -1e-10), (v >= -1e-10)
        if not inside:
            pin, qin = not pin, not qin
        if pin:
            result.append(p)
        if pin != qin and abs(u-v) > 1e-12:
            t = u/(u-v)
            result.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
    return result


def subtract(poly, cover, margin=0.):
    """Partition a convex polygon outside another convex polygon."""
    remaining, pieces = poly, []
    for a, b in zip(cover, cover[1:]+cover[:1]):
        outside = clip(remaining, a, b, False, margin)
        if area(outside) > 1e-9:
            pieces.append(outside)
        remaining = clip(remaining, a, b, True, margin)
    return pieces


def project(pose, polygon):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0]+c*x-s*y, pose[1]+s*x+c*y) for x, y in polygon]


class ObservationLedger:
    """Geometry of observed ground, separate from robot navigation progress.

    Uncovered polygons remain pending. No guessed reverse distance or turn is
    issued; a caller may only supply additional views reached by its safety gates.
    """
    def __init__(self, footprint):
        if footprint and len(footprint[0]) == 2 and finite(footprint[0]):
            footprint = [footprint]
        self.views = [hull(polygon) for polygon in footprint]
        self.footprint = hull([p for polygon in self.views for p in polygon])
        if area(self.footprint) <= 1e-6:
            raise Fault('OBSERVATION_GEOMETRY_MISSING', 'calibrated ground footprint required')
        self.last, self.pending, self.after = None, [], None

    def observe(self, sequence, stamp, pose, error=0.):
        if (not finite([sequence, stamp, error] + list(pose)) or error < 0
                or (self.last and (sequence <= self.last['sequence'] or stamp <= self.last['stamp']))):
            return False
        if self.after is not None and stamp <= self.after:
            return False
        for view in self.views:
            cover = project(pose, view)
            self.pending = [piece for polygon in self.pending for piece in subtract(polygon, cover, error)]
        self.last = dict(sequence=sequence, stamp=stamp, pose=list(pose), error=error)
        return True

    def interrupt(self, stop_pose, after, stop_error=.04):
        if self.last is None:
            raise Fault('OBSERVATION_CHECKPOINT_MISSING', 'no valid pre-outage observation')
        old = self.last
        # The convex sweep bounds straight coast. Rotation is bounded by the
        # circumradius, so intermediate headings cannot hide a swept sliver.
        angle = abs(wrap(stop_pose[2]-old['pose'][2]))
        radius = max(math.hypot(x,y) for x,y in self.footprint)
        padding = stop_error + old['error']
        angular_sweep = radius * min(angle, math.pi)
        swept = []
        for view in self.views:
            inner = view
            for a,b in zip(view, view[1:]+view[:1]):
                inner = clip(inner, a, b, margin=padding)
            if area(inner) > 1e-6:
                endpoints = project(old['pose'], inner) + project(stop_pose, inner)
                # Outward square bounds the intermediate rotated view. Shrinking
                # by the turn angle would silently discard the region to reobserve.
                swept.append(hull([(x+dx, y+dy) for x,y in endpoints
                    for dx,dy in ((-angular_sweep,-angular_sweep),(-angular_sweep,angular_sweep),
                                  (angular_sweep,-angular_sweep),(angular_sweep,angular_sweep))]))
        if not swept:
            raise Fault('OBSERVATION_GAP_UNRESOLVED', 'uncertainty consumes calibrated view')
        for view in self.views:
            swept = [part for polygon in swept for part in subtract(polygon, project(old['pose'], view), old['error'])]
        self.pending += swept
        self.after = after

    def require_complete(self):
        if self.pending:
            raise Fault('OBSERVATION_GAP_UNRESOLVED', 'current calibrated view does not cover the perception gap')


class MotionProgress:
    """Measure a straight motion from one immutable odometry origin."""
    def __init__(self, origin, distance, reverse=True):
        self.origin, self.distance, self.reverse = list(origin), distance, reverse

    def remaining(self, odom):
        a = self.origin
        dx, dy = odom[0]-a[0], odom[1]-a[1]
        sign = -1 if self.reverse else 1
        travelled = sign*(dx*math.cos(a[2])+dy*math.sin(a[2]))
        lateral = abs(-dx*math.sin(a[2])+dy*math.cos(a[2]))
        if lateral > .04 or abs(wrap(odom[2]-a[2])) > math.radians(5) or travelled < -.02:
            raise Fault('MOTION_DEVIATED', 'straight recovery motion deviated')
        if travelled > self.distance + .05:
            raise Fault('MOTION_OVERRUN', 'coast exceeded the existing 5cm allowance')
        return max(0., self.distance-travelled), max(0., travelled)


class EngineGate:
    """Local-file barrier for the existing model worker, never an SSH per tick."""
    def __init__(self, state_file, context, checkpoint_file, footprint, holding,
                 clock=monotonic, wall=time.time, emit=None, frame_bound_verified=False):
        self.state_file, self.context = state_file, tuple(context)
        self.clock, self.wall = clock, wall
        self.frame_bound_verified = frame_bound_verified is True
        self.provenance_waiting = False
        self.checkpoint = Checkpoint('APPROACH', holding=holding, context=context,
                                     path=checkpoint_file, emit=emit)
        self.episode = Episode(clock=clock, emit=self.checkpoint.event)
        self.ledger = ObservationLedger(footprint)
        self.last_good, self.anchor = None, None
        self.last_stamp, self.stable, self.first = None, 0, None
        self.cutoff, self.vision_sequence, self.vision_count = None, 0, 0
        self.paused_total, self.pause_origin = 0., None
        self.resume_generation = 0
        self.history = deque(maxlen=300)
        self._cached, self._read_at = None, float('-inf')

    def read(self):
        if self._cached is None or self.clock()-self._read_at >= .025:
            try:
                with io.open(self.state_file, encoding='utf-8') as stream:
                    self._cached = json.load(stream)
                self._read_at = self.clock()
            except (IOError, ValueError):
                raise Fault('SUPERVISOR_STALE', 'local state unavailable', True)
        state = dict(self._cached)
        context = state.get('context')
        if context is not None and tuple(context) != self.context:
            raise Fault('CONTEXT_CHANGED', 'worker localization context changed')
        age = self.clock()-state.get('received_at', float('-inf'))+state.get('source_age_s', float('inf'))
        if state.get('fatal'):
            raise Fault(state.get('reason_code', 'HARDWARE_STATE_UNKNOWN'), 'supervisor requires intervention')
        if state.get('ready') is not True or not 0 <= age <= .4:
            code = state.get('reason_code', 'SUPERVISOR_STALE') if state.get('ready') is not True else 'SUPERVISOR_STALE'
            raise Fault(code, 'waiting for fresh localization', code in TRANSIENT,
                        state.get('wait_elapsed_s', 0.))
        values = state.get('pose', [])+state.get('odom', [])
        if len(values) != 6 or not finite(values+[state.get('stamp')]):
            raise Fault('CHECKPOINT_INVALID', 'worker pose/odometry missing')
        state['context'] = list(self.context)
        return state

    def poll(self, sequence, frame_time, measurement, sensor_problem=None, observation_valid=True):
        reset = False
        try:
            state = self.read()
            for old in state.pop('pose_history', []):
                if (not self.history or old['stamp'] > self.history[-1][1]['stamp']):
                    if len(old['pose']) == len(old['odom']) == 3 and finite(old['pose']+old['odom']+[old['at'], old['stamp']]):
                        self.history.append((old['at'], dict(old, context=list(self.context))))
            if not self.history or state['stamp'] != self.history[-1][1]['stamp']:
                self.history.append((state['received_at']-state['source_age_s'], dict(state)))
            if sensor_problem:
                raise Fault(sensor_problem, 'approach sensors stale', True)
            if self.episode.started is None:
                if self.episode.pending:
                    self.episode.check()  # retained budget, no per-loop reset
                self.last_good = state
                if self.checkpoint.data['start_odom'] is None:
                    self.checkpoint.data['start_odom'] = state['odom']
                self.checkpoint.data['confirmed_odom'] = state['odom']
                if isinstance(measurement, dict):
                    self.checkpoint.data['target'] = {key: measurement[key] for key in ('x','y') if key in measurement}
                elif measurement is not None:
                    self.checkpoint.data['target'] = measurement
                if observation_valid and frame_time and 0 <= self.wall()-frame_time <= 1.5:
                    capture = self.clock()-(self.wall()-frame_time+.30)
                    closest = min(self.history, key=lambda item: abs(item[0]-capture))
                    uncertainty = abs(closest[0]-capture)
                    if uncertainty <= .10:
                        radius = max(math.hypot(x,y) for x,y in self.ledger.footprint)
                        error = .04 + uncertainty*(.35+1.5*radius)
                        if self.ledger.observe(sequence, frame_time-.30, closest[1]['pose'], error):
                            self.checkpoint.save(last_frame=self.ledger.last,
                                last_valid_state={key:state[key] for key in ('pose','odom','stamp','context')})
                return dict(allow=True, reset=False, resumed=False)
            continuity(self.anchor, state)
            self.episode.check()
            if abs(state.get('measured_vx', 1.)) >= .01 or abs(state.get('measured_wz', 1.)) >= .02:
                self.stable, self.first = 0, None
                return dict(allow=False, reset=False, resumed=False)
            if self.last_stamp is not None and state['stamp'] < self.last_stamp:
                raise Fault('ODOM_TIME_REVERSED', 'odometry clock reversed')
            if self.anchor is not None and state['stamp'] <= self.anchor['stamp']:
                return dict(allow=False, reset=False, resumed=False)
            if self.first is not None and (math.hypot(state['pose'][0]-self.first[0], state['pose'][1]-self.first[1]) > .03
                    or abs(wrap(state['pose'][2]-self.first[2])) > math.radians(3)):
                self.stable, self.first = 0, None
            if state['stamp'] != self.last_stamp:
                self.stable += 1
                self.first = self.first or state['pose']
                self.last_stamp = state['stamp']
            if self.stable < 5:
                return dict(allow=False, reset=False, resumed=False)
            if not self.frame_bound_verified:
                self.provenance_waiting = True
                self.checkpoint.save(status='WAIT_OBSERVATION_PROVENANCE', stopped=state,
                    observation_complete=False, reason='0.30s source-to-decoder bound is not measured')
                return dict(allow=False, reset=False, resumed=False)
            if self.cutoff is None:
                self.cutoff = self.wall()
                if self.anchor is not None and self.ledger.last is None:
                    raise Fault('OBSERVATION_CHECKPOINT_MISSING', 'no frame/pose pair before motion outage')
                if self.ledger.last is not None:
                    self.ledger.interrupt(state['pose'], self.cutoff)
                else:
                    self.ledger.after = self.cutoff
                self.vision_sequence = sequence
                self.checkpoint.save(status='WAIT_FRESH_OBSERVATION', stopped=state)
            # A sequence number alone is not fresh evidence: require capture
            # AFTER stopped acknowledgement, including transport latency.
            if (not observation_valid or sequence <= self.vision_sequence or frame_time-.30 <= self.cutoff
                    or not 0 <= self.wall()-frame_time <= 1.5):
                return dict(allow=False, reset=False, resumed=False)
            self.vision_sequence = sequence
            self.vision_count += 1
            self.ledger.observe(sequence, frame_time-.30, state['pose'], .04)
            if self.vision_count < 3:
                return dict(allow=False, reset=False, resumed=False)
            self.ledger.require_complete()
            self.episode.resume()
            self.paused_total += self.clock()-self.pause_origin
            self.pause_origin = None
            self.last_good = state
            self.resume_generation += 1
            self.checkpoint.save(status='REVALIDATE_PATH', paused_total_s=self.paused_total,
                paused_at=None, last_frame=self.ledger.last, observation_complete=True,
                wait_used_s=self.episode.elapsed(),
                resume_generation=self.resume_generation)
            return dict(allow=measurement is not None, reset=True, resumed=True,
                        target_lost=measurement is None)
        except Fault as exc:
            if not exc.retryable:
                if exc.code == 'RECOVERY_TIMEOUT' and self.provenance_waiting:
                    self.checkpoint.save(status='OBSERVATION_PROVENANCE_REQUIRED')
                    raise Fault('OBSERVATION_PROVENANCE_REQUIRED', 'source capture freshness is not validated')
                self.checkpoint.save(status=exc.code)
                raise
            reset = self.episode.started is None
            if reset:
                self.anchor = self.last_good
                self.pause_origin = self.clock()
                self.cutoff, self.stable, self.first, self.vision_count, self.last_stamp = None, 0, None, 0, None
            self.episode.pause(exc.code, exc.elapsed)
            self.checkpoint.save(status='WAIT_LOCALIZATION', paused_at=self.pause_origin,
                paused_total_s=self.paused_total, last_frame=self.ledger.last,
                stopped=self.anchor, observation_complete=False)
            self.stable, self.first, self.last_stamp = 0, None, None
            return dict(allow=False, reset=reset, resumed=False)


def effective_pause(path, now=None):
    """Parent lease timeout excludes verified pause, but never over 30 seconds."""
    if not path:
        return 0.
    try:
        with io.open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        duration = data.get('paused_total_s', 0.)
        if data.get('paused_at') is not None:
            duration += (monotonic() if now is None else now)-data['paused_at']
        return min(30., max(0., duration)) if finite([duration]) else 0.
    except (IOError, ValueError, TypeError):
        return 0.
