"""Session-local self-return filtering and bounded supervised yaw probe."""
import argparse
import dataclasses
import json
import math
import socket
import time
from pathlib import Path

try:
    from . import g2_turn_probe as turn
    from .lidar_self_return_filter import SelfReturnFilter
except ImportError:
    import g2_turn_probe as turn
    from lidar_self_return_filter import SelfReturnFilter


def rpc(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(8)
        sock.connect('/tmp/grasp_service.sock')
        sock.sendall((command + '\n').encode())
        result = json.loads(sock.makefile().readline())
    if not result.get('ok'):
        raise RuntimeError('RPC failed: ' + str(result))
    return result


def available_mb():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 1024
    raise RuntimeError('MemAvailable unavailable')


def side_points(scan, cfg):
    points = []
    for angle, distance in scan.points:
        value = turn.nr._transform_scan_sample(angle, distance, cfg)
        if value is None:
            continue
        a, r = value
        x, y = r * math.cos(a), r * math.sin(a)
        if -.35 < x < .35 and .12 < abs(y) < .65:
            points.append((angle, distance))
    return dataclasses.replace(scan, points=tuple(points))


class FilteredSensors:
    def __init__(self, sensors, profile):
        self.sensors, self.profile = sensors, profile

    def snapshot(self):
        scan, odom, info, error = self.sensors.snapshot()
        if available_mb() < 768:
            error = 'low_memory'
        if scan is not None:
            scan = dataclasses.replace(scan, points=self.profile.apply(scan.points))
        return scan, odom, info, error


class SideConfirmation:
    """Near returns stop immediately; farther ones need 3 distinct frames."""
    def __init__(self):
        self.last_stamp = None
        self.last_side = ''
        self.last_distance = math.inf
        self.count = 0

    def update(self, stamp, side, distance):
        if side and distance <= .26:
            return side
        if stamp != self.last_stamp:
            same = side == self.last_side and abs(distance - self.last_distance) <= .05
            self.count = self.count + 1 if side and same else (1 if side else 0)
            self.last_stamp, self.last_side, self.last_distance = stamp, side, distance
        return side if self.count >= 3 else ''


def bounded_turn(sensors, cfg, direction):
    scan, start, _, error = sensors.snapshot()
    why, _ = turn.health(scan, start, cfg)
    if error or why or abs(start.vx) > .01 or abs(start.wz) > .02:
        raise RuntimeError('turn start gate: ' + str(error or why or 'motion'))
    begun = time.monotonic()
    confirmation = SideConfirmation()
    client = turn.TurnClient(direction)
    reason, commands, trace = 'unknown', 0, []
    try:
        while True:
            scan, odom, _, error = sensors.snapshot()
            why, _ = turn.health(scan, odom, cfg)
            if error or why:
                reason = error or why
                break
            side, distance = turn.side_hazard(scan.points, cfg)
            confirmed = confirmation.update(scan.stamp, side, distance)
            angle = direction * math.degrees(turn.yaw_delta(start.yaw, odom.yaw))
            drift = math.hypot(odom.x - start.x, odom.y - start.y)
            trace.append({'t': time.monotonic()-begun, 'yaw_deg': angle,
                          'odom_stamp': odom.stamp, 'side': side, 'confirmed': confirmed})
            if confirmed:
                reason = confirmed
                break
            if angle < -1.5 or angle > turn.HARD_YAW_DEG or drift > turn.MAX_TRANSLATION_M:
                reason = 'yaw_or_translation_limit'
                break
            if angle >= turn.STOP_AT_DEG:
                reason = 'angle_target'
                break
            if time.monotonic()-begun >= turn.MAX_TURN_S:
                reason = 'time_limit'
                break
            if time.monotonic()-begun >= .7 and angle < .5:
                reason = 'no_odom_progress'
                break
            client.velocity()
            commands += 1
            time.sleep(turn.CONTROL_PERIOD_S)
    finally:
        client.close()
    # Require fresh, stopped feedback rather than treating one delayed sample
    # as the final physical pose.
    end, stopped, stamp = time.monotonic()+2, 0, None
    while time.monotonic() < end:
        _, final, _, error = sensors.snapshot()
        if error or time.monotonic()-final.received_at > .2:
            raise RuntimeError('final odom stale/error')
        if final.stamp != stamp:
            stamp = final.stamp
            stopped = stopped+1 if abs(final.vx)<.01 and abs(final.wz)<.02 else 0
        if stopped >= 3 and time.monotonic()-end > -1.4:
            break
        time.sleep(.05)
    if stopped < 3:
        raise RuntimeError('did not confirm stopped odom')
    final_angle = direction*math.degrees(turn.yaw_delta(start.yaw, final.yaw))
    drift = math.hypot(final.x-start.x, final.y-start.y)
    return {'reason': reason, 'directional_yaw_deg': final_angle,
            'drift_m': drift, 'velocity_commands': commands, 'trace': trace,
            'within_limits': -1.5 <= final_angle <= turn.HARD_YAW_DEG and drift <= .03}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--operator-confirmed-clear', action='store_true', required=True)
    parser.add_argument('--direction', choices=['left', 'right'], default='left')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = {'mode': 'bounded_filtered_turn' if args.real else 'read_only',
              'direction': args.direction, 'lidar_forward_m': .13,
              'scope': 'session-local side returns; raw /scan unchanged',
              'operator_confirmed_clear': True, 'motion_attempted': False}
    sensors = None
    try:
        report['initial_status'] = turn.check_service_status()
        turn.check_one_serial_owner()
        if available_mb() < 1024:
            raise RuntimeError('less than 1 GB available before test')
        cfg = turn.nr.NavRLConfig(lidar_yaw_offset_deg=180.,
                                 lidar_forward_offset_m=.13, safety_brake_dist=.42)
        sensors = turn.RosNavSensors()
        sensors.wait_ready()
        frames, stamps = [], set()
        end = time.monotonic() + 5
        while len(frames) < 10:
            if time.monotonic() > end:
                raise RuntimeError('capture timeout')
            scan, odom, _, error = sensors.snapshot()
            why, _ = turn.health(scan, odom, cfg)
            if error or why or abs(odom.vx) > .01 or abs(odom.wz) > .02:
                raise RuntimeError('stationary capture gate: ' + str(error or why or 'motion'))
            if scan.stamp not in stamps:
                stamps.add(scan.stamp)
                frames.append(side_points(scan, cfg))
            time.sleep(.02)
        profile = SelfReturnFilter.from_clear_scans(frames)
        report['profile'] = profile.ranges_by_angle
        report['capture_side_points'] = [list(frame.points) for frame in frames]
        report['capture_stamps'] = sorted(stamps)
        filtered = FilteredSensors(sensors, profile)
        # Profile is frozen. Validate on subsequent distinct frames before unlocking.
        checks, last = [], frames[-1].stamp
        clear_count = 0
        end = time.monotonic() + 4
        while clear_count < 5:
            if time.monotonic() > end:
                raise RuntimeError('validation timeout')
            scan, odom, _, error = filtered.snapshot()
            if scan.stamp != last:
                last = scan.stamp
                why, front = turn.health(scan, odom, cfg)
                side, distance = turn.side_hazard(scan.points, cfg)
                checks.append({'stamp': last, 'error': error or why,
                               'side_gate': side, 'side_distance_m': distance if side else None,
                               'front_m': front if math.isfinite(front) else None})
                clear_count = clear_count + 1 if not (error or why or side) else 0
            time.sleep(.02)
        report['validation'] = checks
        if args.real:
            turn.check_service_status()
            turn.check_one_serial_owner()
            if available_mb() < 1024:
                raise RuntimeError('less than 1 GB available before unlock')
            report['start_odom'] = dataclasses.asdict(filtered.snapshot()[1])
            report['unlock'] = rpc('maintenance-unlock')
            report['motion_attempted'] = True
            report['result'] = bounded_turn(filtered, cfg, 1 if args.direction == 'left' else -1)
            report['final_odom'] = dataclasses.asdict(filtered.snapshot()[1])
            if report['result']['reason'] != 'angle_target' or not report['result']['within_limits']:
                raise RuntimeError('turn did not reach target within limits')
        report['ok'] = True
    except Exception as exc:
        report['ok'] = False
        report['error'] = str(exc)
    finally:
        if args.real:
            try:
                report['lock'] = rpc('maintenance-lock')
                report['final_status'] = rpc('status')
            except Exception as exc:
                report['ok'] = False
                report['cleanup_error'] = str(exc)
        if sensors is not None:
            sensors.close()
        Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in report.items()
                      if k in ('ok', 'error', 'result', 'motion_attempted', 'validation')}))
    if not report['ok']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
