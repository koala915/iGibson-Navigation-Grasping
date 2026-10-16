"""Supervised accumulated-90deg calibration with settled, bounded pulses.

Uses the persistent filtered /scan, not automatic self-profile learning.
Measured body rectangle plus 2cm measurement tolerance is excluded as
operator-confirmed structure/cables (the supplied dimensions are approximate).
This circular swept-envelope gate is for this supervised calibration only.
"""
import argparse
import dataclasses
import json
import math
import time
from pathlib import Path

try:
    from . import g2_turn_probe as turn
    from .g2_filtered_turn_calibration import rpc, available_mb
except ImportError:
    import g2_turn_probe as turn
    from g2_filtered_turn_calibration import rpc, available_mb


def swept_hazard(points):
    candidates = []
    for angle, distance in points:
        theta = math.radians(angle + 180)
        x, y = .13 + distance*math.cos(theta), distance*math.sin(theta)
        if -.15 <= x <= .22 and abs(y) <= .14:
            continue
        if math.hypot(x, y) <= math.hypot(.20, .12)+.02:
            candidates.append((angle, x, y))
    candidates.sort()
    cluster = []
    for point in candidates:
        if cluster and (point[0]-cluster[-1][0] > 1 or
                        math.hypot(point[1]-cluster[-1][1],point[2]-cluster[-1][2]) > .04):
            if len(cluster) >= 3:
                return True
            cluster = []
        cluster.append(point)
    return len(cluster) >= 3


class EnvelopeGate:
    def __init__(self):
        self.last = None
        self.count = 0

    def check(self, scan):
        # Three independent scans suppress brief residual speckles. At the
        # 10Hz scanner this adds ~0.2s; bounded pulses stop independently.
        if scan.stamp != self.last:
            self.last = scan.stamp
            self.count = self.count+1 if swept_hazard(scan.points) else 0
        return self.count >= 3


def pulse_duration(remaining):
    if remaining > 20:
        return .20
    if remaining > 7:
        return .10
    return .08 if remaining > 3 else .06


def directed_yaw(origin, current, direction):
    return direction*math.degrees(turn.yaw_delta(origin,current))


def target_limits(target):
    if target not in (5,10,90):
        raise ValueError('supported calibration targets: 5, 10, 90 degrees')
    return (89.2,91.5,95) if target==90 else (target-.4,target+.8,target+3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--resume-report', help='verify and continue from last settled pulse; retain original origin')
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--direction', choices=('left','right'), default='left')
    parser.add_argument('--target-deg', type=int, choices=(5,10,90), default=90)
    parser.add_argument('--operator-confirmed-self-returns', action='store_true',
                        help='supervised clear-floor calibration only; record swept returns without gating')
    args = parser.parse_args()
    direction=1 if args.direction=='left' else -1
    stop_min,success_max,hard_max=target_limits(args.target_deg)
    reference = json.loads(Path(args.reference).read_text())
    origin = reference['start_odom']
    expected = reference['final_odom']
    if args.resume_report:
        previous=json.loads(Path(args.resume_report).read_text())
        if previous.get('reference_odom') != origin or not previous.get('pulses'):
            raise ValueError('resume reference mismatch or no settled pulse')
        if previous.get('direction','left') != args.direction:
            raise ValueError('resume direction mismatch')
        if previous.get('target_accumulated_deg') != args.target_deg:
            raise ValueError('resume target mismatch')
        expected=previous['pulses'][-1]['odom']
    report = {'target_accumulated_deg':args.target_deg, 'pulses':[], 'ok':False,
              'reference_odom':origin, 'real':args.real,
              'direction':args.direction,
              'operator_confirmed_self_returns':args.operator_confirmed_self_returns,
              'swept_gate_observations':0}
    sensors = None
    try:
        report['initial_status'] = turn.check_service_status()
        turn.check_one_serial_owner()
        if available_mb() < 1024:
            raise RuntimeError('RAM below 1GB before new test')
        sensors = turn.RosNavSensors()
        sensors.wait_ready()
        cfg = turn.nr.NavRLConfig(lidar_yaw_offset_deg=180,
                                 lidar_forward_offset_m=.13,safety_brake_dist=.42)
        scan,start,_,error=sensors.snapshot()
        if abs(turn.yaw_delta(expected['yaw'],start.yaw)) > math.radians(1) or math.hypot(start.x-expected['x'],start.y-expected['y']) > .02:
            raise RuntimeError('robot moved or odom reset since measured reference')
        report['start_odom']=dataclasses.asdict(start)
        def checked():
            scan,odom,_,error=sensors.snapshot()
            why,_=turn.health(scan,odom,cfg)
            if error or why:
                raise RuntimeError('sensor gate: '+(error or why))
            if available_mb() < 768:
                raise RuntimeError('RAM below 768MB')
            angle=directed_yaw(origin['yaw'],odom.yaw,direction)
            drift=math.hypot(odom.x-start.x,odom.y-start.y)
            if not -1.5 <= angle <= hard_max or drift > (.05 if args.target_deg==90 else .03):
                raise RuntimeError('accumulated yaw or translation limit')
            return scan,odom,angle
        gate=EnvelopeGate()
        def envelope_blocked(scan):
            found=gate.check(scan)
            if found:
                report['swept_gate_observations']+=1
            return found and not args.operator_confirmed_self_returns
        for _ in range(8):
            scan,odom,angle=checked()
            if abs(odom.vx)>.01 or abs(odom.wz)>.02 or envelope_blocked(scan):
                raise RuntimeError('stationary/swept-envelope preflight')
            time.sleep(.1)
        report['initial_accumulated_deg']=angle
        if args.real:
            rpc('maintenance-unlock')
            begun=time.monotonic()
            stalled=0
            for _ in range(30):
                scan,odom,angle=checked()
                if angle >= stop_min:
                    break
                if time.monotonic()-begun > 60:
                    raise RuntimeError('total timeout')
                if envelope_blocked(scan):
                    raise RuntimeError('swept-envelope gate')
                before=angle
                duration=pulse_duration(args.target_deg-angle)
                pulse_start=time.monotonic()
                client=turn.TurnClient(direction)
                commands=0
                try:
                    while time.monotonic()-pulse_start < duration:
                        scan,odom,angle=checked()
                        if envelope_blocked(scan):
                            raise RuntimeError('swept-envelope gate')
                        client.velocity()
                        commands+=1
                        time.sleep(min(.04,max(0,duration-(time.monotonic()-pulse_start))))
                finally:
                    client.close()
                end=time.monotonic()+2
                stable,last=0,None
                while True:
                    scan,odom,angle=checked()
                    if odom.stamp!=last:
                        last=odom.stamp
                        stable=stable+1 if abs(odom.vx)<.01 and abs(odom.wz)<.02 else 0
                    if stable>=3 and time.monotonic()-pulse_start>=.8:
                        break
                    if time.monotonic()>end:
                        raise RuntimeError('stopped feedback timeout')
                    time.sleep(.04)
                report['pulses'].append({'duration_s':duration,'commands':commands,
                                         'before_deg':before,'after_deg':angle,
                                         'odom':dataclasses.asdict(odom)})
                print('settled_accumulated_deg=%.3f' % angle,flush=True)
                if not -.5 <= angle-before <= 15:
                    raise RuntimeError('pulse progress/overshoot limit')
                stalled=stalled+1 if angle-before < .05 else 0
                if stalled >= 3:
                    raise RuntimeError('three pulses without progress')
            else:
                raise RuntimeError('pulse count limit')
        scan,odom,angle=checked()
        report['final_odom']=dataclasses.asdict(odom)
        report['final_accumulated_deg']=angle
        report['ok']=not args.real or stop_min<=angle<=success_max
        if not report['ok']:
            report['error']='final angle outside tolerance'
    except Exception as exc:
        report['error']=str(exc)
    finally:
        if args.real:
            try:
                report['lock']=rpc('maintenance-lock')
                report['final_status']=rpc('status')
            except Exception as exc:
                report['cleanup_error']=str(exc)
                report['ok']=False
        if sensors is not None:
            sensors.close()
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k in ('ok','error','final_accumulated_deg','initial_accumulated_deg')}))
    if not report['ok']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
