#!/usr/bin/env python3
"""Read-only default G2 boot audit; optional bounded ROS-only repair.

No serial owner, arm command or nonzero velocity. health polls do not read
servos. Output is bounded current-state JSON, not an unbounded event buffer.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time

UNITS = ['grasp-service', 'grasp-vision', 'x3plus-g2-roscore', 'x3plus-g2-lidar',
         'x3plus-g2-rosbridge']


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def control_faults(status, owner_ok, available):
    ch, od = status.get('chassis') or {}, status.get('odom') or {}
    reasons = []
    if not status.get('ok'):
        reasons.append('service_health_unavailable')
    if not owner_ok:
        reasons.append('serial_owner_mismatch')
    age = ch.get('board_rx_age_s')
    if not finite(age) or not 0 <= age <= 0.2:
        reasons.append('board_feedback_stale')
    if ch.get('motors') != [0, 0, 0, 0] or ch.get('moving') is not False:
        reasons.append('chassis_not_idle')
    if ch.get('arm_busy') is not False or ch.get('arm_pose') != 'travel':
        reasons.append('arm_not_idle_travel')
    if ch.get('client') is not None:
        reasons.append('navigation_client_present')
    if not status.get('chassis_tcp_ready'):
        reasons.append('startup_interlock')
    twist = od.get('twist')
    if not od.get('valid') or not isinstance(twist, list) or len(twist) != 3 or \
            not all(finite(v) and abs(v) <= 0.01 for v in twist):
        reasons.append('odom_not_valid_stationary')
    if not finite(available) or available < 768:
        reasons.append('RAM_pressure')
    return reasons


def message_age(msg, now):
    stamp = msg['header']['stamp']
    age = now-float(stamp['secs'])-float(stamp['nsecs'])*1e-9
    if not math.isfinite(age):
        raise ValueError('invalid stamp')
    return age


def vision_faults(path, now, pid):
    try:
        sample = json.loads(Path(path).read_text())
        age = now-sample['monotonic']
        if sample['pid'] != pid or not finite(age) or not 0 <= age <= 3 or \
                sample['width'] <= 0 or sample['height'] <= 0 or \
                not finite(sample['inference_s']) or sample['inference_s'] < 0:
            raise ValueError('invalid heartbeat')
        return []
    except (OSError, KeyError, TypeError, ValueError):
        return ['vision_frame_missing_or_stale']


def sensor_faults(samples, now, monotonic):
    reasons = []
    for topic, limit in [('/scan',0.35),('/odom_setmotor',0.20),('odom_tf',0.20)]:
        try:
            msg, received = samples[topic]
            if not (0 <= monotonic-received <= limit and
                    0 <= message_age(msg,now) <= 0.4):
                raise ValueError('stale')
            if topic == '/scan':
                if msg['header']['frame_id'] != 'laser' or not msg['ranges'] or \
                        not any(finite(v) and msg['range_min'] <= v <= msg['range_max'] for v in msg['ranges']):
                    raise ValueError('invalid scan')
            elif msg['header']['frame_id'] != 'odom' or msg['child_frame_id'] != 'base_footprint':
                raise ValueError('wrong frame')
        except (KeyError, TypeError, ValueError):
            reasons.append(topic+'_missing_or_stale')
    return reasons


def diagnostic(command='health'):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(2)
        connection.connect('/tmp/grasp_health.sock' if command == 'health'
                           else '/tmp/grasp_service.sock')
        connection.sendall((command+'\n').encode())
        with connection.makefile('rb') as stream:
            return json.loads(stream.readline(65536))


def shell(args):
    return subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                          universal_newlines=True,timeout=8)


def available_mb():
    fields = dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
    return int(fields['MemAvailable'].split()[0])/1024


def owner_matches():
    result = shell(['sudo','-n','fuser','/dev/myserial'])
    pids = result.stdout.split()
    main = shell(['systemctl','show','grasp-service','-p','MainPID','--value'])
    return result.returncode == 0 and len(pids) == 1 and pids[0] == main.stdout.strip()


class Sensors:
    def __init__(self):
        import roslibpy
        self.lock = threading.Lock()
        self.latest, self.counts = {}, {}
        self.ros = roslibpy.Ros(host='127.0.0.1',port=9090)
        self.topics = []
        self.ros.factory.maxDelay = 2
        try:
            self.ros.run(timeout=5)
        except Exception as exc:
            # Keep the reconnecting factory alive; an unavailable bridge must
            # produce a failed audit, not prevent diagnostics from starting.
            print(json.dumps({'ros_connection_pending':str(exc)}),flush=True)
        for name, kind in [('/scan','sensor_msgs/LaserScan'),
                           ('/odom_setmotor','nav_msgs/Odometry'),('/tf','tf2_msgs/TFMessage')]:
            topic = roslibpy.Topic(self.ros,name,kind,queue_length=1)
            self.topics.append(topic)
            topic.subscribe(lambda msg,n=name:self.accept(n,msg))

    def accept(self,name,msg):
        with self.lock:
            self.counts[name] = self.counts.get(name,0)+1
            if name == '/tf':
                for transform in msg.get('transforms',[]):
                    if transform.get('header',{}).get('frame_id') == 'odom' and \
                            transform.get('child_frame_id') == 'base_footprint':
                        self.latest['odom_tf'] = (transform,time.monotonic())
            else:
                self.latest[name] = (msg,time.monotonic())

    def snapshot(self):
        with self.lock:
            return dict(self.latest),dict(self.counts)

    def close(self):
        for topic in self.topics:
            topic.unsubscribe()
        self.ros.terminate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=30)
    parser.add_argument('--repair-ros',action='store_true')
    parser.add_argument('--boot-gate',action='store_true')
    parser.add_argument('--vision-health-file',default='')
    parser.add_argument('--output',default='/tmp/g2_safety_audit.json')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300:
        parser.error('duration must be 10..300 s')
    sensors = None
    repairs = 0
    deadline = time.monotonic()+args.seconds
    final = {}
    boot_locked = False
    boot_released = False
    healthy_streak = 0
    try:
        sensors = Sensors()
        time.sleep(2)
        while time.monotonic() < deadline:
            try:
                status = diagnostic()
            except (OSError,ValueError):
                status = {}
            available = available_mb()
            controls = control_faults(status,owner_matches(),available)
            if args.boot_gate and not boot_locked and not controls:
                boot_locked = bool(diagnostic('maintenance-lock').get('ok'))
            samples,counts = sensors.snapshot()
            sensor_reasons = sensor_faults(samples,time.time(),time.monotonic())
            services = {unit:shell(['systemctl','is-active',unit]).stdout.strip() for unit in UNITS}
            unit_reasons = [unit+'_inactive' for unit,state in services.items() if state!='active']
            if args.vision_health_file:
                vision_pid = shell(['systemctl','show','grasp-vision','-p','MainPID','--value']).stdout.strip()
                try:
                    vision_pid = int(vision_pid)
                except ValueError:
                    vision_pid = 0
                unit_reasons += vision_faults(args.vision_health_file,time.monotonic(),vision_pid)
            all_healthy = not (controls or sensor_reasons or unit_reasons)
            healthy_streak = healthy_streak+1 if all_healthy else 0
            if args.boot_gate and boot_locked and healthy_streak >= 2:
                if diagnostic('maintenance-unlock').get('ok'):
                    status = diagnostic()
                    boot_locked = False
                    boot_released = not control_faults(status,owner_matches(),available_mb())
            final = dict(ready=not (controls or sensor_reasons or unit_reasons),
                         control_faults=controls,sensor_faults=sensor_reasons,
                         unit_faults=unit_reasons,
                         services=services,memory_available_mb=round(available,1),
                         health=status,counts=counts,repairs=repairs,time=time.time(),
                         note='readiness snapshot; no movement authorization')
            if status.get('chassis',{}).get('maintenance'):
                final['ready'] = False
            if args.boot_gate and not boot_released:
                final['ready'] = False
            final['boot_gate'] = args.boot_gate
            # Atomic single snapshot; boot mode releases only its verified gate.
            target = Path(args.output)
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(final,indent=2)+'\n')
            temporary.replace(target)
            print(json.dumps({k:final[k] for k in ['ready','control_faults','sensor_faults','repairs','memory_available_mb']}),flush=True)
            if args.boot_gate and final['ready'] and healthy_streak >= 2:
                break
            if args.repair_ros and not controls and available >= 1024 and repairs < 2 and \
                    services['grasp-service']=='active' and services['x3plus-g2-roscore']=='active':
                unit = None
                if not status.get('odom',{}).get('rosbridge') or \
                        services['x3plus-g2-rosbridge']!='active' or '/odom_setmotor_missing_or_stale' in sensor_reasons:
                    unit = 'x3plus-g2-rosbridge'
                elif '/scan_missing_or_stale' in sensor_reasons:
                    unit = 'x3plus-g2-lidar'
                if unit:
                    # Fresh cached idle gate immediately before the restart.
                    if not control_faults(diagnostic(),owner_matches(),available_mb()) and \
                            diagnostic('maintenance-lock').get('ok'):
                        result = shell(['sudo','-n','systemctl','restart',unit])
                        repairs += 1
                        print(json.dumps({'repair':unit,'returncode':result.returncode}),flush=True)
                        for _ in range(16):
                            time.sleep(0.5)
                            recovered,_ = sensors.snapshot()
                            if not sensor_faults(recovered,time.time(),time.monotonic()) and \
                                    not control_faults(diagnostic(),owner_matches(),available_mb()):
                                if not args.boot_gate:
                                    diagnostic('maintenance-unlock')
                                break
            time.sleep(2)
    finally:
        if sensors:
            sensors.close()
    raise SystemExit(0 if final.get('ready') else 2)


if __name__ == '__main__':
    main()
