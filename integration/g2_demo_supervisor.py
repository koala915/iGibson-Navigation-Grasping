#!/usr/bin/env python3
"""Bounded forward demo supervisor. UDP 7002 intent -> resident TCP 7000.

Sender must refresh {seq,vx,ttl} every <=0.1 s (ttl <=0.3 s).
One run is one <=0.25 m segment; explicit stop/expired intent ends the run.
No serial access, service restarts, old-command replay or reverse recovery.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path

import nav_rl as nr
from g2_nav_client import (RosNavSensors, G2ChassisClient, health,
                           check_service_status, check_one_serial_owner,
                           forward_distance)


def memory_available_mb(path='/proc/meminfo'):
    fields = dict(line.split(':', 1) for line in Path(path).read_text().splitlines())
    return int(fields['MemAvailable'].split()[0]) / 1024.0


class Lease:
    def __init__(self):
        self.seq = -1
        self.deadline = 0.0
        self.vx = 0.0

    def accept(self, msg, now):
        seq, vx, ttl = msg['seq'], float(msg['vx']), float(msg['ttl'])
        if type(seq) is not int or seq <= self.seq:
            raise ValueError('sequence must strictly increase')
        if not (math.isfinite(vx) and 0 <= vx <= 0.15 and
                math.isfinite(ttl) and 0 < ttl <= 0.3):
            raise ValueError('forward vx 0..0.15, ttl 0..0.3 required')
        self.seq, self.vx, self.deadline = seq, vx, now + ttl

    def active(self, now):
        return now < self.deadline and self.vx > 0


class Recovery:
    """Stop immediately on raw hazard; resume only after stable clearance."""
    def __init__(self):
        self.blocked = False
        self.clear_since = None
        self.resumes = 0

    def tick(self, reason, now):
        if reason and reason != 'lidar_brake':
            raise RuntimeError(reason)
        if reason:
            self.blocked, self.clear_since = True, None
            return False
        if not self.blocked:
            return True
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since < 0.5:
            return False
        if self.resumes >= 2:
            raise RuntimeError('recovery_budget_exhausted')
        self.resumes += 1
        self.blocked = False
        return True


def run(args):
    # Memory read failure fails closed. Never poll graspctl during motion.
    if memory_available_mb() < args.min_available_mb:
        raise RuntimeError('insufficient_RAM_before_start')
    check_service_status()
    check_one_serial_owner()
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=180,
                         lidar_forward_offset_m=0.10, safety_brake_dist=0.42)
    sensors = RosNavSensors()
    client = None
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind(('127.0.0.1', 7002))
        receiver.setblocking(False)
        sensors.wait_ready()
        scan, start, info, error = sensors.snapshot()
        verified, why = nr.validate_orientation_evidence(
            args.lidar_orientation_evidence, cfg, nr.describe_scan(info, cfg))
        if not verified:
            raise RuntimeError('orientation: ' + why)
        if error or health(scan, start, cfg)[0]:
            raise RuntimeError('sensor_preflight')
        if abs(start.vx) > 0.01 or abs(start.wz) > 0.01:
            raise RuntimeError('not_stationary')
        client = G2ChassisClient()
        lease, recovery = Lease(), Recovery()
        waiting = time.monotonic()
        begun = None
        last_m, last_progress = 0.0, waiting
        print('ready: refresh UDP localhost:7002 {seq,vx,ttl}; 5 s start timeout', flush=True)
        while True:
            now = time.monotonic()
            # Fixed packet cap keeps intent floods from starving sensor checks.
            for _ in range(8):
                try:
                    packet = receiver.recv(1024)
                except BlockingIOError:
                    break
                lease.accept(json.loads(packet.decode('ascii')), now)
                if lease.vx == 0:
                    return 'explicit_stop'
            if begun is None:
                if lease.active(now):
                    begun = last_progress = now
                elif now - waiting > 5:
                    return 'start_timeout'
                else:
                    time.sleep(0.05)
                    continue
            if not lease.active(now):
                return 'intent_expired'
            if memory_available_mb() < args.min_available_mb:
                raise RuntimeError('RAM_pressure')
            scan, odom, _, error = sensors.snapshot()
            reason, _ = health(scan, odom, cfg)
            if error:
                raise RuntimeError(error)
            # Validate sensor freshness before trusting odometry.
            if reason and reason != 'lidar_brake':
                raise RuntimeError(reason)
            moved = forward_distance(start, odom)
            lateral = -(odom.x-start.x)*math.sin(start.yaw)+(odom.y-start.y)*math.cos(start.yaw)
            yaw = (odom.yaw-start.yaw+math.pi) % (2*math.pi)-math.pi
            if abs(lateral) > 0.05 or abs(yaw) > math.radians(15):
                raise RuntimeError('path_deviation')
            if moved < -0.02 or moved-last_m > 0.05:
                raise RuntimeError('odom_jump')
            if moved > last_m + 0.002:
                last_progress = now
            last_m = moved
            if moved >= args.distance_m-0.06:
                return 'distance_target'
            if now-begun > 2.5 or moved >= 0.28:
                raise RuntimeError('segment_hard_limit')
            allowed = recovery.tick(reason, now)
            if not allowed:
                client.stop()
                last_progress = now
            elif now-last_progress > 0.8:
                # A stalled wheel is not proven to be a dropped board command.
                raise RuntimeError('no_odom_progress')
            else:
                client.velocity(lease.vx)
            time.sleep(0.05)
    finally:
        if client is not None:
            client.close()
        receiver.close()
        sensors.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance-m', type=float, default=0.15)
    parser.add_argument('--min-available-mb', type=float, default=768)
    parser.add_argument('--lidar-orientation-evidence', required=True)
    parser.add_argument('--i-confirm-clear-path', action='store_true')
    parser.add_argument('--i-confirm-power-cut', action='store_true')
    args = parser.parse_args()
    if not (0.10 <= args.distance_m <= 0.25 and
            math.isfinite(args.min_available_mb) and args.min_available_mb >= 768 and
            args.i_confirm_clear_path and args.i_confirm_power_cut):
        parser.error('requires 0.10..0.25 m, RAM reserve >=768 MB and physical confirmations')
    print(run(args))


if __name__ == '__main__':
    main()
