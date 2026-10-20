#!/usr/bin/env python3
"""Bounded mapping launcher, run after sourcing ROS Melodic environment.

No chassis commands. Stops its own children on low RAM, timeout or exit.
Does not launch ROS master/driver/AMCL or restart the resident arm service.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def memory_available_mb():
    # ROS Melodic's system Python must not import the Python 3.8 nav/model stack.
    fields = dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
    return int(fields['MemAvailable'].split()[0])/1024.0


def stop(children):
    for child in children:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
    deadline = time.monotonic()+5
    while any(c.poll() is None for c in children) and time.monotonic() < deadline:
        time.sleep(0.1)
    for child in children:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
        child.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=300)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600:
        parser.error('duration must be 10..600 s')
    children = []
    try:
        if memory_available_mb() < 1024:
            raise RuntimeError('mapping requires >=1024 MB available before launch')
        # Refuse competing map TF publishers rather than restarting them.
        nodes = subprocess.check_output(['rosnode','list'],timeout=5).decode().splitlines()
        if any('amcl' in n or 'gmapping' in n or 'map_server' in n or
               'g2_mapping_filter' in n for n in nodes):
            raise RuntimeError('existing localization/mapping node; stop it explicitly first')
        # RLIMIT_AS limits virtual allocations; reserve monitor handles global RAM.
        # No preexec_fn: this process may later acquire ROS worker threads.
        children.append(subprocess.Popen([
            'prlimit','--as=268435456','--', '/usr/bin/python3',
            str(Path(__file__).with_name('g2_mapping_filter.py'))],start_new_session=True))
        children.append(subprocess.Popen([
            'prlimit','--as=838860800','--','rosrun','gmapping','slam_gmapping',
            '__name:=g2_demo_gmapping','scan:=/scan_mapping',
            '_base_frame:=base_footprint','_odom_frame:=odom',
            '_maxRange:=3.0','_maxUrange:=3.0','_particles:=10',
            '_delta:=0.02','_xmin:=-2.0','_ymin:=-2.0',
            '_xmax:=2.0','_ymax:=2.0','_map_update_interval:=1.0',
            '_linearUpdate:=0.05','_angularUpdate:=0.05',
            '_temporalUpdate:=-1.0'],start_new_session=True))
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            if memory_available_mb() < 768:
                raise RuntimeError('RAM_pressure; mapping stopped')
            if any(c.poll() is not None for c in children):
                raise RuntimeError('mapping child exited; no automatic restart')
            time.sleep(0.25)
    finally:
        stop(children)


if __name__ == '__main__':
    main()
