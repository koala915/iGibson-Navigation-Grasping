"""Jetson end of the patrol's vision link; mission_loop.py starts it over ssh.

ncu's g2_map_patrol.py --vision-permit PATH reads a permit before every output
and drives only while it says allow, with a source stamp under 1.2 s old on this
clock (PR #5, 2026-10-19). The PC sees the rear camera, so it decides; this end
stamps and writes.

stdin, one JSON line per rear-camera frame the PC processed:
    {"allow": true|false, "age": seconds from that frame's capture to now}
The source stamp is this clock's now minus `age`, so it is never newer than the
frame. Bad input, EOF or a lost PC write allow false: the patrol stops at once.
    {"watch": PID}   the patrol process the PC just started (writes no permit)

stdout, every 0.1 s: the localization supervisor's pose, so the PC can place a
sighting on the map without an ssh round trip per frame, and whether the watched
patrol still runs ("patrol": {"pid", "alive"}). 2026-10-20: one ssh `kill -0` a
second from the PC held the permit renewals back by ~0.5 s each time.

Python 3.6 (the Jetson's system python3), standard library only.
"""
import json
import os
import socket
import sys
import threading
import time

AGE_MAX_S = 5.0


def owner_pid(path):
    """The resident grasp service's PID, which the patrol binds the permit to."""
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(2)
    try:
        s.connect(path)
        s.sendall(b'health\n')
        return json.loads(s.makefile('r').readline())['service_pid']
    finally:
        s.close()


def write_permit(path, owner, boot, allow, stamp):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'owner_pid': owner, 'boot_id': boot, 'source_stamp': stamp,
                   'allow': allow is True}, f)
    os.rename(tmp, path)


def permit_from_line(line, now):
    """(allow, source_stamp) for one PC line; anything malformed denies."""
    try:
        message = json.loads(line)
        age = float(message['age'])
        if not 0.0 <= age <= AGE_MAX_S:
            raise ValueError('age out of range')
        return message['allow'] is True, now - age
    except (ValueError, KeyError, TypeError):
        return False, now


def pose_line(health_path, now):
    try:
        with open(health_path) as f:
            health = json.load(f)
        return {'t': now, 'ready': health.get('ready') is True, 'stamp': health.get('stamp'),
                'pose': (health.get('state') or {}).get('pose')}
    except (IOError, ValueError) as exc:
        return {'t': now, 'ready': False, 'error': str(exc)}


def watched_pid(line):
    """The PID of a {"watch": PID} line, else None (a permit line)."""
    try:
        message = json.loads(line)
        pid = message['watch'] if isinstance(message, dict) else None
        return pid if isinstance(pid, int) and not isinstance(pid, bool) and pid > 0 else None
    except (ValueError, KeyError, TypeError):
        return None


def process_alive(pid, proc='/proc'):
    """Running, and not a zombie waiting to be reaped."""
    try:
        with open('{}/{}/stat'.format(proc, pid)) as f:
            state = f.read().rsplit(')', 1)[1].split()[0]
    except (IOError, OSError, IndexError):
        return False
    return state not in ('Z', 'X')


def main(argv):
    permit_path, health_path, socket_path = argv[1:4]
    with open('/proc/sys/kernel/random/boot_id') as f:
        boot = f.read().strip()
    owner = owner_pid(socket_path)
    write_permit(permit_path, owner, boot, False, time.time())
    watching = [None]

    def report_poses():
        while True:
            line = pose_line(health_path, time.time())
            pid = watching[0]
            if pid is not None:
                line['patrol'] = {'pid': pid, 'alive': process_alive(pid)}
            sys.stdout.write(json.dumps(line) + '\n')
            sys.stdout.flush()
            time.sleep(0.1)

    thread = threading.Thread(target=report_poses)
    thread.daemon = True
    thread.start()
    try:
        for line in sys.stdin:
            pid = watched_pid(line)
            if pid is not None:
                watching[0] = pid
                continue
            allow, stamp = permit_from_line(line, time.time())
            write_permit(permit_path, owner, boot, allow, stamp)
    finally:
        write_permit(permit_path, owner, boot, False, time.time())


if __name__ == '__main__':
    main(sys.argv)
