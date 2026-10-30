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

After an outage ncu's patrol (70ca5eb, from b5a618f) pauses and resumes only once
the permit's recovery_ack equals the pause count it publishes in its report
(task_checkpoint.recovery_generation, requires_observation). With the report path
as the 4th argument this end acks a pause after ACK_FRAMES allowed frames that
were captured after it first saw that pause: the camera has looked again.

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
ACK_FRAMES = 3
ACK_TRACE = '/tmp/mission_permit_ack.json'


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


def write_permit(path, owner, boot, allow, stamp, recovery_ack=None):
    tmp = path + '.tmp'
    value = {'owner_pid': owner, 'boot_id': boot, 'source_stamp': stamp, 'allow': allow is True}
    if recovery_ack is not None:
        value['recovery_ack'] = recovery_ack
    with open(tmp, 'w') as f:
        json.dump(value, f)
    os.rename(tmp, path)


def patrol_pause(report_path):
    """The pause count the patrol waits on, or None when it waits on none."""
    try:
        with open(report_path) as f:
            record = json.load(f).get('task_checkpoint') or {}
    except (IOError, OSError, ValueError, AttributeError):
        return None
    generation = record.get('recovery_generation')
    if record.get('requires_observation') is not True or not isinstance(generation, int):
        return None
    return generation


class RecoveryAck:
    """Ack the patrol's pause after ACK_FRAMES fresh allowed frames, never before."""

    def __init__(self, frames=ACK_FRAMES):
        self.frames, self.acked = frames, 0
        self.pending, self.since, self.count = None, None, 0

    def reset(self):
        """A new patrol counts its pauses from 1 again."""
        self.acked, self.pending, self.since, self.count = 0, None, None, 0

    def update(self, pause, allow, stamp, now):
        if pause is not None and pause != self.acked and pause != self.pending:
            self.pending, self.since, self.count = pause, now, 0
        if self.pending is not None:
            if not allow:
                self.count = 0
            elif stamp > self.since:
                self.count += 1
                if self.count >= self.frames:
                    self.acked, self.pending = self.pending, None
        return self.acked


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
    report_path = argv[4] if len(argv) > 4 else None
    ack, trace = RecoveryAck(), []
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
                ack.reset()
                continue
            now = time.time()
            allow, stamp = permit_from_line(line, now)
            pause = patrol_pause(report_path) if report_path else None
            acked = ack.update(pause, allow, stamp, now) if report_path else None
            write_permit(permit_path, owner, boot, allow, stamp, acked)
            if pause is not None or ack.pending is not None:
                # What the ack saw, line by line, while the patrol waits on one.
                trace.append({'now': round(now, 3), 'pause': pause, 'allow': allow,
                              'stamp': round(stamp, 3), 'since': ack.since,
                              'count': ack.count, 'acked': acked})
                with open(ACK_TRACE + '.tmp', 'w') as f:
                    json.dump(trace[-200:], f)
                os.rename(ACK_TRACE + '.tmp', ACK_TRACE)
    finally:
        write_permit(permit_path, owner, boot, False, time.time())


if __name__ == '__main__':
    main(sys.argv)
