"""X3Plus 決賽操作台 (loop console): relocalize, run the patrol loop, watch the map.

Runs on this PC, beside mission_loop.py (the YOLO/SAM2 models and the loop's
master live here, not on the Jetson). Open http://localhost:8090; a phone on the
same network can open http://<this PC>:8090.

    python ui/loop_console/server.py --host 10.53.253.252
    python ui/loop_console/server.py --simulate logs/mission_20261021_patrol19/mission.out

What it does, and nothing more:
  * Relocalize: runs integration/relocalize.py (the robot stays still).
  * Start the loop: runs integration/mission_loop.py --real --i-am-beside-the-robot,
    only after the operator confirms they stand at the robot's power switch.
  * Stop: ends that process, the robot's patrol, and sends the wheels a stop. The
    real emergency stop is still the power switch.
  * Map: walls, the patrol's allowed band, the placement box and the wall markers
    from the robot's own files, the AMCL pose twice a second, and the sugarboxes
    the mission reports as it sees them.
  * Self-check: services, both cameras (the arm camera drops off USB), LiDAR and
    localization, the arm and the chassis, the Jetson's memory, the PC's files.
No camera picture is shown (the consoles never do).
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from integration import placement_dock as dock  # noqa: E402  (stdlib-only module)

STATIC = HERE / 'static'
BASE_CSS = REPO / 'ui' / 'static' / 'style.css'    # the existing console's look
SCENE = dock.SCENE
LANDMARKS = SCENE + '/aruco_corner_landmarks.json'
ARM_CAMERA = 'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'
REAR_CAMERA = 'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
SERVICES = ('grasp-service', 'grasp-vision', 'x3plus-g2-localization', 'x3plus-g2-lidar',
            'x3plus-g2-rosbridge', 'x3plus-g2-roscore')
POSE_PERIOD_S = 0.5
POSE_STALE_S = 3.0           # no pose line for this long: the link counts as down
TRAIL_POINTS = 600
LOG_LINES = 300
MASK_CELL_M = 0.02           # the allowed band is sent at this resolution
CHECK_IDLE_PERIOD_S = 10.0
SSE_CLIENTS = 6
ARM_FRAME_MAX_AGE_S = 2.0
JETSON_MIN_FREE_MB = 500
# How long a stage usually takes, for the "about N s" the page shows. Relocalize
# took 56-58 s in the 10-21 runs, ~40 s of it the wall match on the Jetson, with
# the robot standing still all along, which reads like a hang without a clock.
STAGE_ETA_S = {'relocalizing': 60.0}

STAGES = {
    'idle': '待命', 'relocalizing': '重新定位中', 'localized': '已定位',
    'warming': '暖機中', 'patrol': '巡航中', 'confirming': '確認候選',
    'approach': '接近糖果盒', 'e1': '手臂相機確認', 'grasped': '夾取成功',
    'face_box': '轉向放置盒', 'to_box': '接近放置盒', 'released': '已放下',
    'back_out': '倒退', 'fixing': '修復中', 'done': '完成', 'stopped': '已停止',
    'error': '失敗',
}


def default_python():
    venv = REPO / '.venv-approach' / 'Scripts' / 'python.exe'
    return str(venv) if venv.is_file() else sys.executable


# ── robot side ────────────────────────────────────────────────────────────

POSE_SCRIPT = r'''
import json, sys, time
path = '/dev/shm/g2_localization_health.json'
while True:
    try:
        with open(path) as f:
            h = json.load(f)
        s = h.get('state') or {}
        out = {'ready': h.get('ready') is True, 'status': h.get('status'),
               'reason': h.get('reason'), 'pose': s.get('pose'),
               'turn_clear': s.get('turn_clear'),
               'age': time.time() - float(h.get('stamp', 0))}
    except Exception as exc:
        out = {'ready': False, 'status': 'unreadable', 'reason': str(exc)}
    sys.stdout.write(json.dumps(out) + '\n')
    sys.stdout.flush()
    time.sleep(%s)
''' % POSE_PERIOD_S

CHECK_SCRIPT = r'''
import json, os, subprocess, time
def run(cmd, timeout=10):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=timeout)
        return p.returncode, p.stdout.strip()
    except Exception as exc:
        return -1, str(exc)
out = {'services': {}}
for s in %(services)r:
    out['services'][s] = run(['systemctl', 'is-active', s])[1]
byid = '/dev/v4l/by-id'
names = os.listdir(byid) if os.path.isdir(byid) else []
out['arm_camera'] = %(arm)r in names
out['rear_camera'] = %(rear)r in names
out['rear_busy'] = out['rear_camera'] and run(['fuser', os.path.join(byid, %(rear)r)])[0] == 0
try:
    with open('/dev/shm/grasp_vision_frame.json') as f:
        out['arm_frame_age'] = time.time() - float(json.load(f).get('t', 0))
except Exception:
    out['arm_frame_age'] = None
code, text = run(['journalctl', '-k', '--since', '-10 min', '--no-pager'])
out['usb_disconnects'] = text.count('USB disconnect') if code == 0 else None
try:
    with open('/dev/shm/g2_localization_health.json') as f:
        h = json.load(f)
    out['localization'] = {'ready': h.get('ready') is True, 'status': h.get('status'),
                           'reason': h.get('reason'),
                           'age': time.time() - float(h.get('stamp', 0))}
except Exception as exc:
    out['localization'] = {'ready': False, 'status': 'unreadable', 'reason': str(exc)}
ctl = os.path.expanduser('~/Documents/deploy_jetson2/grasp/v23/graspctl.py')
for cmd in ('status', 'health'):
    code, text = run(['python3', ctl, cmd], timeout=20)
    try:
        out[cmd] = json.loads(text)
    except Exception:
        out[cmd] = {'error': text[-200:]}
mem = {}
with open('/proc/meminfo') as f:
    for line in f:
        k, v = line.split(':', 1)
        mem[k] = int(v.split()[0])
out['mem_free_mb'] = mem.get('MemAvailable', 0) / 1024.0
out['mem_total_mb'] = mem.get('MemTotal', 0) / 1024.0
out['load'] = os.getloadavg()[0]
out['patrol_running'] = run(['pgrep', '-f', '[g]2_map_patrol.py'])[0] == 0
print(json.dumps(out))
''' % {'services': SERVICES, 'arm': ARM_CAMERA, 'rear': REAR_CAMERA}


def ssh_argv(host, command):
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6', '-o', 'LogLevel=ERROR',
            'jetson@' + host, command]


class Robot:
    """Everything this console asks of the robot, over ssh, one call at a time."""

    def __init__(self, host):
        self.host = host

    def run(self, command, timeout=40):
        return subprocess.run(ssh_argv(self.host, command), capture_output=True, text=True,
                              timeout=timeout, encoding='utf-8', errors='replace')

    def check(self):
        result = self.run('python3 -c ' + shlex.quote(CHECK_SCRIPT), timeout=60)
        if result.returncode != 0:
            raise OSError((result.stderr or result.stdout or 'ssh failed').strip()[-300:])
        return json.loads(result.stdout.strip().splitlines()[-1])

    def pose_stream(self):
        return subprocess.Popen(ssh_argv(self.host, 'python3 -u -c ' + shlex.quote(POSE_SCRIPT)),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                                encoding='utf-8', errors='replace', bufsize=1)

    def map_files(self):
        layout = dock.fetch_layout(self.host, SCENE)
        try:
            mask = dock.fetch_nav_mask(self.host, SCENE)
        except (ValueError, ImportError, OSError, subprocess.SubprocessError):
            mask = None
        result = self.run('cat {}/{}'.format(dock.JETSON_ROOT, LANDMARKS), timeout=20)
        try:
            landmarks = json.loads(result.stdout)
        except ValueError:
            landmarks = None
        return layout, mask, landmarks

    def stop(self):
        """End the robot's patrol and send the wheels a stop, whatever is running."""
        try:
            self.run("pkill -TERM -f '[g]2_map_patrol.py'", timeout=15)
        except (OSError, subprocess.SubprocessError):
            pass
        with socket.create_connection((self.host, 7000), timeout=3) as sock:
            for _ in range(3):
                sock.sendall(b'{"action":"stop"}\n')
                time.sleep(0.05)

    def restart_arm_camera(self):
        return self.run('sudo -n systemctl reset-failed grasp-vision; '
                        'sudo -n systemctl restart grasp-vision', timeout=60)

    def stow_arm(self):
        """Open the jaw at E1, stow to travel, restart the service so its own
        startup check passes and the chassis channel comes up (2026-10-21)."""
        ctl = 'python3 ~/Documents/deploy_jetson2/grasp/v23/graspctl.py'
        return self.run('{0} home && {0} stow && sudo -n systemctl restart grasp-service'
                        .format(ctl), timeout=180)


# ── map ───────────────────────────────────────────────────────────────────

def map_payload(layout, mask=None, landmarks=None, cell=MASK_CELL_M):
    """What the browser draws: walls, box, markers and the allowed band (a grid
    over the walls' bounding box, '1' allowed)."""
    walls = dock.walls_from_layout(layout)
    if walls is None:
        raise ValueError('scene layout has no room_polygon')
    try:
        box = dock.box_from_layout(layout)
    except ValueError:
        box = None
    if box is not None:
        # The long edge's map heading, from the same entry box_from_layout used, so
        # the page draws the box turned as it stands (2026-10-29: drawn level, it
        # read as the long edge on the top wall when it ran along the right one).
        entry = layout['bin']
        pose = (entry.get('pose_map') if box['verified']
                else (entry.get('placement_update') or {}).get('midpoint_pose_map_candidate'))
        yaw = (pose or {}).get('yaw')
        box['yaw'] = yaw if isinstance(yaw, (int, float)) and math.isfinite(yaw) else None
    xs, ys = [p[0] for p in walls], [p[1] for p in walls]
    bounds = [min(xs), min(ys), max(xs), max(ys)]
    grid = None
    if mask is not None:
        cols = max(1, int(math.ceil((bounds[2] - bounds[0]) / cell)))
        rows = max(1, int(math.ceil((bounds[3] - bounds[1]) / cell)))
        grid = {'origin': bounds[:2], 'cell': cell, 'rows': [
            ''.join('1' if mask.allowed(bounds[0] + (c + 0.5) * cell,
                                        bounds[1] + (r + 0.5) * cell) else '0'
                    for c in range(cols)) for r in range(rows)]}
    markers = []
    for key, marker in sorted(((landmarks or {}).get('markers') or {}).items()):
        point = marker.get('point_map') if isinstance(marker, dict) else None
        if isinstance(point, list) and len(point) == 2:
            markers.append({'id': str(key), 'x': point[0], 'y': point[1]})
    return {'walls': walls, 'bounds': bounds, 'box': box, 'mask': grid, 'markers': markers}


# ── state ─────────────────────────────────────────────────────────────────

class Hub:
    """The console's one state, and the SSE listeners waiting for changes."""

    def __init__(self, host):
        self.lock = threading.Lock()
        self.listeners = []
        self.log = deque(maxlen=LOG_LINES)
        self.trail = deque(maxlen=TRAIL_POINTS)
        self.state = {
            'host': host, 'link': False, 'job': None, 'stage': 'idle', 'detail': '',
            'delivered': 0, 'failures': 0, 'lap': 0, 'started_at': None, 'ended_at': None,
            'pose': None, 'relocalized': None, 'sightings': [], 'checks': None,
            'map': None, 'map_error': None, 'stage_since': None, 'stage_eta_s': None,
        }
        self.next_sighting = 1

    def snapshot(self):
        with self.lock:
            data = dict(self.state)
            data['stage_text'] = STAGES.get(data['stage'], data['stage'])
            data['log'] = list(self.log)[-80:]
            data['trail'] = list(self.trail)
            return data

    def update(self, **changes):
        with self.lock:
            stage = changes.get('stage', self.state['stage'])
            if stage != self.state['stage'] or self.state['stage_since'] is None:
                changes.update(stage_since=time.time(), stage_eta_s=STAGE_ETA_S.get(stage))
            self.state.update(changes)
        self.broadcast()

    def note(self, text, kind='info'):
        with self.lock:
            self.log.append({'t': time.time(), 'text': text, 'kind': kind})
        self.broadcast()

    def broadcast(self):
        data = json.dumps(self.snapshot(), ensure_ascii=False)
        with self.lock:
            listeners = list(self.listeners)
        for q in listeners:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass

    def listen(self):
        q = queue.Queue(maxsize=8)
        with self.lock:
            if len(self.listeners) >= SSE_CLIENTS:
                return None
            self.listeners.append(q)
        return q

    def unlisten(self, q):
        with self.lock:
            if q in self.listeners:
                self.listeners.remove(q)

    def set_pose(self, pose):
        with self.lock:
            self.state['pose'] = pose
            if pose and pose.get('x') is not None and self.state['job'] == 'mission':
                last = self.trail[-1] if self.trail else None
                if not last or math.hypot(pose['x'] - last[0], pose['y'] - last[1]) > 0.02:
                    self.trail.append((round(pose['x'], 3), round(pose['y'], 3)))
        self.broadcast()

    def add_sighting(self, ahead, left, state, rough):
        """A sugarbox the mission reports in the chassis frame, placed on the map
        with the pose it has now (the robot is stopping or parked by then)."""
        with self.lock:
            pose = self.state['pose']
            if not pose or pose.get('x') is None or pose.get('yaw') is None:
                return None
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            point = (pose['x'] + c * ahead - s * left, pose['y'] + s * ahead + c * left)
            sightings = [d for d in self.state['sightings'] if d['state'] != 'candidate']
            if state == 'confirmed':
                sightings = [d for d in sightings
                             if not (d['state'] == 'confirmed'
                                     and math.hypot(d['x'] - point[0], d['y'] - point[1]) < 0.35)]
            sighting = {'id': self.next_sighting, 'x': round(point[0], 3),
                        'y': round(point[1], 3), 'state': state, 'rough': rough}
            self.next_sighting += 1
            sightings.append(sighting)
            self.state['sightings'] = sightings
        self.broadcast()
        return sighting


# ── what the mission and relocalization print ─────────────────────────────

NUM = r'[-+]?\d+(?:\.\d+)?'
PATTERNS = [
    ('reloc_seen', re.compile(r'^\[RELOC\] captured: markers \[(.*)\]')),
    ('reloc_match', re.compile(r'^\[RELOC\] matching the walls')),
    ('reloc_ok', re.compile(r'^\[RELOC\] localized at map \(({0}), ({0})\), heading ({0}) deg'
                            .format(NUM))),
    ('reloc_failed', re.compile(r'^\[RELOC\] (?:not localized|capture \d/\d failed): (.*)')),
    ('blocked', re.compile(r'^\[BLOCKED\] (.*)')),
    ('warming', re.compile(r'^\[MISSION\] warming models')),
    ('lap', re.compile(r'^\[PATROL\] lap (\d+) started')),
    ('candidate', re.compile(r'^\[PATROL\] candidate ({0}) m ahead, ({0}) m left( \(beyond)?'
                             .format(NUM))),
    ('confirmed', re.compile(r'^\[PATROL\] sugarbox ({0}) m ahead, ({0}) m left( \(beyond)?.*'
                             r'confirmed'.format(NUM))),
    ('unconfirmed', re.compile(r'^\[PATROL\] not confirmed')),
    ('patrol_note', re.compile(r'^\[PATROL\] (stopped before a turn|refused to start'
                               r'|stopped by a localization outage|localization back).*')),
    ('loc_wait', re.compile(r'^\[LOCALIZE\] (.*): waiting up to ({0}) s'.format(NUM))),
    ('loc_back', re.compile(r'^\[LOCALIZE\] back after ({0}) s'.format(NUM))),
    ('approach', re.compile(r'^\[SEED\] standing still|^\[TARGET\] memory locked')),
    ('arrived', re.compile(r'^\[ARRIVED\] ARRIVED_CONTRACT PURSUIT_DONE x=({0})'.format(NUM))),
    ('e1', re.compile(r'^\[E1\]|^\[CREEP\]')),
    ('grasped', re.compile(r'"outcome": "confirmed"')),
    ('face', re.compile(r'^\[FACE\] placement box ({0}) m away'.format(NUM))),
    ('released', re.compile(r'"outcome": "released"')),
    ('delivered', re.compile(r'^\[MISSION\] (\d+) delivered')),
    ('missed', re.compile(r'^\[MISSION\] grasp \d+ missed (?:with the robot at the box )?at map '
                          r'\(({0}), ({0})\)'.format(NUM))),
    ('backing', re.compile(r'^\[MISSION\] backing')),
    ('done', re.compile(r'^\[MISSION\] (?:no sugarbox in \d+ whole laps: done|\d+ delivered: done)')),
    ('stopped', re.compile(r'^\[MISSION\] stopped in ([^:]+): (.*)')),
    ('engine_stop', re.compile(r'^\[STOP(?:PED)?\] (.*)')),
    ('moved', re.compile(r'^\[ADJUST\] odometry moved ({0}) m'.format(NUM))),
    ('turned', re.compile(r'^\[TURN\] odometry turned ({0}) deg'.format(NUM))),
    ('rpc', re.compile(r'^\[RPC\] (.*)')),
]
# Anything else goes to the log only when it looks like trouble.
TROUBLE = re.compile(r'^\[WARN\]|Traceback|Error|error:|FAILED')


def parse(line):
    """(event, groups) for a line this console cares about, else None."""
    for name, pattern in PATTERNS:
        match = pattern.search(line)
        if match:
            return name, match.groups()
    return None


class Feed:
    """Turns one job's output lines into state changes and log entries."""

    def __init__(self, hub):
        self.hub = hub
        self.placing = False

    def line(self, text):
        text = text.rstrip()
        event = parse(text)
        if event is None:
            if TROUBLE.search(text):
                self.hub.note(text, 'warn')
            return
        name, g = event
        hub, update, note = self.hub, self.hub.update, self.hub.note
        if name == 'reloc_seen':
            update(stage='relocalizing', detail='看到標記 ' + g[0] + '，準備比對牆面')
            note('後相機看到標記 {}，比對牆面中'.format(g[0]))
        elif name == 'reloc_match':
            update(stage='relocalizing', detail='車上正在比對牆面（約 40 秒，車子不會動）')
        elif name == 'reloc_ok':
            x, y, heading = float(g[0]), float(g[1]), float(g[2])
            update(stage='localized', detail='',
                   relocalized={'x': x, 'y': y, 'yaw': math.radians(heading), 't': time.time()})
            note('定位成功：({:.2f}, {:.2f})，朝向 {:.0f}°'.format(x, y, heading), 'ok')
        elif name in ('reloc_failed', 'blocked'):
            update(detail=g[0])
            note(g[0], 'crit')
        elif name == 'warming':
            update(stage='warming', detail='')
            note('暖機中（模型載入一次）')
        elif name == 'lap':
            self.placing = False
            update(stage='patrol', lap=int(g[0]), detail='')
            note('巡航第 {} 圈開始'.format(g[0]))
        elif name == 'candidate':
            hub.add_sighting(float(g[0]), float(g[1]), 'candidate', bool(g[2]))
            update(stage='confirming', detail='前方 {} m、左 {} m'.format(g[0], g[1]))
            note('看到候選：前方 {} m、左 {} m{}'.format(g[0], g[1], '（粗估）' if g[2] else ''))
        elif name == 'confirmed':
            hub.add_sighting(float(g[0]), float(g[1]), 'confirmed', bool(g[2]))
            update(stage='approach', detail='')
            note('確認糖果盒：前方 {} m、左 {} m'.format(g[0], g[1]), 'ok')
        elif name == 'unconfirmed':
            with hub.lock:
                hub.state['sightings'] = [d for d in hub.state['sightings']
                                          if d['state'] != 'candidate']
            update(stage='patrol')
            note('候選沒有確認，繼續巡航')
        elif name == 'patrol_note':
            note(text[len('[PATROL] '):], 'warn')
        elif name == 'loc_wait':
            note('定位暫時中斷（{}），停車等待最多 {} 秒'.format(g[0], g[1]), 'warn')
        elif name == 'loc_back':
            note('定位恢復（等了 {} 秒），繼續'.format(g[0]))
        elif name == 'approach':
            update(stage='to_box' if self.placing else 'approach')
        elif name == 'arrived':
            update(stage='to_box' if self.placing else 'e1', detail='停在 x={}'.format(g[0]))
        elif name == 'e1':
            update(stage='e1')
        elif name == 'grasped':
            self.placing = True
            with hub.lock:
                hub.state['sightings'] = [d for d in hub.state['sightings']
                                          if d['state'] != 'confirmed']
            update(stage='grasped', detail='')
            note('夾取成功', 'ok')
        elif name == 'face':
            update(stage='face_box', detail='放置盒 {} m'.format(g[0]))
        elif name == 'released':
            self.placing = False
            update(stage='released', detail='')
            note('已放進放置盒', 'ok')
        elif name == 'delivered':
            update(delivered=int(g[0]))
        elif name == 'missed':
            with hub.lock:
                hub.state['sightings'] = [d for d in hub.state['sightings']
                                          if d['state'] not in ('candidate', 'confirmed')]
                failures = hub.state['failures'] + 1
                near = sum(1 for d in hub.state['sightings'] if d['state'] in ('failed', 'skipped')
                           and math.hypot(d['x'] - float(g[0]), d['y'] - float(g[1])) < 0.30)
                hub.state['sightings'].append({
                    'id': hub.next_sighting, 'x': float(g[0]), 'y': float(g[1]),
                    'state': 'skipped' if near else 'failed', 'rough': False})
                hub.next_sighting += 1
            update(failures=failures, stage='back_out')
            note(text[len('[MISSION] '):], 'warn')
        elif name == 'backing':
            update(stage='back_out')
            note(text[len('[MISSION] '):])
        elif name == 'done':
            update(stage='done', detail='')
            note(text[len('[MISSION] '):], 'ok')
        elif name == 'stopped':
            update(stage='stopped', detail=g[1])
            note('在 {} 停下：{}'.format(g[0], g[1]), 'crit')
        elif name == 'engine_stop':
            note(g[0], 'warn')
        elif name == 'moved':
            note('{} {:.2f} m'.format('前進' if float(g[0]) > 0 else '倒退', abs(float(g[0]))))
        elif name == 'turned':
            note('原地轉 {:+.0f}°'.format(float(g[0])))
        elif name == 'rpc':
            note('指令通道：' + g[0], 'warn')


# ── the console ───────────────────────────────────────────────────────────

class Console:
    """Jobs (one at a time), the pose stream, self-checks and the map."""

    def __init__(self, host, python=None, robot=None, popen=subprocess.Popen):
        self.host = host
        self.python = python or default_python()
        self.robot = robot or Robot(host)
        self.popen = popen
        self.hub = Hub(host)
        self.proc = None
        self.job_lock = threading.Lock()
        self.checking = threading.Lock()
        self.closed = threading.Event()

    # jobs
    def start_job(self, kind, argv, stage):
        with self.job_lock:
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError('已有工作在執行：{}'.format(self.hub.state['job']))
            # relocalize.py runs as a script yet imports `integration.`: the repo
            # root must be on the path (2026-10-29: ModuleNotFoundError, exit 1).
            env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1',
                       PYTHONPATH=os.pathsep.join(
                           p for p in (str(REPO), os.environ.get('PYTHONPATH')) if p))
            flags = getattr(subprocess, 'CREATE_NEW_PROCESS_GROUP', 0)
            self.proc = self.popen(argv, cwd=str(REPO), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                   errors='replace', bufsize=1, env=env, creationflags=flags)
            proc = self.proc
        if kind == 'mission':
            with self.hub.lock:
                self.hub.trail.clear()
                self.hub.state.update(delivered=0, failures=0, lap=0, sightings=[])
        detail = '拍後相機畫面、找牆上的標記' if stage == 'relocalizing' else ''
        self.hub.update(job=kind, stage=stage, detail=detail, started_at=time.time(),
                        ended_at=None)
        threading.Thread(target=self._pump, args=(kind, proc), daemon=True).start()

    def _pump(self, kind, proc):
        feed = Feed(self.hub)
        for line in proc.stdout:
            feed.line(line)
        code = proc.wait()
        stage = self.hub.state['stage']
        if code == 0 and kind == 'relocalize':
            stage = 'localized'
        elif code == 0 and kind == 'mission':
            stage = 'done' if stage != 'stopped' else stage
        elif code == 0 and kind == 'fix':
            stage = 'idle'
        elif stage not in ('stopped', 'done'):
            stage = 'error'
        self.hub.update(job=None, stage=stage, ended_at=time.time())
        self.hub.note('{} 結束（exit {}）'.format({'relocalize': '重新定位', 'mission': '任務',
                                                  'fix': '修復'}.get(kind, kind), code),
                      'ok' if code == 0 else 'warn')
        threading.Thread(target=self.self_check, daemon=True).start()

    def relocalize(self):
        self.start_job('relocalize', [self.python, '-u', 'integration/relocalize.py',
                                      '--host', self.host], 'relocalizing')

    def start_mission(self, max_deliveries=None, relocalize=False):
        argv = [self.python, '-u', 'integration/mission_loop.py', '--host', self.host,
                '--real', '--i-am-beside-the-robot']
        if relocalize:
            argv.append('--relocalize')
        if max_deliveries:
            argv += ['--max-deliveries', str(int(max_deliveries))]
        self.start_job('mission', argv, 'relocalizing' if relocalize else 'warming')

    def fix(self, what):
        if what not in ('arm_camera', 'arm_stow'):
            raise ValueError('unknown fix')
        script = ('import sys; sys.path.insert(0, %r); from server import Robot; '
                  'r = Robot(%r).%s(); print(r.stdout[-400:]); print(r.stderr[-400:]); '
                  'sys.exit(r.returncode)') % (str(HERE), self.host,
                                               'restart_arm_camera' if what == 'arm_camera'
                                               else 'stow_arm')
        self.start_job('fix', [self.python, '-u', '-c', script], 'fixing')
        self.hub.note('修復：{}'.format('重啟手臂相機服務' if what == 'arm_camera'
                                        else '收回手臂（home → stow → 重啟夾取服務）'))

    def stop(self):
        """End the job's process tree, then the robot's patrol and the wheels."""
        with self.job_lock:
            proc = self.proc
        if proc is not None and proc.poll() is None:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(proc.pid)],
                               capture_output=True)
            else:
                proc.send_signal(signal.SIGINT)
        try:
            self.robot.stop()
            self.hub.note('已送出停止：巡航結束、輪子停止', 'warn')
        except OSError as exc:
            self.hub.note('停止指令沒送到機器人：{}。請用電源開關'.format(exc), 'crit')
        self.hub.update(stage='stopped', detail='操作員停止')

    # pose
    def pose_loop(self):
        while not self.closed.is_set():
            try:
                stream = self.robot.pose_stream()
            except OSError as exc:
                self.hub.update(link=False)
                self.hub.note('定位串流無法啟動：{}'.format(exc), 'warn')
                self.closed.wait(5)
                continue
            last = time.monotonic()
            watchdog = threading.Thread(target=self._pose_watchdog, args=(stream, lambda: last),
                                        daemon=True)
            watchdog.start()
            for line in self._lines(stream):
                try:
                    data = json.loads(line)
                except ValueError:
                    continue
                last = time.monotonic()
                pose = data.get('pose')
                fields = {'ready': data.get('ready'), 'status': data.get('status'),
                          'reason': data.get('reason'), 'age': data.get('age'),
                          'turn_clear': data.get('turn_clear'), 'x': None, 'y': None, 'yaw': None}
                if isinstance(pose, list) and len(pose) == 3:
                    fields.update(x=pose[0], y=pose[1], yaw=pose[2])
                if not self.hub.state['link']:
                    self.hub.update(link=True)
                self.hub.set_pose(fields)
            stream.wait()
            self.hub.update(link=False)
            self.closed.wait(3)

    @staticmethod
    def _lines(stream):
        try:
            for line in stream.stdout:
                yield line
        except (OSError, ValueError):       # the watchdog killed it mid-read
            return

    def _pose_watchdog(self, stream, last):
        while stream.poll() is None and not self.closed.is_set():
            if time.monotonic() - last() > POSE_STALE_S:
                self.hub.update(link=False)
                stream.kill()
                return
            time.sleep(0.5)

    # checks
    def self_check(self):
        if not self.checking.acquire(blocking=False):
            return
        try:
            try:
                report = self.robot.check()
                items = assess(report, self.hub.state['job'])
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                items = [item('ssh', '連線', 'crit', '連不到機器人：{}'.format(exc))]
            items += pc_checks()
            self.hub.update(checks={'at': time.time(), 'items': items})
        finally:
            self.checking.release()

    def check_loop(self):
        while not self.closed.is_set():
            if self.hub.state['job'] is None:
                self.self_check()
            self.closed.wait(CHECK_IDLE_PERIOD_S)

    # map
    def load_map(self):
        try:
            layout, mask, landmarks = self.robot.map_files()
            self.hub.update(map=map_payload(layout, mask, landmarks), map_error=None)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            self.hub.update(map_error=str(exc))

    def start(self):
        for target in (self.load_map, self.pose_loop, self.check_loop):
            threading.Thread(target=target, daemon=True).start()


def item(key, label, level, detail, fix=None):
    return {'key': key, 'label': label, 'level': level, 'detail': detail, 'fix': fix}


def assess(r, job=None):
    """The robot's check report as rows: ok / warn / crit, with a fix where one exists."""
    rows = [item('ssh', '連線', 'ok', 'ssh 正常')]
    down = [s for s, v in (r.get('services') or {}).items() if v != 'active']
    rows.append(item('services', '車上服務', 'crit' if down else 'ok',
                     '未執行：' + '、'.join(down) if down else '{} 個都在執行'.format(len(SERVICES))))
    age = r.get('arm_frame_age')
    if not r.get('arm_camera'):
        rows.append(item('arm_camera', '手臂相機', 'crit',
                         'USB 沒偵測到：重插線，換 hub 上另一個孔', 'arm_camera'))
    elif age is None or age > ARM_FRAME_MAX_AGE_S:
        rows.append(item('arm_camera', '手臂相機', 'crit',
                         '有接上但沒有新畫面' + ('' if age is None else '（{:.0f} s 前）'.format(age)),
                         'arm_camera'))
    else:
        rows.append(item('arm_camera', '手臂相機', 'ok', '畫面 {:.1f} s 前'.format(age)))
    drops = r.get('usb_disconnects')
    if drops:
        rows.append(item('usb', 'USB 斷線', 'warn',
                         '最近 10 分鐘 {} 次：手臂相機的線可能鬆了'.format(drops)))
    if not r.get('rear_camera'):
        rows.append(item('rear_camera', '後相機', 'crit', 'USB 沒偵測到'))
    elif r.get('rear_busy') and job is None:
        rows.append(item('rear_camera', '後相機', 'ok', '串流中'))
    else:
        rows.append(item('rear_camera', '後相機', 'ok', '已接上'))
    loc = r.get('localization') or {}
    if loc.get('ready') and (loc.get('age') or 99) < 2:
        rows.append(item('localization', '定位', 'ok', '就緒'))
    else:
        rows.append(item('localization', '定位', 'warn',
                         '未就緒：{}。按「重新定位」'.format(loc.get('reason') or loc.get('status'))))
    status, health = r.get('status') or {}, r.get('health') or {}
    chassis = status.get('chassis') or {}
    pose = chassis.get('arm_pose')
    if 'error' in status:
        rows.append(item('arm', '手臂', 'crit', '夾取服務沒有回應'))
    elif pose == 'travel':
        rows.append(item('arm', '手臂', 'ok', '行駛姿態'))
    elif pose == 'startup_failed':
        rows.append(item('arm', '手臂', 'warn', '開機沒收回（夾爪沒全開）', 'arm_stow'))
    elif job is None:
        rows.append(item('arm', '手臂', 'warn', '姿態 {}'.format(pose), 'arm_stow'))
    else:
        rows.append(item('arm', '手臂', 'ok', '姿態 {}'.format(pose)))
    holding = health.get('holding')
    if holding is True and job is None:
        rows.append(item('jaw', '夾爪', 'warn', '夾著東西：先拿出來'))
    elif holding is None:
        rows.append(item('jaw', '夾爪', 'crit', '讀不到夾爪狀態'))
    else:
        rows.append(item('jaw', '夾爪', 'ok', '夾著盒子' if holding else '空'))
    if health.get('chassis_tcp_ready') is not True:
        rows.append(item('chassis', '底盤', 'crit', '控制通道未就緒（手臂收回後才會開）',
                         'arm_stow' if pose == 'startup_failed' else None))
    else:
        rows.append(item('chassis', '底盤', 'ok',
                         '就緒' + ('（已鎖，任務會自己解鎖）' if chassis.get('maintenance') else '')))
    free = r.get('mem_free_mb') or 0
    rows.append(item('memory', 'Jetson 記憶體', 'warn' if free < JETSON_MIN_FREE_MB else 'ok',
                     '剩 {:.0f} MB'.format(free)))
    if r.get('patrol_running') and job is None:
        rows.append(item('patrol', '巡航程式', 'warn', '還在車上執行：按「停止」'))
    return rows


def pc_checks():
    base = Path(os.environ.get('SUGARBOX_ASSET_DIR', REPO / 'detection'))
    files = {
        '糖果盒模型': base / 'models' / 'best.pt',
        '放置盒模型': base / 'models' / 'placement_box.pt',
        'SAM2 模型': base / 'models' / 'sam2.1_b.pt',
        '後相機校正': base / 'rear_ground_homography.json',
        'GStreamer': REPO / '.venv-approach' / 'Lib' / 'site-packages' / 'gstreamer_cli' / 'bin'
                     / 'gst-launch-1.0.exe',
    }
    missing = [name for name, path in files.items() if not path.is_file()]
    if os.environ.get('GSTREAMER_LAUNCH_EXE') and 'GStreamer' in missing:
        missing.remove('GStreamer')
    return [item('pc', '這台電腦', 'crit' if missing else 'ok',
                 '缺：' + '、'.join(missing) if missing else '模型、校正、串流接收都在')]


# ── replay (no robot) ─────────────────────────────────────────────────────

class ReplayRobot:
    """A robot for trying the screen without one: a pose going round the room, a
    canned self-check, and the map from the repo's copy of the scene."""

    def __init__(self, log_path):
        self.log_path = Path(log_path)
        self.started = time.monotonic()

    def map_files(self):
        folder = REPO / SCENE
        layout_path = folder / 'scene_layout.json'
        if not layout_path.is_file():
            layout_path = self.log_path.parent / 'scene_layout.json'
        layout = json.loads(layout_path.read_text(encoding='utf-8'))
        mask = None
        mask_yaml = folder / 'navigation_center_mask.yaml'
        if mask_yaml.is_file():
            meta = {}
            for line in mask_yaml.read_text(encoding='utf-8').splitlines():
                if ':' in line and not line.startswith(' ') and not line.startswith('-'):
                    k, v = line.split(':', 1)
                    meta[k.strip()] = v.strip()
            lines = mask_yaml.read_text(encoding='utf-8').split('origin:')[1].splitlines()[1:3]
            origin = [float(line.strip()[1:]) for line in lines]   # '- -2.0': drop the list dash
            width, height, cells = dock.parse_pgm((folder / meta['image']).read_bytes())
            mask = dock.NavMask(width, height, cells, float(meta['resolution']), origin)
        landmarks = None
        for path in (folder / 'aruco_corner_landmarks.json',):
            if path.is_file():
                landmarks = json.loads(path.read_text(encoding='utf-8'))
        return layout, mask, landmarks

    def check(self):
        return {'services': {s: 'active' for s in SERVICES}, 'arm_camera': True,
                'rear_camera': True, 'rear_busy': False, 'arm_frame_age': 0.3,
                'usb_disconnects': 0, 'localization': {'ready': True, 'age': 0.1},
                'status': {'chassis': {'arm_pose': 'travel', 'maintenance': False}},
                'health': {'holding': False, 'chassis_tcp_ready': True},
                'mem_free_mb': 1500, 'patrol_running': False}

    def pose_stream(self):
        script = ('import json, math, sys, time\n'
                  't0 = time.time()\n'
                  'while True:\n'
                  '    a = (time.time() - t0) / 20.0\n'
                  '    x, y = 0.75 + 0.55 * math.cos(a), 0.45 + 0.55 * math.sin(a)\n'
                  '    print(json.dumps({"ready": True, "status": "localized", "age": 0.1,'
                  ' "pose": [x, y, a + math.pi / 2], "turn_clear": True}), flush=True)\n'
                  '    time.sleep(0.5)\n')
        return subprocess.Popen([sys.executable, '-u', '-c', script], stdout=subprocess.PIPE,
                                text=True, bufsize=1)

    def stop(self):
        pass


def replay_argv(log_path, python):
    """A job that prints the recorded mission output at a readable pace, and ends
    with the exit code mission_loop gave it: 2 for a mission that stopped."""
    script = ('import sys, time\n'
              'code = 0\n'
              'for line in open(%r, encoding="utf-8", errors="replace"):\n'
              '    if line.startswith(("[RUN]", "[VISION_LAT]")):\n'
              '        continue\n'
              '    if line.startswith("[MISSION] stopped in"):\n'
              '        code = 2\n'
              '    print(line.rstrip(), flush=True)\n'
              '    time.sleep(0.35)\n'
              'sys.exit(code)\n') % str(log_path)
    return [python, '-u', '-c', script]


# ── HTTP ──────────────────────────────────────────────────────────────────

TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
         '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml'}


def handler_for(console, simulate_log=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def send(self, code, body, ctype='application/json; charset=utf-8'):
            data = body if isinstance(body, bytes) else body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def json(self, code, value):
            self.send(code, json.dumps(value, ensure_ascii=False))

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                return self.file(STATIC / 'index.html')
            if path == '/static/base.css':
                return self.file(BASE_CSS)
            if path.startswith('/static/'):
                target = (STATIC / path[len('/static/'):]).resolve()
                if STATIC.resolve() not in target.parents:
                    return self.json(404, {'error': 'not found'})
                return self.file(target)
            if path == '/api/state':
                return self.json(200, console.hub.snapshot())
            if path == '/api/events':
                return self.events()
            return self.json(404, {'error': 'not found'})

        def file(self, target):
            if not target.is_file():
                return self.json(404, {'error': 'not found'})
            self.send(200, target.read_bytes(), TYPES.get(target.suffix, 'application/octet-stream'))

        def events(self):
            q = console.hub.listen()
            if q is None:
                return self.json(503, {'error': '連線數已滿'})
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                self.wfile.write(('data: ' + json.dumps(console.hub.snapshot(), ensure_ascii=False)
                                  + '\n\n').encode('utf-8'))
                self.wfile.flush()
                while not console.closed.is_set():
                    try:
                        data = q.get(timeout=15)
                        while not q.empty():          # only the newest matters
                            data = q.get_nowait()
                        self.wfile.write(('data: ' + data + '\n\n').encode('utf-8'))
                    except queue.Empty:
                        self.wfile.write(b': keep-alive\n\n')
                    self.wfile.flush()
            except (OSError, ValueError):
                pass
            finally:
                console.hub.unlisten(q)

        def do_POST(self):
            path = urlparse(self.path).path
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}') if length else {}
            except ValueError:
                return self.json(400, {'error': '格式錯誤'})
            if not isinstance(body, dict):
                return self.json(400, {'error': '格式錯誤'})
            try:
                if path == '/api/stop':
                    console.stop()
                    return self.json(200, {'ok': True})
                if path == '/api/check':
                    threading.Thread(target=console.self_check, daemon=True).start()
                    return self.json(200, {'ok': True})
                if path == '/api/map':
                    threading.Thread(target=console.load_map, daemon=True).start()
                    return self.json(200, {'ok': True})
                if path == '/api/relocalize':
                    console.relocalize()
                    return self.json(200, {'ok': True})
                if path in ('/api/start', '/api/fix'):
                    moves = path == '/api/start' or body.get('what') == 'arm_stow'
                    if moves and body.get('beside_robot') is not True:
                        return self.json(400, {'error': '要先確認人在車旁、手放電源開關'})
                    if path == '/api/fix':
                        console.fix(body.get('what'))
                    elif simulate_log is not None:
                        console.start_job('mission', replay_argv(simulate_log, sys.executable),
                                          'warming')
                    else:
                        max_deliveries = body.get('max_deliveries')
                        if max_deliveries not in (None, '') and (
                                not str(max_deliveries).isdigit() or int(max_deliveries) < 1):
                            return self.json(400, {'error': '送達上限要是正整數'})
                        console.start_mission(max_deliveries or None,
                                              relocalize=body.get('relocalize') is True)
                    return self.json(200, {'ok': True})
            except RuntimeError as exc:
                return self.json(409, {'error': str(exc)})
            except ValueError as exc:
                return self.json(400, {'error': str(exc)})
            except (OSError, subprocess.SubprocessError) as exc:
                return self.json(500, {'error': str(exc)})
            return self.json(404, {'error': 'not found'})

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='10.53.253.252', help='the robot (Jetson) address')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--bind', default='0.0.0.0',
                        help='0.0.0.0 lets a phone on the same network open it')
    parser.add_argument('--python', default=None,
                        help='python for the mission (default: the repo .venv-approach)')
    parser.add_argument('--simulate', type=Path, default=None, metavar='MISSION_OUT',
                        help='no robot: replay a recorded mission.out, a pose going round the room')
    args = parser.parse_args(argv)
    robot = ReplayRobot(args.simulate) if args.simulate else None
    console = Console(args.host, python=args.python, robot=robot)
    if args.simulate:
        console.relocalize = lambda: console.start_job(
            'relocalize', [sys.executable, '-u', '-c',
                           'import time; print("[RELOC] captured: markers [2, 7]", flush=True); '
                           'time.sleep(1.5); print("[RELOC] localized at map (0.22, 0.24), '
                           'heading -46 deg, from markers [2, 7] (2 s)", flush=True)'],
            'relocalizing')
    console.start()
    server = ThreadingHTTPServer((args.bind, args.port), handler_for(console, args.simulate))
    server.daemon_threads = True
    print('X3Plus 決賽操作台：http://localhost:{}  （機器人 {}{}）'.format(
        args.port, args.host, '，重播模式' if args.simulate else ''), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        console.closed.set()
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
