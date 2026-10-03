#!/usr/bin/env python3
"""The operator console: a web UI served by the robot itself.

Run this on the Jetson.  Point a phone at http://<jetson>:8080 and the whole
mission is operable without a terminal: configure it, start it, watch it, stop it.

    python3 ui/server.py                     # watch only; cannot drive
    python3 ui/server.py --allow-real        # enable the first real-mode gate
    python3 ui/server.py --simulate          # no robot needed; for development

Three deliberate choices, each of which the obvious alternative gets wrong:

*Standard library only.*  Jetson Nano runs Python 3.8 on aarch64, where a pip
install of FastAPI drags in a pydantic that has no wheel and has to compile
Rust.  On a machine that takes four minutes to import torch, "just pip install
it" is how a demo dies the morning of.  ThreadingHTTPServer and Server-Sent
Events are already there and are enough: status only ever flows server to
browser, and commands are ordinary POSTs.

*This process never touches the hardware.*  Exactly one process may hold
/dev/myserial, and that process is the mission.  The console starts it, reads
its telemetry, and signals it.  A web server that also opened the serial port
would be a second owner, and the failure mode of two owners on a half-duplex
servo bus is an arm that moves when nobody asked it to.

*Real mode fails closed.*  --allow-real and the operator acknowledgement are
necessary first gates.  The current console cannot supply the mode-specific
serial, LiDAR and camera-calibration evidence, so A/B/C real requests are still
refused.  Dry-run and simulation remain available.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
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
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STATIC = HERE / "static"

sys.path.insert(0, str(ROOT / "integration"))
import mission_status                                            # noqa: E402

DEFAULT_PORT = 8080
DEFAULT_STATUS_PORT = 8099
# Browsers give up on an idle SSE stream and reconnect; a comment line often
# enough keeps it open without pretending a stale mission is still reporting.
SSE_KEEPALIVE_S = 15.0
# How long a mission gets to stop cleanly before we say so.  It cannot be
# interrupted mid-grasp -- the episode is a blocking call -- so this is longer
# than a tick and shorter than a human's patience.
STOP_GRACE_S = 8.0
MAX_BODY_BYTES = 64 * 1024
# Fewer than before: the log is only there to show what the mission printed,
# and every line is a Python string held for the life of the run.
MAX_LOG_LINES = 200
# One phone and a laptop is the realistic case; the rest of the allowance is
# for reconnects that have not been reaped yet.
MAX_SSE_CLIENTS = 4

MODES = {
    "A": "mission_pipeline.py",
    "B": "vision_grasp_bridge.py",
    "C": "nav_rl_grasp_pipeline.py",
}
# Anything the operator can put in a command line is checked before it becomes
# an argv entry.  The console builds the command, but the request comes off the
# network, and "the UI would never send that" is not a property the server gets
# to assume.
#
# \A and \Z, not ^ and $: in Python '$' also matches immediately before a
# trailing newline, so '^...$' accepts "route.yaml\n" -- a value that looks
# validated and is not.
#
# Both patterns require the first character to be alphanumeric.  A value
# starting with '-' would arrive at argparse as a flag rather than as the value
# of the option it was meant to fill.
SAFE_NAME = re.compile(r"\A[A-Za-z0-9_][A-Za-z0-9._-]{0,63}\Z")
SAFE_PATH = re.compile(r"\A[A-Za-z0-9_][A-Za-z0-9._/-]{0,119}\Z")


def safe_relative_path(text: str) -> bool:
    """A path inside the project, on any platform.

    ``Path('/etc/passwd').is_absolute()`` is False on Windows -- there is no
    drive letter -- so the console would develop clean here and accept an
    absolute path on the Jetson, which is the machine that matters.  The
    leading-separator test is checked on the string itself for that reason.
    """
    if not SAFE_PATH.match(text):
        return False
    if text[0] in "/\\" or ":" in text[:2]:
        return False
    return ".." not in Path(text.replace("\\", "/")).parts


def memory() -> Dict:
    """How much room is left on the robot, and how much of it is this console.

    The operator's real worry on a Nano is headroom, and today the only way to
    see it is to SSH in and run free -m -- which is exactly the terminal this
    console exists to replace.  Two small file reads, no dependency on psutil.

    Returns empty values off Linux; the development machine's memory is not
    interesting and inventing a number would be worse than showing none.
    """
    out: Dict[str, Any] = {"total_mb": None, "available_mb": None,
                           "used_pct": None, "console_mb": None}
    try:
        fields = {}
        with open("/proc/meminfo", "r") as fh:
            for line in fh:
                key, _, rest = line.partition(":")
                fields[key] = float(rest.strip().split()[0]) / 1024.0   # kB -> MB
        total = fields.get("MemTotal")
        # MemAvailable, not MemFree: the kernel's own estimate of what a new
        # allocation could actually get, which counts reclaimable cache. MemFree
        # on a box with a warm page cache reads alarmingly low for no reason.
        avail = fields.get("MemAvailable", fields.get("MemFree"))
        if total and avail is not None:
            out["total_mb"] = round(total)
            out["available_mb"] = round(avail)
            out["used_pct"] = round(100.0 * (1.0 - avail / total))
    except (OSError, ValueError, IndexError):
        pass
    try:
        with open("/proc/self/statm", "r") as fh:
            pages = int(fh.read().split()[1])
        out["console_mb"] = round(pages * os.sysconf("SC_PAGE_SIZE") / 1e6, 1)
    except (OSError, ValueError, IndexError, AttributeError):
        pass
    return out


def clamp(v, lo, hi, default):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    if f != f or f in (float("inf"), float("-inf")):
        return default
    return max(lo, min(hi, f))


# ════════════════════════════════════════════════════════════════════════════
# Broadcast
# ════════════════════════════════════════════════════════════════════════════

class Hub:
    """Fan one event out to every open browser, without one slow client
    blocking the others.

    Each subscriber gets a bounded queue.  A phone that went to sleep stops
    draining its queue; rather than backing the whole console up behind it, its
    oldest frames are dropped.  Telemetry is a repeating snapshot, so a client
    that misses frames is merely behind, not broken.
    """

    def __init__(self, maxsize: int = 32, max_subs: int = MAX_SSE_CLIENTS):
        self._subs: List[queue.Queue] = []
        self._lock = threading.Lock()
        self._maxsize = maxsize
        self._max_subs = max_subs

    def subscribe(self) -> Optional[queue.Queue]:
        """A queue for one browser, or None if the console is already full.

        Each stream holds a thread for as long as it is open.  A phone that
        keeps reconnecting without the old connection being noticed as dead
        would otherwise accumulate them, and threads are not free on a box with
        no memory to spare.
        """
        q: queue.Queue = queue.Queue(maxsize=self._maxsize)
        with self._lock:
            if len(self._subs) >= self._max_subs:
                return None
            self._subs.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._subs:
                self._subs.remove(q)

    def publish(self, event: str, data: Dict) -> None:
        payload = (event, data)
        with self._lock:
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(payload)
            except queue.Full:
                try:
                    q.get_nowait()            # drop the oldest, keep the newest
                    q.put_nowait(payload)
                except (queue.Empty, queue.Full):
                    pass

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._subs)


# ════════════════════════════════════════════════════════════════════════════
# Mission supervision
# ════════════════════════════════════════════════════════════════════════════

class MissionProcess:
    """Owns the one child process that owns the robot."""

    def __init__(self, hub: Hub, repo: Path, python: str):
        self.hub = hub
        self.repo = repo
        self.python = python
        self.proc: Optional[subprocess.Popen] = None
        self.argv: List[str] = []
        self.started_at = 0.0
        self.log: List[str] = []
        self.exit_code: Optional[int] = None
        self.estop_latched = False
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def snapshot(self) -> Dict:
        return {
            "running": self.running,
            "pid": self.proc.pid if self.proc else None,
            "argv": self.argv,
            "command": " ".join(shlex.quote(a) for a in self.argv),
            "started_at": self.started_at,
            "uptime": (time.time() - self.started_at) if self.running else 0.0,
            "exit_code": self.exit_code,
            "estop_latched": self.estop_latched,
        }

    # ── lifecycle ──
    def start(self, argv: List[str]) -> Dict:
        with self._lock:
            if self.running:
                return {"ok": False, "error": "任務已在執行中。請先停止目前的任務。"}
            if self.estop_latched:
                return {"ok": False,
                        "error": "停止鍵仍在鎖定狀態，必須先解除才能開始新的任務。"}
            self.log = []
            self.exit_code = None
            self.argv = argv
            try:
                self.proc = subprocess.Popen(
                    argv, cwd=str(self.repo),
                    # stdin is the operator's Enter key: the mission blocks on
                    # input() before it drives and again when it pauses, and
                    # holding the pipe is how a button on a phone answers it.
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace", bufsize=1,
                    env=dict(os.environ, PYTHONUNBUFFERED="1",
                             PYTHONIOENCODING="utf-8"),
                )
            except OSError as exc:
                self.proc = None
                return {"ok": False, "error": f"無法啟動任務程序：{exc}"}
            self.started_at = time.time()
        threading.Thread(target=self._pump, daemon=True).start()
        self.hub.publish("process", self.snapshot())
        return {"ok": True, "pid": self.proc.pid}

    def _pump(self) -> None:
        """Drain the child's stdout, keeping only a short tail.

        Draining is not optional: a pipe nobody reads fills up and blocks the
        mission mid-drive.  Broadcasting it is optional, and it is not done --
        no screen shows it, so every line would be an SSE frame to every phone
        for nothing.  The tail stays for diagnosing a mission that died, which
        is readable at /api/state.
        """
        proc = self.proc
        assert proc is not None and proc.stdout is not None
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                self.log.append(line)
                del self.log[:-MAX_LOG_LINES]
            self.exit_code = proc.wait()
        finally:
            for stream in (proc.stdout, proc.stdin):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass
        self.hub.publish("process", self.snapshot())

    def confirm(self) -> Dict:
        """Answer the mission's input() -- start, or clear a pause."""
        if not self.running or self.proc is None or self.proc.stdin is None:
            return {"ok": False, "error": "沒有正在等待確認的任務。"}
        try:
            self.proc.stdin.write("\n")
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            return {"ok": False, "error": f"無法送出確認：{exc}"}
        return {"ok": True}

    def stop(self, *, estop: bool = False) -> Dict:
        """Ask the mission to stop, the same way Ctrl+C does.

        SIGINT and not SIGKILL, deliberately.  The mission's own shutdown path
        is what zeroes the wheels and releases the servo bus; killing it outright
        would skip that and leave a robot driving with nothing holding the port.
        A grasp episode is a blocking call, so a stop pressed mid-grasp lands
        when the episode returns -- which is why the physical power switch, not
        this button, is the emergency stop.
        """
        if estop:
            self.estop_latched = True
            self.hub.publish("estop", {"latched": True, "t": time.time()})
        if not self.running or self.proc is None:
            return {"ok": True, "note": "沒有正在執行的任務。"}
        try:
            if os.name == "nt":
                self.proc.send_signal(signal.CTRL_BREAK_EVENT)   # pragma: no cover
            else:
                self.proc.send_signal(signal.SIGINT)
        except (OSError, ValueError) as exc:
            return {"ok": False, "error": f"無法送出停止訊號：{exc}"}
        deadline = time.time() + STOP_GRACE_S
        while time.time() < deadline and self.running:
            time.sleep(0.1)
        if self.running:
            return {"ok": True, "stopping": True,
                    "note": "已送出停止訊號，但任務尚未結束 —— 夾取動作無法中途打斷，"
                            "會在該次動作完成後停止。若機器人仍在移動，請按電源開關。"}
        return {"ok": True, "stopping": False}

    def clear_estop(self) -> Dict:
        self.estop_latched = False
        self.hub.publish("estop", {"latched": False, "t": time.time()})
        return {"ok": True}


# ════════════════════════════════════════════════════════════════════════════
# Console state
# ════════════════════════════════════════════════════════════════════════════

class Console:
    def __init__(self, args):
        self.args = args
        self.hub = Hub()
        self.mission = MissionProcess(self.hub, ROOT, sys.executable)
        self.status: Dict[str, Any] = {}
        self.status_at = 0.0
        self._lock = threading.Lock()

    # ── telemetry in ──
    def serve_status(self, port: int, host: str = "127.0.0.1") -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        while True:
            try:
                data, _ = s.recvfrom(65535)
            except OSError:
                return
            try:
                frame = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(frame, dict):
                continue
            with self._lock:
                self.status = frame
                self.status_at = time.time()
            self.hub.publish("status", frame)

    def snapshot(self) -> Dict:
        with self._lock:
            status, at = dict(self.status), self.status_at
        return {
            "memory": memory(),
            "status": status,
            "status_age": (time.time() - at) if at else None,
            "process": self.mission.snapshot(),
            "allow_real": bool(self.args.allow_real),
            "simulate": bool(self.args.simulate),
            "log": self.mission.log[-40:],
            "server_time": time.time(),
        }

    # ── route ──
    def route(self, path: str = "") -> Dict:
        """The patrol polyline and the bin, in map-frame metres.

        Read on demand rather than cached at startup: the route file lives
        outside this repo and gets re-exported, and a console showing last
        week's route beside this week's live pose would be worse than showing
        no route at all.
        """
        if path and not safe_relative_path(path):
            return {"ok": False, "error": "路線必須是專案內的相對路徑。"}
        try:
            sys.path.insert(0, str(ROOT / "integration"))
            import map_goal_provider as mgp
            target = str(ROOT / path) if path else str(mgp.DEFAULT_ROUTE_HINT)
            if not Path(target).exists():
                return {"ok": False,
                        "error": f"找不到路線檔：{target}。地圖會維持空白，"
                                 f"其餘功能不受影響。"}
            spec = mgp.load_route(target, resample_m=0.75)
        except Exception as exc:
            return {"ok": False, "error": f"路線讀取失敗：{type(exc).__name__}: {exc}"}
        return {
            "ok": True,
            "source": getattr(spec, "source_path", target),
            "loop": bool(spec.loop),
            "waypoints": [{"id": w.id, "x": w.x, "y": w.y} for w in spec.waypoints],
            "bin": list(spec.bin_center) if spec.bin_center else None,
            "bin_approach": list(spec.bin_approach) if spec.bin_approach else None,
        }

    # ── command building ──
    def build_argv(self, cfg: Dict) -> Dict:
        """Turn console settings into the command line, or explain the refusal.

        Every value is bounded here rather than trusted.  The console is the
        only intended caller, but it reaches this over the network and the
        result becomes an argv for a process that can move a robot.
        """
        mode = str(cfg.get("mode", "A")).upper()
        if mode not in MODES:
            return {"error": f"未知的執行方式 {mode!r}。"}

        real = bool(cfg.get("real"))
        unlocked = bool(cfg.get("unlock"))
        if real and not self.args.allow_real:
            return {"error": "這台伺服器不允許驅動硬體。請在機器人上以 "
                             "--allow-real 重新啟動操作台。"}
        if real and not unlocked:
            return {"error": "驅動硬體前必須先在「任務設定」勾選安全確認。"}
        if real and self.args.simulate:
            return {"error": "模擬模式不能驅動硬體。"}
        if real:
            # The three children do not share a real-mode contract. Mode A
            # requires serial-owner and measured LiDAR/camera evidence, mode B
            # is only the vision sender and needs a separately managed receiver
            # plus a homography, and mode C deliberately refuses integrated
            # real grasp. A single web checkbox cannot truthfully provide any
            # of those facts, so fail here instead of launching an argv that
            # either argparse rejects or that weakens a child safety gate.
            reasons = {
                "A": "方式 A 的實機啟動需要 serial owner、LiDAR 方向與手臂相機校正證據；目前操作台尚未提供這些欄位。",
                "B": "方式 B 只有視覺傳送端；實機模式還需要指定 homography 與獨立的夾取接收端。",
                "C": "方式 C 的整合實機夾取目前由 runtime 安全閘停用。",
            }
            return {"error": reasons[mode]}

        script = ROOT / "integration" / MODES[mode]
        if not script.exists():
            return {"error": f"找不到 {script.name}。"}
        argv = [sys.executable, str(script)]

        height_cm = clamp(cfg.get("height_cm"), 0.5, 30.0, 6.5)
        cls = str(cfg.get("cls", "sugarbox"))
        if not SAFE_NAME.match(cls):
            return {"error": "物體類別名稱只能是英數字、底線、句點與連字號，且須以英數字開頭。"}
        argv += ["--class-height", f"{cls}={height_cm / 100.0:.3f}"]

        if mode == "A":
            route = str(cfg.get("route", "")).strip()
            if route:
                if not safe_relative_path(route):
                    return {"error": "路線必須是專案內的相對路徑，且不能包含 .. 或特殊字元。"}
                argv += ["--route", route]
            argv += ["--max-laps", str(int(clamp(cfg.get("laps"), 0, 99, 0)))]
            if not cfg.get("deliver", True):
                argv.append("--no-deliver")

        # All three modes report; they just have different amounts to say.
        argv += ["--status-udp", f"127.0.0.1:{self.args.status_port}"]
        # A and B expose --dry-run. C is dry by omission and does not accept
        # that flag. Real requests have already been refused above until their
        # mode-specific evidence can be represented by this console.
        if mode in ("A", "B"):
            argv.append("--dry-run")
        return {"argv": argv}

    def start_mission(self, cfg: Dict) -> Dict:
        built = self.build_argv(cfg)
        if "error" in built:
            return {"ok": False, "error": built["error"]}
        if self.args.simulate:
            return {"ok": False,
                    "error": "模擬模式不會真的啟動任務程序，狀態由模擬器產生。"}
        return self.mission.start(built["argv"])


# ════════════════════════════════════════════════════════════════════════════
# Simulator
# ════════════════════════════════════════════════════════════════════════════

SIM_SCRIPT = [
    ("BOOT", "RUN_SELF_CHECK", "boot", {}),
    ("SELF_CHECK", "RUN_SELF_CHECK", "waiting for map pose", {}),
    ("IDLE", "WAIT_OPERATOR", "waiting for the operator to start",
     {"waiting": "start"}),
    ("PATROL", "DRIVE_PATROL", "patrolling", {"goal": 12}),
    ("PATROL", "DRIVE_PATROL", "detection streak 1/3", {"goal": 41}),
    ("INVESTIGATE", "TURN_TO_TARGET", "streak 3/3 · bearing +21 deg",
     {"goal": 58, "tgt": 3.10}),
    ("APPROACH", "DRIVE_TARGET", "camera dist 2.10 m", {"goal": 58, "tgt": 2.10}),
    ("APPROACH", "DRIVE_TARGET", "camera dist 0.94 m", {"goal": 58, "tgt": 0.94}),
    ("ALIGN", "FINE_ALIGN", "handoff at 0.72 m", {"tgt": 0.72}),
    ("STATIONARY_GATE", "SETTLE", "|v| 0.014 m/s, need < 0.010", {"tgt": 0.31}),
    ("LATCH", "LATCH", "obj frozen in base frame", {"tgt": 0.26, "latched": True}),
    ("GRASP", "RUN_GRASP", "stage 0 · xy 0.021 m", {"tgt": 0.26, "latched": True}),
    ("GRASP", "RUN_GRASP", "stage 1 · gates passed", {"tgt": 0.26, "latched": True}),
    ("GRASP", "RUN_GRASP", "stage 2 · contact @ 148.2 deg",
     {"tgt": 0.26, "latched": True, "finished": True}),
    ("VERIFY", "VERIFY", "object lifted", {"verified": True}),
    ("CARRY_HOME", "STOP", "arm at home · object held", {"verified": True}),
    ("DELIVER", "DRIVE_BIN", "bin dist 1.84 m", {"verified": True}),
    ("PLACE_ALIGN", "SETTLE", "bin dist 0.28 m", {"verified": True}),
    ("PLACE", "PLACE", "release sequence", {"verified": True}),
    ("RESUME", "RESUME_PATROL", "map goal re-armed", {}),
    ("PATROL", "DRIVE_PATROL", "patrolling", {"goal": 96}),
]
SIM_PERIOD_S = 2.1
DRIVING = {"PATROL", "INVESTIGATE", "APPROACH", "ALIGN", "RETRY", "DELIVER", "PLACE_ALIGN"}
ARM = {"GRASP", "PLACE", "CARRY_HOME"}


def watch_memory(console: "Console", period: float = 10.0) -> None:
    """Push the headroom figure occasionally, and only when someone is looking.

    Two file reads every ten seconds is nothing, but doing it with no browser
    attached would still be work done for no reader.
    """
    while True:
        time.sleep(period)
        if console.hub.count:
            console.hub.publish("system", {"memory": memory(), "t": time.time()})


def simulate(console: "Console") -> None:
    """Drive the console from a scripted mission, so the UI can be built,
    demonstrated and tested on a machine with no robot attached."""
    # Walk the real route, so --simulate exercises the position display against
    # the same waypoints the robot would drive. A made-up pose would sit off the
    # map and prove nothing about the transform.
    route = console.route()
    path = [(w["x"], w["y"]) for w in route["waypoints"]] if route.get("ok") else []

    i = 0
    while True:
        # A stopped mission stops reporting. On real hardware that happens
        # because the process is gone; here it has to be honoured explicitly,
        # or pressing stop during a demo leaves the console cheerfully
        # narrating a robot that was just halted.
        if console.mission.estop_latched:
            time.sleep(SIM_PERIOD_S)
            continue
        state, action, reason, extra = SIM_SCRIPT[i % len(SIM_SCRIPT)]
        prev = console.status.get("state")
        frame = {
            "v": mission_status.SCHEMA_VERSION,
            "seq": i + 1, "t": time.time(),
            "state": state, "action": action, "reason": reason,
            "changed": state != prev,
            "chassis_allowed": state in DRIVING,
            "arm_allowed": state in ARM,
            "terminal": False,
            "waiting": extra.get("waiting"),
            "laps": 0,
            "lifted": sum(1 for k in range(i + 1)
                          if SIM_SCRIPT[k % len(SIM_SCRIPT)][0] == "CARRY_HOME"
                          and SIM_SCRIPT[(k - 1) % len(SIM_SCRIPT)][0] != "CARRY_HOME"),
            "target": {"visible": "tgt" in extra, "dist": extra.get("tgt"),
                       "streak": 3 if "tgt" in extra else 0, "age": 0.08},
            "grasp": {"latched": extra.get("latched", False),
                      "finished": extra.get("finished", False),
                      "verified": extra.get("verified", False),
                      "handoff_ready": state in ("LATCH", "GRASP"),
                      "align_failed": False},
            "base": {"stationary": state not in DRIVING, "arm_at_home": state not in ARM},
            "blocking": None,
            "goal": {"id": f"wp_{extra.get('goal', 0):03d}",
                     "dist": extra.get("tgt"), "bearing": 0.0},
            "pose": None,
            "simulated": True,
        }
        if path:
            here = path[(i * 2) % len(path)]
            ahead = path[(i * 2 + 1) % len(path)]
            frame["pose"] = {
                "x": here[0], "y": here[1],
                "yaw": math.atan2(ahead[1] - here[1], ahead[0] - here[0]),
            }
        console.status = frame
        console.status_at = time.time()
        console.hub.publish("status", frame)
        i += 1
        time.sleep(SIM_PERIOD_S)


# ════════════════════════════════════════════════════════════════════════════
# HTTP
# ════════════════════════════════════════════════════════════════════════════

class Handler(BaseHTTPRequestHandler):
    server_version = "X3PlusConsole/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *a):           # quieter than the default
        if self.server.verbose:               # type: ignore[attr-defined]
            sys.stderr.write("[http] %s\n" % (fmt % a))

    # ── helpers ──
    def _send(self, code: int, body: bytes, ctype: str, extra: Dict = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # The console is served from the robot on a local network, but a stale
        # cached build is a confusing way to debug a live robot.
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _body(self) -> Dict:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return {}
        if n <= 0 or n > MAX_BODY_BYTES:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8")) or {}
        except (ValueError, UnicodeDecodeError):
            return {}

    # ── routes ──
    def do_GET(self) -> None:
        path = urlparse(self.path).path
        c = self.server.console                # type: ignore[attr-defined]

        if path == "/api/state":
            return self._json(c.snapshot())
        if path == "/api/events":
            return self._sse()
        if path in ("/", "/index.html"):
            return self._static("index.html")
        if path.startswith("/static/"):
            return self._static(path[len("/static/"):])
        if path == "/healthz":
            return self._json({"ok": True, "clients": c.hub.count})
        return self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        c = self.server.console                # type: ignore[attr-defined]
        body = self._body()

        if path == "/api/route":
            return self._json(c.route(str(body.get("route", "")).strip()))
        if path == "/api/mission/preview":
            built = c.build_argv(body)
            if "error" in built:
                return self._json({"ok": False, "error": built["error"]})
            return self._json({"ok": True, "argv": built["argv"],
                               "command": " ".join(shlex.quote(a) for a in built["argv"])})
        if path == "/api/mission/start":
            return self._json(c.start_mission(body))
        if path == "/api/mission/confirm":
            return self._json(c.mission.confirm())
        if path == "/api/mission/stop":
            return self._json(c.mission.stop(estop=False))
        if path == "/api/estop":
            return self._json(c.mission.stop(estop=True))
        if path == "/api/estop/clear":
            return self._json(c.mission.clear_estop())
        return self._json({"error": "not found"}, 404)

    # ── static ──
    def _static(self, rel: str) -> None:
        # resolve() then containment check: without it, a request for
        # ../../etc/passwd would be served happily.
        target = (STATIC / rel).resolve()
        try:
            target.relative_to(STATIC.resolve())
        except ValueError:
            return self._json({"error": "forbidden"}, 403)
        if not target.is_file():
            return self._json({"error": "not found"}, 404)
        ctype, _ = mimetypes.guess_type(target.name)
        if ctype and ctype.startswith(("text/", "application/javascript")):
            ctype += "; charset=utf-8"
        self._send(200, target.read_bytes(), ctype or "application/octet-stream")

    # ── server-sent events ──
    def _sse(self) -> None:
        c = self.server.console                # type: ignore[attr-defined]
        q = c.hub.subscribe()
        if q is None:
            return self._json({"error": "太多裝置同時連著這個操作台，請關掉其中一個分頁。"},
                              503)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        # Nginx and friends buffer streams by default, which would hold every
        # frame until the connection closed.
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            self._emit("snapshot", c.snapshot())
            while True:
                try:
                    event, data = q.get(timeout=SSE_KEEPALIVE_S)
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                self._emit(event, data)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            c.hub.unsubscribe(q)

    def _emit(self, event: str, data: Dict) -> None:
        payload = json.dumps(data, ensure_ascii=False)
        self.wfile.write(("event: %s\ndata: %s\n\n" % (event, payload)).encode("utf-8"))
        self.wfile.flush()


# ════════════════════════════════════════════════════════════════════════════
# Wiring
# ════════════════════════════════════════════════════════════════════════════

def lan_address() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def build_server(console: Console, port: int, bind: str, verbose: bool):
    httpd = ThreadingHTTPServer((bind, port), Handler)
    httpd.daemon_threads = True
    httpd.console = console                    # type: ignore[attr-defined]
    httpd.verbose = verbose                    # type: ignore[attr-defined]
    return httpd


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="0.0.0.0 so a phone on the same network can reach it")
    ap.add_argument("--status-port", type=int, default=DEFAULT_STATUS_PORT,
                    help="UDP port the mission publishes telemetry to")
    ap.add_argument("--allow-real", action="store_true",
                    help="enable the console's first real-mode gate; current A/B/C "
                         "real requests still fail closed until their evidence "
                         "fields are implemented")
    ap.add_argument("--simulate", action="store_true",
                    help="scripted mission, no robot; for developing the UI")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)

    if args.selftest:
        from test_server import run_selftest      # noqa: WPS433
        run_selftest()
        return 0

    if args.allow_real and args.simulate:
        ap.error("--allow-real and --simulate contradict each other")
    if not STATIC.is_dir():
        ap.error(f"missing {STATIC} — the console's front end is not installed")

    console = Console(args)
    threading.Thread(target=console.serve_status, args=(args.status_port,),
                     daemon=True).start()
    threading.Thread(target=watch_memory, args=(console,), daemon=True).start()
    if args.simulate:
        threading.Thread(target=simulate, args=(console,), daemon=True).start()

    httpd = build_server(console, args.port, args.bind, args.verbose)
    ip = lan_address()
    print("X3Plus 操作台")
    print("  本機   http://127.0.0.1:%d" % args.port)
    print("  手機   http://%s:%d" % (ip, args.port))
    print("  遙測   UDP %d" % args.status_port)
    print("  硬體   %s" % ("第一道閘已開；A/B/C 仍因缺少模式證據而鎖定"
                           if args.allow_real else "唯讀，不驅動硬體"))
    if args.simulate:
        print("  模式   模擬（沒有連接機器人）")
    print("  QR     python3 ui/make_qr.py --port %d" % args.port)
    print("Ctrl+C 結束。")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n收到中斷訊號，正在關閉…")
        if console.mission.running:
            print("任務程序仍在執行，送出停止訊號…")
            console.mission.stop()
    finally:
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
