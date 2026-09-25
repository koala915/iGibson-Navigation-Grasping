#!/usr/bin/env python3
"""Tests for the operator console.

The console can start a process that moves a robot, so the gates that stop it
doing that are the part worth testing hardest.  Everything here runs offline
and touches no hardware.

    python3 ui/test_server.py
"""

from __future__ import annotations

import json
import os
import re
import socket
import sys
import threading
import time
import unittest
from unittest import mock
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "integration"))

import server as srv                                             # noqa: E402
import mission_status                                            # noqa: E402
import make_qr                                                   # noqa: E402


class Args:
    """Stand-in for the parsed command line."""

    def __init__(self, **kw):
        self.allow_real = kw.get("allow_real", False)
        self.simulate = kw.get("simulate", False)
        self.status_port = kw.get("status_port", 8099)
        self.port = kw.get("port", 8080)
        self.bind = kw.get("bind", "127.0.0.1")
        self.verbose = False


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# ════════════════════════════════════════════════════════════════════════════

class TestSafetyGates(unittest.TestCase):
    """Nothing may drive without both gates. These are the tests that matter."""

    def test_real_refused_without_allow_real(self):
        c = srv.Console(Args(allow_real=False))
        r = c.build_argv({"mode": "A", "real": True, "unlock": True})
        self.assertIn("error", r)
        self.assertIn("--allow-real", r["error"])

    def test_real_refused_without_operator_unlock(self):
        c = srv.Console(Args(allow_real=True))
        r = c.build_argv({"mode": "A", "real": True, "unlock": False})
        self.assertIn("error", r)

    def test_real_still_refused_when_mode_specific_evidence_is_unavailable(self):
        c = srv.Console(Args(allow_real=True))
        for mode in "ABC":
            with self.subTest(mode=mode):
                r = c.build_argv({"mode": mode, "real": True, "unlock": True})
                self.assertIn("error", r)
                self.assertNotIn("argv", r)

    def test_simulate_never_drives(self):
        c = srv.Console(Args(allow_real=True, simulate=True))
        r = c.build_argv({"mode": "A", "real": True, "unlock": True})
        self.assertIn("error", r)

    def test_dry_run_is_the_default(self):
        c = srv.Console(Args())
        r = c.build_argv({"mode": "A"})
        self.assertIn("--dry-run", r["argv"])
        self.assertNotIn("--real", r["argv"])

    def test_estop_latch_blocks_a_new_start(self):
        c = srv.Console(Args())
        c.mission.estop_latched = True
        out = c.mission.start([sys.executable, "-c", "pass"])
        self.assertFalse(out["ok"])
        c.mission.clear_estop()
        self.assertFalse(c.mission.estop_latched)


class TestArgvBuilding(unittest.TestCase):
    """The console writes a command line from data that arrived over a network."""

    def setUp(self):
        self.c = srv.Console(Args())

    def test_unknown_mode_refused(self):
        self.assertIn("error", self.c.build_argv({"mode": "Z"}))

    def test_shell_metacharacters_refused(self):
        for evil in ("sugarbox; rm -rf /", "a b", "$(id)", "x`id`", "a|b", "a&b",
                     "--real", "" ):
            with self.subTest(evil=evil):
                self.assertIn("error", self.c.build_argv({"mode": "A", "cls": evil}))

    def test_a_trailing_newline_does_not_sneak_through(self):
        """'$' matches before a trailing newline in Python; '\\Z' does not."""
        for evil in ("sugarbox\n", "route.yaml\n"):
            with self.subTest(evil=evil):
                self.assertIn("error", self.c.build_argv({"mode": "A", "cls": evil}))

    def test_route_is_validated_too(self):
        self.assertIn("error", self.c.build_argv({"mode": "A", "route": "r.yaml; reboot"}))
        self.assertNotIn("error", self.c.build_argv({"mode": "A", "route": "route.yaml"}))
        self.assertNotIn("error",
                         self.c.build_argv({"mode": "A", "route": "config/routes/route.yaml"}))

    def test_route_cannot_escape_the_project(self):
        for evil in ("../../etc/passwd", "a/../../b.yaml", "/etc/passwd"):
            with self.subTest(evil=evil):
                self.assertIn("error", self.c.build_argv({"mode": "A", "route": evil}))

    def test_height_is_clamped_not_trusted(self):
        for bad, expect in ((None, "0.065"), ("abc", "0.065"), (-5, "0.005"),
                            (10 ** 9, "0.300"), (float("nan"), "0.065")):
            with self.subTest(bad=bad):
                argv = self.c.build_argv({"mode": "A", "height_cm": bad})["argv"]
                self.assertIn(f"sugarbox={expect}", argv)

    def test_laps_cannot_go_negative(self):
        argv = self.c.build_argv({"mode": "A", "laps": -7})["argv"]
        self.assertEqual(argv[argv.index("--max-laps") + 1], "0")

    def test_status_endpoint_points_at_this_server(self):
        c = srv.Console(Args(status_port=9123))
        argv = c.build_argv({"mode": "A"})["argv"]
        self.assertIn("127.0.0.1:9123", argv)

    def test_deliver_off_adds_the_flag(self):
        self.assertIn("--no-deliver",
                      self.c.build_argv({"mode": "A", "deliver": False})["argv"])
        self.assertNotIn("--no-deliver",
                         self.c.build_argv({"mode": "A", "deliver": True})["argv"])

    def test_each_dry_mode_is_accepted_by_its_real_parser(self):
        import mission_pipeline
        import vision_grasp_bridge
        import nav_rl_grasp_pipeline
        parsers = {
            "A": mission_pipeline.parse_args,
            "B": vision_grasp_bridge.parse_args,
            "C": nav_rl_grasp_pipeline.parse_args,
        }
        for mode, parser in parsers.items():
            argv = self.c.build_argv({"mode": mode})["argv"]
            with self.subTest(mode=mode), mock.patch.object(sys, "argv", argv[1:]):
                parsed = parser()
                if mode in ("A", "B"):
                    self.assertTrue(parsed.dry_run)
                else:
                    self.assertFalse(parsed.real)


class TestHub(unittest.TestCase):
    def test_subscriber_receives_published_events(self):
        h = srv.Hub()
        q = h.subscribe()
        h.publish("status", {"state": "PATROL"})
        self.assertEqual(q.get(timeout=1), ("status", {"state": "PATROL"}))
        h.unsubscribe(q)
        self.assertEqual(h.count, 0)

    def test_a_stalled_client_does_not_block_the_others(self):
        """A phone that slept stops draining; it must not hold up the console."""
        h = srv.Hub(maxsize=4)
        slow, fast = h.subscribe(), h.subscribe()
        for i in range(50):
            h.publish("status", {"seq": i})
        # The slow queue dropped its oldest frames instead of growing forever...
        self.assertLessEqual(slow.qsize(), 4)
        # ...and what it does hold is the newest, so it is behind, not stale.
        newest = None
        while not slow.empty():
            newest = slow.get_nowait()
        self.assertEqual(newest[1]["seq"], 49)
        self.assertLessEqual(fast.qsize(), 4)


class TestStatusIngest(unittest.TestCase):
    def test_a_real_emitter_frame_lands_in_the_console(self):
        """End to end over the loopback, using the mission's own emitter."""
        import dataclasses

        @dataclasses.dataclass
        class Tr:
            state: str = "GRASP"
            action: str = "RUN_GRASP"
            reason: str = "stage 1"
            changed: bool = True
            chassis_allowed: bool = False
            arm_allowed: bool = True
            terminal: bool = False

        port = free_port()
        c = srv.Console(Args(status_port=port))
        t = threading.Thread(target=c.serve_status, args=(port,), daemon=True)
        t.start()
        time.sleep(0.2)

        emitter = mission_status.StatusEmitter(f"127.0.0.1:{port}")
        emitter.publish(Tr())
        deadline = time.time() + 3.0
        while time.time() < deadline and not c.status:
            time.sleep(0.05)
        emitter.close()
        self.assertEqual(c.status.get("state"), "GRASP")
        self.assertEqual(c.status.get("action"), "RUN_GRASP")

    def test_garbage_datagrams_are_ignored(self):
        port = free_port()
        c = srv.Console(Args(status_port=port))
        threading.Thread(target=c.serve_status, args=(port,), daemon=True).start()
        time.sleep(0.2)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for junk in (b"not json", b"\xff\xfe\x00", b"[1,2,3]", b"null"):
            s.sendto(junk, ("127.0.0.1", port))
        time.sleep(0.3)
        s.close()
        # A malformed frame must not become the console's idea of the world.
        self.assertEqual(c.status, {})


class TestHTTP(unittest.TestCase):
    """The real server, over a real socket."""

    @classmethod
    def setUpClass(cls):
        cls.port = free_port()
        cls.console = srv.Console(Args(port=cls.port, allow_real=False))
        cls.httpd = srv.build_server(cls.console, cls.port, "127.0.0.1", False)
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        time.sleep(0.2)
        cls.base = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def get(self, path):
        with urlopen(self.base + path, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))

    def post(self, path, obj):
        req = Request(self.base + path, data=json.dumps(obj).encode("utf-8"),
                      headers={"Content-Type": "application/json"}, method="POST")
        with urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))

    def test_healthz(self):
        code, body = self.get("/healthz")
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])

    def test_state_reports_the_hardware_gate(self):
        _, body = self.get("/api/state")
        self.assertIn("process", body)
        self.assertFalse(body["allow_real"])

    def test_index_is_served(self):
        with urlopen(self.base + "/", timeout=5) as r:
            html = r.read().decode("utf-8")
        self.assertIn("<title>", html)
        self.assertIn("/static/app.js", html)

    def test_static_traversal_is_refused(self):
        """A path that climbs out of static/ must not be served."""
        for attack in ("/static/../server.py", "/static/../../etc/passwd",
                       "/static/....//server.py"):
            with self.subTest(attack=attack):
                try:
                    with urlopen(self.base + attack, timeout=5) as r:
                        body = r.read().decode("utf-8", "replace")
                    self.assertNotIn("allow_real", body)
                    self.assertNotIn("root:", body)
                except HTTPError as e:
                    self.assertIn(e.code, (403, 404))

    def test_preview_refuses_a_real_run_on_a_read_only_server(self):
        _, body = self.post("/api/mission/preview",
                            {"mode": "A", "real": True, "unlock": True})
        self.assertFalse(body["ok"])

    def test_preview_returns_a_quoted_command(self):
        _, body = self.post("/api/mission/preview", {"mode": "A", "height_cm": 6.5})
        self.assertTrue(body["ok"])
        self.assertIn("--dry-run", body["command"])
        self.assertIn("sugarbox=0.065", body["command"])

    def test_stop_is_safe_with_nothing_running(self):
        _, body = self.post("/api/mission/stop", {})
        self.assertTrue(body["ok"])

    def test_confirm_without_a_mission_says_so(self):
        _, body = self.post("/api/mission/confirm", {})
        self.assertFalse(body["ok"])

    def test_unknown_route_is_404(self):
        try:
            self.get("/api/nope")
            self.fail("expected 404")
        except HTTPError as e:
            self.assertEqual(e.code, 404)

    def test_sse_delivers_a_published_frame(self):
        import http.client
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        conn.request("GET", "/api/events")
        r = conn.getresponse()
        self.assertEqual(r.status, 200)
        self.assertIn("text/event-stream", r.getheader("Content-Type"))
        # The stream opens with a snapshot so a browser joining mid-mission is
        # not blank until the next tick.
        head = r.fp.readline().decode()
        self.assertEqual(head.strip(), "event: snapshot")
        r.fp.readline()
        r.fp.readline()
        time.sleep(0.1)
        self.console.hub.publish("status", {"state": "ALIGN", "seq": 7})
        self.assertEqual(r.fp.readline().decode().strip(), "event: status")
        data = r.fp.readline().decode()
        self.assertEqual(json.loads(data[len("data: "):])["state"], "ALIGN")
        conn.close()


class TestMissionProcess(unittest.TestCase):
    """Supervision, exercised against a stand-in child rather than the robot."""

    def test_output_is_captured_and_exit_recorded(self):
        c = srv.Console(Args())
        code = "import sys; print('hello from the mission'); sys.exit(3)"
        out = c.mission.start([sys.executable, "-c", code])
        self.assertTrue(out["ok"], out)
        deadline = time.time() + 10
        while time.time() < deadline and c.mission.exit_code is None:
            time.sleep(0.05)
        self.assertEqual(c.mission.exit_code, 3)
        self.assertIn("hello from the mission", "\n".join(c.mission.log))

    def test_only_one_mission_at_a_time(self):
        c = srv.Console(Args())
        code = "import time; time.sleep(5)"
        self.assertTrue(c.mission.start([sys.executable, "-c", code])["ok"])
        second = c.mission.start([sys.executable, "-c", code])
        self.assertFalse(second["ok"])
        self.assertIn("已在執行", second["error"])
        c.mission.stop()

    def test_confirm_reaches_the_child_stdin(self):
        """The mission blocks on input() before driving; a button answers it."""
        c = srv.Console(Args())
        code = "input(); print('operator confirmed')"
        self.assertTrue(c.mission.start([sys.executable, "-c", code])["ok"])
        time.sleep(0.4)
        self.assertTrue(c.mission.confirm()["ok"])
        deadline = time.time() + 10
        while time.time() < deadline and c.mission.exit_code is None:
            time.sleep(0.05)
        self.assertEqual(c.mission.exit_code, 0)
        self.assertIn("operator confirmed", "\n".join(c.mission.log))


class TestResourceGuards(unittest.TestCase):
    """The Jetson runs this mission near 90% memory. The console must not be
    the thing that pushes it over."""

    def test_the_console_never_opens_a_camera(self):
        source = (HERE / "server.py").read_text(encoding="utf-8")
        for forbidden in ("VideoCapture", "cv2", "imencode", "mjpg", "numpy"):
            self.assertNotIn(forbidden, source,
                             f"the console must not reference {forbidden}")

    def test_the_pipeline_no_longer_encodes_preview_frames(self):
        """JPEG encoding inside the detection loop was the largest cost added."""
        src = (HERE.parent / "integration" / "vision_grasp_pipeline.py").read_text(encoding="utf-8")
        for forbidden in ("camera_publish", "_pub_arm", "_preview", "imencode"):
            self.assertNotIn(forbidden, src)
        self.assertFalse((HERE.parent / "integration" / "camera_publish.py").exists())

    def test_sse_clients_are_capped(self):
        """A reconnect loop must not accumulate threads on a memory-bound box."""
        h = srv.Hub(max_subs=2)
        self.assertIsNotNone(h.subscribe())
        self.assertIsNotNone(h.subscribe())
        self.assertIsNone(h.subscribe(), "accepted more clients than the cap")

    def test_memory_reports_or_says_nothing(self):
        """Never invent a figure: off Linux the fields are simply absent."""
        m = srv.memory()
        for key in ("total_mb", "available_mb", "used_pct", "console_mb"):
            self.assertIn(key, m)
        if m["used_pct"] is not None:
            self.assertGreaterEqual(m["used_pct"], 0)
            self.assertLessEqual(m["used_pct"], 100)
            self.assertGreater(m["total_mb"], 0)
        json.dumps(m)


class TestLiteFrontend(unittest.TestCase):
    def setUp(self):
        self.html = (HERE / "static" / "index.html").read_text(encoding="utf-8")
        self.js = (HERE / "static" / "app.js").read_text(encoding="utf-8")

    def test_formal_ui_keeps_only_the_requested_surfaces(self):
        for removed in ("recentField", 'data-mode="B"', 'data-mode="C"',
                        "cmdOut", "copyBtn", 'data-go="log"', "s-log",
                        "fullLog", "exportBtn", "procLog"):
            self.assertNotIn(removed, self.html)
        self.assertNotIn('source.addEventListener("log"', self.js)
        self.assertNotIn("appendProcLog", self.js)
        self.assertIn('mode: "A"', self.js)

    def test_qr_enters_the_console_without_an_ip_prompt(self):
        url = make_qr.console_url("192.168.1.42", 8080)
        self.assertEqual(url, "http://192.168.1.42:8080/?connect=1")
        self.assertIn('get("connect") === "1"', self.js)


class TestTelemetryCost(unittest.TestCase):
    def test_unchanged_ticks_are_collapsed(self):
        import dataclasses

        @dataclasses.dataclass
        class Tr:
            state: str = "PATROL"
            action: str = "DRIVE_PATROL"
            reason: str = "patrolling"
            changed: bool = False
            chassis_allowed: bool = True
            arm_allowed: bool = False
            terminal: bool = False

        e = mission_status.StatusEmitter("127.0.0.1:9", period=10.0)
        sent = sum(1 for _ in range(100) if e.publish(Tr()) is not None)
        self.assertEqual(sent, 1, "100 identical ticks should publish once")
        self.assertEqual(e.skipped, 99)
        # A change is never withheld: it is the whole point of the stream.
        self.assertIsNotNone(e.publish(Tr(state="GRASP", changed=True)))
        e.close()

    def test_the_mission_defaults_to_the_throttled_period(self):
        src = (HERE.parent / "integration" / "mission_pipeline.py").read_text(encoding="utf-8")
        self.assertIn("--status-period", src)
        self.assertGreater(mission_status.DEFAULT_PERIOD_S, 0.0)


class TestRoute(unittest.TestCase):
    def setUp(self):
        self.c = srv.Console(Args())

    def test_a_traversal_path_is_refused(self):
        for evil in ("../../etc/passwd", "/etc/passwd", "a/../../b.yaml"):
            with self.subTest(evil=evil):
                r = self.c.route(evil)
                self.assertFalse(r["ok"])

    def test_a_missing_route_explains_itself(self):
        r = self.c.route("config/routes/definitely_absent.yaml")
        self.assertFalse(r["ok"])
        self.assertIn("找不到", r["error"])

    def test_a_readable_route_returns_map_frame_metres(self):
        """Skipped where the route package is not checked out beside the repo."""
        r = self.c.route("")
        if not r["ok"]:
            self.skipTest(r["error"])
        self.assertGreater(len(r["waypoints"]), 1)
        for w in r["waypoints"][:5]:
            self.assertIsInstance(w["x"], float)
            self.assertIsInstance(w["y"], float)
        json.dumps(r)


class TestSimpleReporter(unittest.TestCase):
    """Modes B and C must speak the same vocabulary as mode A."""

    def test_modes_b_and_c_are_told_where_to_report(self):
        c = srv.Console(Args(status_port=9321))
        for mode in ("A", "B", "C"):
            with self.subTest(mode=mode):
                argv = c.build_argv({"mode": mode})["argv"]
                self.assertIn("--status-udp", argv)
                self.assertIn("127.0.0.1:9321", argv)

    def test_every_state_the_pipelines_report_has_ui_wording(self):
        """A state the front end cannot describe would render as a raw token."""
        app = (HERE / "static" / "app.js").read_text(encoding="utf-8")
        described = set(re.findall(r"^  ([A-Z_]{3,}):", app, re.M))
        for path in ("nav_rl_grasp_pipeline.py", "vision_grasp_bridge.py"):
            src = (HERE.parent / "integration" / path).read_text(encoding="utf-8")
            for state in re.findall(r'report\.say\(\s*"([A-Z_]+)"', src):
                with self.subTest(path=path, state=state):
                    self.assertIn(state, described,
                                  f"{path} reports {state} but app.js has no wording for it")


class TestSimulator(unittest.TestCase):
    def test_the_script_only_uses_real_state_names(self):
        """A simulated state the UI has no wording for would be a silent hole."""
        sys.path.insert(0, str(HERE.parent / "integration"))
        import mission_fsm
        real = {s.value for s in mission_fsm.State}
        actions = {a.value for a in mission_fsm.Action}
        for state, action, _reason, _extra in srv.SIM_SCRIPT:
            self.assertIn(state, real, f"{state} is not a real mission state")
            self.assertIn(action, actions, f"{action} is not a real mission action")

    def test_simulated_frames_match_the_wire_schema(self):
        console = srv.Console(Args(simulate=True))
        t = threading.Thread(target=srv.simulate, args=(console,), daemon=True)
        t.start()
        deadline = time.time() + 5
        while time.time() < deadline and not console.status:
            time.sleep(0.05)
        f = console.status
        self.assertEqual(f["v"], mission_status.SCHEMA_VERSION)
        for key in ("state", "action", "reason", "target", "grasp", "base", "lifted"):
            self.assertIn(key, f)
        self.assertTrue(f["simulated"])
        json.dumps(f, allow_nan=False)


def run_selftest() -> None:
    unittest.main(module=__name__, argv=[sys.argv[0], "-v"], exit=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
