"""Loop console (ui/loop_console/server.py): parsing, checks, map and API gates; no robot."""
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import finals_qr  # noqa: E402
import server  # noqa: E402

REPO = HERE.parents[1]
RUN19 = REPO / 'logs' / 'mission_20261021_patrol19' / 'mission.out'


def feed(lines, pose=(0.0, 0.0, 0.0)):
    hub = server.Hub('10.0.0.1')
    hub.state['pose'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2], 'ready': True}
    f = server.Feed(hub)
    for line in lines:
        f.line(line)
    return hub


class FinalsQrTests(unittest.TestCase):
    SS = ('Recv-Q Send-Q Local Address:Port Peer Address:Port\n'
          '0 0 10.0.0.2:22 10.0.0.9:56019\n'
          '0 0 10.0.0.2:22 10.0.0.9:49172\n'
          '0 0 [::ffff:10.0.0.2]:22 [::ffff:10.0.0.7]:50000\n'
          '0 0 [fe80::1]:22 [fe80::2]:50001\n')

    def test_peers_are_ipv4_and_unique(self):
        self.assertEqual(finals_qr.ssh_peers(self.SS), ['10.0.0.9', '10.0.0.7'])

    def test_only_a_host_answering_as_the_console_gets_a_qr(self):
        with patch.object(finals_qr, 'answers', side_effect=lambda ip, port: ip == '10.0.0.7'):
            self.assertEqual(finals_qr.pick(['10.0.0.9', '10.0.0.7'], 8090), '10.0.0.7')
            self.assertIsNone(finals_qr.pick(['10.0.0.9'], 8090))


class ParseTests(unittest.TestCase):
    def test_relocalization_lines(self):
        hub = feed(['[RELOC] captured: markers [2, 7]',
                    '[RELOC] localized at map (0.22, 0.24), heading -46 deg, from markers [2, 7] (57 s)'])
        self.assertEqual(hub.state['stage'], 'localized')
        self.assertAlmostEqual(hub.state['relocalized']['yaw'], math.radians(-46))
        hub = feed(['[RELOC] captured: markers [1, 6]',
                    '[RELOC] matching the walls on the robot (~40 s)'])
        self.assertEqual(hub.state['stage'], 'relocalizing')
        self.assertIn('40', hub.state['detail'])
        hub = feed(['[RELOC] captured: markers [3, 4]',
                    '[RELOC] turned -29 deg to look for a marker on another wall'])
        self.assertIn('轉動', hub.state['detail'])
        self.assertIn('29', hub.log[-1]['text'])
        hub = feed(['[BLOCKED] relocalization: AMBIGUOUS: markers [2] fit more than one place'])
        self.assertIn('AMBIGUOUS', hub.state['detail'])
        self.assertEqual(hub.log[-1]['kind'], 'crit')

    def test_a_sighting_goes_on_the_map_from_the_pose_and_its_state_follows(self):
        hub = feed(['[PATROL] candidate 1.00 m ahead, +0.50 m left (beyond the calibrated area, '
                    'rough): parked to confirm'], pose=(1.0, 1.0, math.pi / 2))
        (s,) = hub.state['sightings']
        self.assertEqual((s['state'], s['rough']), ('candidate', True))
        self.assertAlmostEqual(s['x'], 0.5)                 # facing +y: left is -x
        self.assertAlmostEqual(s['y'], 2.0)
        f = server.Feed(hub)
        f.line('[PATROL] not confirmed with the robot parked')
        self.assertEqual(hub.state['sightings'], [])
        f.line('[PATROL] sugarbox 0.83 m ahead, -0.06 m left: confirmed')
        self.assertEqual([d['state'] for d in hub.state['sightings']], ['confirmed'])
        f.line('{"ok": true, "confirmed": true, "outcome": "confirmed", "elapsed_s": 7.5}')
        self.assertEqual((hub.state['stage'], hub.state['sightings']), ('grasped', []))

    def test_a_replay_ends_with_the_recorded_missions_exit_code(self):
        # Run 19 stopped with exit 2; its replay must not report a clean exit 0.
        import subprocess
        import tempfile
        for text, code in (('[MISSION] stopped in PATROL 2: patrol stopped\n', 2),
                           ('[MISSION] 1 delivered: done\n', 0)):
            with tempfile.TemporaryDirectory() as folder:
                log = Path(folder) / 'mission.out'
                log.write_text(text, encoding='utf-8')
                done = subprocess.run(server.replay_argv(log, sys.executable), capture_output=True)
            self.assertEqual(done.returncode, code)

    def test_a_localization_outage_is_told_in_the_log(self):
        hub = feed(['[LOCALIZE] not localized: waiting for fresh scan: waiting up to 20 s '
                    'with the robot stopped', '[LOCALIZE] back after 2.4 s'])
        texts = [e['text'] for e in hub.log]
        self.assertIn('定位暫時中斷（not localized: waiting for fresh scan），停車等待最多 20 秒',
                      texts)
        self.assertIn('定位恢復（等了 2.4 秒），繼續', texts)

    def test_a_miss_is_marked_and_a_second_at_the_spot_is_skipped(self):
        hub = feed(['[MISSION] grasp 1 missed with the robot at the box at map (0.40, 1.09): '
                    'backing out 0.20 m to see it again',
                    '[MISSION] grasp 2 missed at map (0.42, 1.10): backing out, patrolling on'])
        self.assertEqual([d['state'] for d in hub.state['sightings']], ['failed', 'skipped'])
        self.assertEqual(hub.state['failures'], 2)

    def test_a_recorded_run_reads_end_to_end(self):
        if not RUN19.is_file():
            self.skipTest('no run 19 log here')
        hub = feed(RUN19.read_text(encoding='utf-8', errors='replace').splitlines(),
                   pose=(1.0, 0.5, 0.0))
        self.assertEqual(hub.state['delivered'], 1)
        self.assertEqual(hub.state['stage'], 'stopped')     # cannot enter turn reserve
        self.assertIn('turn reserve', hub.state['detail'])
        texts = [e['text'] for e in hub.log]
        self.assertTrue(any('夾取成功' in t for t in texts))
        self.assertTrue(any('已放進放置盒' in t for t in texts))
        self.assertFalse(any(t.startswith('[PURSUIT]') or t.startswith('[RUN]') for t in texts))


def report(**over):
    r = {'services': {s: 'active' for s in server.SERVICES}, 'arm_camera': True,
         'rear_camera': True, 'rear_busy': False, 'arm_frame_age': 0.3, 'usb_disconnects': 0,
         'localization': {'ready': True, 'age': 0.1},
         'status': {'chassis': {'arm_pose': 'travel', 'maintenance': True}},
         'health': {'holding': False, 'chassis_tcp_ready': True},
         'mem_free_mb': 1500, 'patrol_running': False}
    r.update(over)
    return r


def levels(rows):
    return {r['key']: (r['level'], r['fix']) for r in rows}


class CheckTests(unittest.TestCase):
    def test_all_well(self):
        rows = server.assess(report())
        self.assertTrue(all(r['level'] == 'ok' for r in rows), rows)

    def test_the_arm_camera_troubles_offer_a_restart(self):
        self.assertEqual(levels(server.assess(report(arm_camera=False)))['arm_camera'],
                         ('crit', 'arm_camera'))
        self.assertEqual(levels(server.assess(report(arm_frame_age=40.0)))['arm_camera'],
                         ('crit', 'arm_camera'))
        self.assertEqual(levels(server.assess(report(usb_disconnects=3)))['usb'], ('warn', None))

    def test_an_arm_left_out_at_boot_offers_the_stow(self):
        rows = levels(server.assess(report(
            status={'chassis': {'arm_pose': 'startup_failed'}},
            health={'holding': False, 'chassis_tcp_ready': False})))
        self.assertEqual(rows['arm'], ('warn', 'arm_stow'))
        self.assertEqual(rows['chassis'], ('crit', 'arm_stow'))

    def test_localization_and_services(self):
        rows = levels(server.assess(report(localization={'ready': False, 'reason': 'seed'},
                                           services={'grasp-vision': 'failed'})))
        self.assertEqual(rows['localization'][0], 'warn')
        self.assertEqual(rows['services'][0], 'crit')


class Mask:
    def allowed(self, x, y):
        return 0.5 <= x <= 1.0 and 0.5 <= y <= 1.0


class MapTests(unittest.TestCase):
    def test_payload(self):
        layout = {'frame_id': 'map', 'room_polygon': [[0, 0], [2, 0], [2, 2], [0, 2]],
                  'bin': {'pose_map': {'x': 1.8, 'y': 1.8},
                          'pose_map_valid_for_current_placement': True}}
        landmarks = {'markers': {'2': {'point_map': [1.0, 0.0]}, '7': {'point_map': [2.0, 1.0]}}}
        m = server.map_payload(layout, Mask(), landmarks, cell=0.5)
        self.assertEqual(m['bounds'], [0.0, 0.0, 2.0, 2.0])
        self.assertTrue(m['box']['verified'])
        self.assertEqual([k['id'] for k in m['markers']], ['2', '7'])
        self.assertEqual(m['mask']['rows'], ['0000', '0100', '0000', '0000'])
        with self.assertRaises(ValueError):
            server.map_payload({'frame_id': 'map'})


class FakeRobot:
    def __init__(self):
        self.stops = 0

    def stop(self):
        self.stops += 1

    def check(self):
        return report()

    def map_files(self):
        raise OSError('offline')

    def pose_stream(self):
        raise OSError('offline')


class FakeProc:
    def __init__(self, argv, **kw):
        self.argv, self.stdout, self.pid, self.kw = argv, iter(()), 4321, kw
        self.done = threading.Event()

    def poll(self):
        return 0 if self.done.is_set() else None

    def wait(self):
        self.done.wait(2)
        return 0

    def send_signal(self, sig):
        self.done.set()


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.procs = []

        def popen(argv, **kw):
            p = FakeProc(argv, **kw)
            self.procs.append(p)
            return p
        self.robot = FakeRobot()
        self.console = server.Console('10.0.0.1', python='py', robot=self.robot, popen=popen)
        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_for(self.console))
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.base = 'http://127.0.0.1:{}'.format(self.httpd.server_address[1])

    def tearDown(self):
        for p in self.procs:
            p.done.set()
        self.console.closed.set()
        self.httpd.shutdown()
        self.httpd.server_close()

    def post(self, path, body=None, raw=None):
        data = raw if raw is not None else json.dumps(body or {}).encode()
        req = Request(self.base + path, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except HTTPError as e:
            return e.code, json.loads(e.read())

    def test_starting_the_loop_needs_the_operator_at_the_robot(self):
        self.assertEqual(self.post('/api/start', {})[0], 400)
        self.assertEqual(self.procs, [])
        code, _ = self.post('/api/start', {'beside_robot': True, 'relocalize': True,
                                           'max_deliveries': '3'})
        self.assertEqual(code, 200)
        argv = self.procs[0].argv
        self.assertIn('integration/mission_loop.py', argv)
        for flag in ('--real', '--i-am-beside-the-robot', '--relocalize'):
            self.assertIn(flag, argv)
        self.assertEqual(argv[argv.index('--max-deliveries') + 1], '3')
        # One job at a time.
        self.assertEqual(self.post('/api/relocalize', {'beside_robot': True})[0], 409)

    def test_bad_input_is_refused(self):
        self.assertEqual(self.post('/api/start', raw=b'[1, 2]')[0], 400)
        self.assertEqual(self.post('/api/start', {'beside_robot': True,
                                                  'max_deliveries': '-1'})[0], 400)
        self.assertEqual(self.post('/api/fix', {'what': 'arm_stow'})[0], 400)   # moves the arm
        self.assertEqual(self.post('/api/fix', {'what': 'reboot', 'beside_robot': True})[0], 400)

    def test_a_camera_restart_moves_nothing_so_needs_no_confirmation(self):
        self.assertEqual(self.post('/api/fix', {'what': 'arm_camera'})[0], 200)
        self.assertIn('restart_arm_camera', self.procs[0].argv[-1])

    def test_relocalize_runs_the_script_and_stop_reaches_the_robot(self):
        # It may turn in place to find a second wall's markers: operator first.
        self.assertEqual(self.post('/api/relocalize')[0], 400)
        self.assertEqual(self.procs, [])
        self.assertEqual(self.post('/api/relocalize', {'beside_robot': True})[0], 200)
        self.assertIn('integration/relocalize.py', self.procs[0].argv)
        for flag in ('--look', '--i-am-beside-the-robot'):
            self.assertIn(flag, self.procs[0].argv)
        # 2026-10-29: run as a script it could not import `integration.` (exit 1).
        paths = self.procs[0].kw['env']['PYTHONPATH'].split(server.os.pathsep)
        self.assertIn(str(server.REPO), paths)
        # A minute of standing still gets a clock and a usual duration.
        state = self.console.hub.state
        self.assertEqual(state['stage_eta_s'], 60.0)
        self.assertIsNotNone(state['stage_since'])
        self.assertTrue(state['detail'])
        with patch.object(server.subprocess, 'run') as run:
            self.assertEqual(self.post('/api/stop')[0], 200)
        self.assertEqual(self.robot.stops, 1)
        self.assertEqual(self.console.hub.state['stage'], 'stopped')
        if server.os.name == 'nt':
            self.assertEqual(run.call_args.args[0][:4], ['taskkill', '/T', '/F', '/PID'])

    def test_state_and_pages(self):
        with urlopen(self.base + '/api/state', timeout=5) as r:
            state = json.loads(r.read())
        self.assertEqual(state['host'], '10.0.0.1')
        for page in ('/', '/static/app.js', '/static/loop.css', '/static/base.css'):
            with urlopen(self.base + page, timeout=5) as r:
                self.assertEqual(r.status, 200)
        with self.assertRaises(HTTPError):
            urlopen(self.base + '/static/../server.py', timeout=5)


if __name__ == '__main__':
    unittest.main()
