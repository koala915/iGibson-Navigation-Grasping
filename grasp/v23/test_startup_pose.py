"""Boot parking checks using a fake plant; never opens a serial port."""
import unittest
import tempfile
from unittest.mock import patch
import grasp_service as gs
import chassis_server as cs
from test_travel_pose import Ctl, E1, TRAVEL, WAY, arms


class StartupTests(unittest.TestCase):
    def service(self, at, fail_leg=None):
        ctl = Ctl(at, fail_leg=fail_leg)
        return ctl, gs.GraspService(ctl, None, 1, '/tmp/unused.sock')

    def test_disabled_never_moves(self):
        ctl, svc = self.service(E1)
        self.assertTrue(svc.prepare_startup_pose(False)['ok'])
        self.assertEqual(ctl.moves, [])

    def test_travel_never_moves(self):
        ctl, svc = self.service(TRAVEL)
        self.assertTrue(svc.prepare_startup_pose(True)['ok'])
        self.assertEqual(ctl.moves, [])

    def test_e1_uses_checked_waypoint(self):
        ctl, svc = self.service(E1)
        self.assertTrue(svc.prepare_startup_pose(True)['ok'])
        self.assertEqual(arms(ctl), [WAY, TRAVEL[:5]])

    def test_other_pose_homes_before_stow(self):
        ctl, svc = self.service((90, 96, 81, 95, 90, 30))
        self.assertTrue(svc.prepare_startup_pose(True)['ok'])
        self.assertEqual(arms(ctl), [E1[:5], WAY, TRAVEL[:5]])

    def test_closed_jaw_never_moves_or_opens(self):
        ctl, svc = self.service(E1[:5] + (100,))
        self.assertEqual(svc.prepare_startup_pose(True)['reason'], 'jaw_not_open_at_boot')
        self.assertEqual(ctl.moves, [])

    def test_missing_feedback_never_moves(self):
        ctl, svc = self.service(E1)
        svc._arm_deg = lambda: None
        self.assertFalse(svc.prepare_startup_pose(True)['ok'])
        self.assertEqual(ctl.moves, [])

    def test_home_failure_does_not_stow(self):
        ctl, svc = self.service((90, 96, 81, 95, 90, 30), fail_leg=0)
        self.assertEqual(svc.prepare_startup_pose(True)['reason'], 'startup_home_failed')
        self.assertEqual(len(ctl.moves), 1)

    def test_stow_failure_does_not_continue(self):
        ctl, svc = self.service(E1, fail_leg=0)
        self.assertEqual(svc.prepare_startup_pose(True)['reason'], 'startup_stow_failed')
        self.assertEqual(len(ctl.moves), 1)

    def test_failed_final_verification(self):
        ctl, svc = self.service(E1)
        svc.run_arm_command = lambda fn: {'ok': True}
        self.assertEqual(svc.prepare_startup_pose(True)['reason'], 'startup_verification_failed')

    def test_stale_board_blocks_startup_and_marks_pose(self):
        ctl, svc = self.service(E1)
        class Chassis:
            pose = None
            def status(self):
                return dict(moving=False, motors=[0]*4, board_rx_age_s=2)
            def set_arm_pose(self, pose):
                self.pose = pose
        svc.chassis = Chassis()
        self.assertEqual(svc.prepare_startup_pose(True)['reason'], 'board_feedback_stale')
        self.assertEqual(svc.chassis.pose, 'startup_failed')
        self.assertEqual(ctl.moves, [])

    def run_serve(self, jaw):
        class Board:
            calls = []
            def set_motor(self, *values):
                self.calls.append(values)
        class Server:
            port = 7000
            started = False
            chassis = cs.Chassis(Board(), lambda vx, wz: [0]*4,
                                 rx_age=lambda: 0, log=lambda msg: None)
            def start(self):
                self.locked_at_start = self.chassis.status()['maintenance']
                if self.locked_at_start:
                    self.velocity_result = self.chassis.command(
                        {'action':'velocity','vx':0.1,'wz':0})
                self.started = True
            def close(self):
                self.chassis.shutdown()
        server = Server()
        ctl = Ctl(E1[:5] + (jaw,))
        with tempfile.TemporaryDirectory() as folder:
            svc = gs.GraspService(ctl, None, 1, folder+'/service.sock', chassis_server=server)
            svc.stop_requested = True
            with patch.dict(gs.os.environ, {'GRASP_SERVICE_STARTUP_TRAVEL': '1'}):
                svc.serve()
        return ctl, svc, server

    def test_serve_zeros_before_boot_pose_and_then_starts_tcp(self):
        ctl, svc, server = self.run_serve(30)
        self.assertTrue(svc.startup_pose['ok'])
        self.assertTrue(server.started)
        self.assertEqual(arms(ctl), [WAY, TRAVEL[:5]])
        self.assertTrue(all(v == (0,0,0,0) for v in server.chassis.device.calls))

    def test_serve_never_starts_tcp_on_boot_failure(self):
        ctl, svc, server = self.run_serve(100)
        self.assertFalse(svc.startup_pose['ok'])
        self.assertFalse(server.started)
        self.assertEqual(server.chassis.status()['arm_pose'], 'startup_failed')
        self.assertEqual(ctl.moves, [])

    def test_boot_audit_keeps_tcp_locked_after_parking(self):
        with patch.dict(gs.os.environ, {'GRASP_SERVICE_BOOT_AUDIT_LOCK': '1'}):
            ctl, svc, server = self.run_serve(30)
        self.assertTrue(svc.startup_pose['ok'])
        self.assertTrue(server.chassis.status()['maintenance'])
        self.assertTrue(server.locked_at_start)
        self.assertEqual(server.velocity_result, 'maintenance')
        self.assertTrue(all(v == (0,0,0,0) for v in server.chassis.device.calls))


if __name__ == '__main__':
    unittest.main()
