"""Fault injection for stop delivery after a swallowed Rosmaster write error."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'grasp/v23'))
from serial_transport import Transport
from chassis_server import Chassis


class Serial:
    def __init__(self):
        self.writes = []
        self.failure = None
        self.short = False

    def write(self, data):
        if self.failure is not None:
            failure, self.failure = self.failure, None
            raise failure
        if self.short:
            self.short = False
            return 0
        self.writes.append(data)
        return len(data)


def board():
    serial = Serial()
    device = SimpleNamespace(ser=serial)
    transport = Transport(device)
    device._v23_transport = transport
    # Match the real driver's exception-swallowing behaviour.
    def set_motor(*values):
        try:
            serial.write(bytes(v + 128 for v in values))
        except OSError:
            pass
    device.set_motor = set_motor
    transport.thread = Mock()
    transport.thread.is_alive.return_value = True
    transport.last_packet = transport.last_motion_packet = transport.clock()
    chassis = Chassis(device, lambda vx, wz: (30,) * 4,
                      rx_age=lambda: 0, log=lambda _: None)
    chassis.set_arm_pose('travel')
    return serial, transport, chassis


@pytest.mark.parametrize('trigger', ['exception', 'short', 'lock'])
def test_stop_is_delivered_after_failed_motion_and_fault_stays_latched(trigger):
    serial, transport, chassis = board()
    chassis.command({'action': 'velocity', 'vx': .15})
    if trigger == 'exception':
        serial.failure = OSError('temporary USB write failure')
    elif trigger == 'short':
        serial.short = True
    else:
        transport.lock.acquire()
    try:
        with pytest.raises(OSError):
            chassis.stop()
    finally:
        if trigger == 'lock':
            transport.lock.release()
    assert chassis.status()['motors'] == [30] * 4
    before = len(serial.writes)
    assert chassis.watchdog_tick()
    assert serial.writes[before:] == [bytes([128] * 4)]
    assert chassis.status()['motors'] == [0] * 4
    assert not transport.status()['healthy']
    assert transport.status()['write_fault']
    assert chassis.begin_arm() == 'board_silent'
    before = len(serial.writes)
    assert chassis.command({'action': 'velocity', 'vx': .15}) == 'board_silent'
    assert len(serial.writes) == before


def test_repeated_failed_stops_are_attempted_until_a_write_succeeds():
    serial, transport, chassis = board()
    chassis.command({'action': 'velocity', 'vx': .15})
    for _ in range(3):
        serial.failure = OSError('still disconnected')
        with pytest.raises(OSError):
            chassis.stop()
        assert chassis.status()['motors'] == [30] * 4
    chassis.stop()
    assert serial.writes[-1] == bytes([128] * 4)
    assert chassis.status()['motors'] == [0] * 4
    assert not transport.status()['healthy']


def test_write_result_belongs_to_the_calling_thread():
    serial, transport, _ = board()
    failed = threading.Event()
    other_done = threading.Event()
    outcomes = []
    serial.failure = OSError('thread A failed')
    def writer_a():
        try:
            transport.write(b'A')
        except OSError:
            pass
        failed.set()
        if not other_done.wait(2):
            outcomes.append('timeout')
            return
        try:
            transport.require_write_ok()
        except OSError:
            outcomes.append('A failed')
    def writer_b():
        if not failed.wait(2):
            other_done.set()
            return
        try:
            transport.write(b'B')
            transport.require_write_ok()
            outcomes.append('B sent')
        finally:
            other_done.set()
    a = threading.Thread(target=writer_a)
    b = threading.Thread(target=writer_b)
    a.start(); b.start(); a.join(3); b.join(3)
    assert not a.is_alive() and not b.is_alive()
    assert sorted(outcomes) == ['A failed', 'B sent']
    assert serial.writes == [b'B']
