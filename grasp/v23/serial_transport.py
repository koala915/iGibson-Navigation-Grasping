"""One bounded receiver/writer for Rosmaster; never reopens or replays motion."""
import threading
import time


class Transport:
    def __init__(self, device, clock=time.monotonic):
        self.device, self.ser, self.clock = device, device.ser, clock
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.last_packet = None
        self.last_motion_packet = None
        self.read_errors = 0
        self.write_fault = ''
        self.rx_fault = ''
        self.ser.timeout = 0.1
        self.ser.write_timeout = 0.2
        self.raw_write = self.ser.write
        self.ser.write = self.write
        # Existing chassis installer must reuse this bounded lock wrapper.
        self.ser._g2_write_lock = self.lock

    def write(self, data):
        if self.write_fault:
            raise OSError(self.write_fault)
        if not self.lock.acquire(timeout=0.2):
            self.write_fault = 'serial_write_lock_timeout'
            raise OSError(self.write_fault)
        try:
            sent = self.raw_write(data)
            if sent != len(data):
                raise OSError('short_serial_write')
            return sent
        except Exception as exc:
            self.write_fault = 'serial_write_failed: %s' % exc
            raise
        finally:
            self.lock.release()

    def require_write_ok(self):
        # Driver methods catch their own exceptions: preserve the failure here.
        if self.write_fault:
            raise OSError(self.write_fault)

    def status(self):
        age = None if self.last_packet is None else self.clock()-self.last_packet
        motion_age = None if self.last_motion_packet is None else self.clock()-self.last_motion_packet
        alive = bool(self.thread and self.thread.is_alive())
        return dict(healthy=alive and not self.write_fault and not self.rx_fault
                    and age is not None and 0 <= age <= 0.5
                    and motion_age is not None and 0 <= motion_age <= 0.5,
                    receiver_alive=alive, packet_age_s=age,
                    motion_packet_age_s=motion_age,
                    read_errors=self.read_errors, write_fault=self.write_fault,
                    rx_fault=self.rx_fault)

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop.clear()
        self.thread = threading.Thread(target=self.receive, name='v23-serial-rx', daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=0.5)

    def receive(self):
        frame = bytearray()
        last_byte = self.clock()
        while not self.stop.is_set():
            try:
                data = self.ser.read(1)
                if not data:
                    if self.clock()-last_byte > 0.2:
                        frame.clear()
                    continue
                if self.clock()-last_byte > 0.2:
                    frame.clear()
                last_byte = self.clock()
                frame.extend(data)
                while frame:
                    if frame[0] != 0xff:
                        del frame[0]
                        continue
                    if len(frame) < 2:
                        break
                    if frame[1] != 0xfb:
                        del frame[0]
                        continue
                    if len(frame) < 3:
                        break
                    length = frame[2]
                    if not 3 <= length <= 64:
                        del frame[0]
                        continue
                    total = length+2
                    if len(frame) < total:
                        break
                    packet = bytes(frame[:total])
                    del frame[:total]
                    if sum(packet[2:-1]) % 256 != packet[-1]:
                        continue
                    self.device._Rosmaster__parse_data(packet[3], list(packet[4:]))
                    self.last_packet = self.clock()
                    if packet[3] == 0x0a:
                        self.last_motion_packet = self.last_packet
                    self.rx_fault = ''
            except Exception as exc:
                frame.clear()
                self.read_errors += 1
                self.rx_fault = 'serial_receive_failed: %s' % exc
                # Passive reads only: no thread churn, reopen or repeated commands.
                self.stop.wait(min(1.0, 0.1*self.read_errors))


def install(device):
    transport = Transport(device)
    device._v23_transport = transport
    device.create_receive_threading = transport.start
    device.cancel_receive_threading = transport.close
    return transport
