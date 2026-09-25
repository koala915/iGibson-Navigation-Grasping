#!/usr/bin/env python3
"""Servo-read tests for the half-duplex bus, with a stub driver. No hardware.

Background
----------
On the first real run the arm walked to C3 and then failed on the first policy
step with "servo read failed: S1: invalid reading -1". The six servos are
daisy-chained on one half-duplex UART, so a read issued while the write burst is
still on the wire has nowhere to come back; Rosmaster_Lib's get_uart_servo_value
polls for 30 ms and then returns -1.

The evidence for that reading is in the run itself: the startup walk to C3 reads
back after a 0.35 s settle and succeeded, while the policy step reads with
settle_s=0.0 and failed on the very first one — S1, the first servo queried
after the write.

Four things fix it, and this file checks all four:

  1. get_uart_servo_angle_array() — one bus transaction with a single 30 ms
     budget, instead of six round trips at up to 30 ms each. Six per-servo reads
     do not even fit inside the 100 ms control period at 10 Hz.
  2. A quiet window after a write before any read is issued.
  3. Whole-read retries, so a collision costs a few milliseconds instead of
     aborting the run.
  4. Reading through get_uart_servo_value and crediting each answer to the servo
     that actually sent it. See below — this one turned out to be the real defect.

The fourth item, and why the first three were not enough
--------------------------------------------------------
Two more runs failed after the first three fixes were in: S6 at policy step 49,
then S3 AND S4 together on the very first read at startup, before a single
command had been sent. No write, so no contention with one — which rules out
everything the quiet window addresses.

The cause is in Rosmaster_Lib itself. get_uart_servo_value:

    self.__read_id = 0
    self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
    timeout = 30
    while timeout > 0:
        if self.__read_id > 0:
            return self.__read_id, self.__read_val      # <-- whoever answered

It hands back the first response to land, with no check that it came from the
servo that was asked. get_uart_servo_angle then compares the ids itself, finds a
mismatch, and returns -1 — discarding a good reading and blaming a servo that
answered perfectly well.

One late response therefore shifts the whole pipeline: S2's answer is collected
by S3's request, so S3 reports -1 while its own answer is collected by S4. The
signature is a run of ADJACENT servos unreadable at once, which is exactly
S3+S4, and it is why retrying did not clear it — each retry re-requests and
keeps the pipeline shifted.

Case 10 below runs the driver's own wrapper against that traffic and shows what
it costs: [90.0, -1, -1, -1, -1, -1] — five good readings thrown away.

What must NOT change is the fail-closed contract: when the servos genuinely
cannot be read, read_degrees still reports invalid rather than handing back the
last command as though it were a measurement. That is the v17 behaviour this
stack exists to avoid, and the last two checks pin it down.
"""
from __future__ import annotations

import math
import sys
import time

import x3plus_real_grasp as X


checks = 0
failures = []


def check(cond, label):
    global checks
    checks += 1
    if not cond:
        failures.append(label)
        print(f"  [FAIL] {label}")
    else:
        print(f"  [ok]   {label}")


def deg_to_raw(s_id, angle):
    """Rosmaster_Lib.__arm_convert_value, so the stub speaks in raw counts."""
    if not isinstance(angle, (int, float)) or not math.isfinite(angle):
        return float("nan")
    if s_id in (1, 2, 3, 4):
        return int((3100 - 900) * (angle - 180) / (0 - 180) + 900)
    if s_id == 5:
        return int((3700 - 380) * (angle - 0) / (270 - 0) + 380)
    return int((3100 - 900) * (angle - 0) / (180 - 0) + 900)


class BusStub:
    """Rosmaster stand-in that models the half-duplex line.

    A read arriving sooner than ``quiet_needed`` after a write returns -1, exactly
    as the real driver does when its 30 ms poll expires with no answer.

    With ``shift=True`` it also models the driver's response/request
    desynchronisation: get_uart_servo_value returns the first response to land
    regardless of which servo sent it, so a late answer from the previous servo is
    collected by the next request. That is a real defect in Rosmaster_Lib, not a
    hypothetical -- the loop at get_uart_servo_value's `if self.__read_id > 0:
    return self.__read_id, self.__read_val` has no comparison against the id that
    was asked for.
    """

    def __init__(self, com=None, quiet_needed=0.0, has_array=True,
                 array_supported=True, always_fail=False,
                 array_omits_gripper=False, shift=False):
        self.com = com
        self.quiet_needed = quiet_needed
        self.array_supported = array_supported
        self.always_fail = always_fail
        # FUNC_ARM_CTRL is an ARM request. A board that answers it for the five arm
        # joints and leaves the gripper empty is a normal outcome, and the suspected
        # shape of the real Jetson failure.
        self.array_omits_gripper = array_omits_gripper
        self.shift = shift
        self.shift_pending = 0
        assert has_array, "use BusStubNoArray for a driver without the array call"
        self.angles = [90.0, 67.08, 9.79, 9.79, 90.0, 30.0]
        self.last_write_t = 0.0
        self.transactions = 0          # bus round trips, the quantity that matters
        self.per_servo_calls = 0

    def create_receive_threading(self):
        pass

    def cancel_receive_threading(self):
        pass

    def set_uart_servo_angle_array(self, angle_s=None, run_time=None):
        self.last_write_t = time.time()
        self.angles = list(angle_s)

    def _collides(self):
        return (time.time() - self.last_write_t) < self.quiet_needed

    def get_uart_servo_angle_array(self):
        self.transactions += 1
        if self.always_fail or not self.array_supported or self._collides():
            return [-1] * 6
        if self.array_omits_gripper:
            return list(self.angles[:5]) + [-1]
        return list(self.angles)

    def get_uart_servo_value(self, s_id):
        """(responding_id, raw_count) — the call that does NOT throw the id away."""
        self.transactions += 1
        self.per_servo_calls += 1
        if self.always_fail or self._collides():
            return -1, -1
        responder = self.shift_pending or s_id
        self.shift_pending = s_id if self.shift else 0
        return responder, deg_to_raw(responder, self.angles[responder - 1])

    def get_uart_servo_angle(self, s_id):
        """The driver's own wrapper, id-mismatch defect faithfully included.

        Nothing in the deployment stack calls this any more; it stays so the tests
        can show what it costs.
        """
        read_id, raw = self.get_uart_servo_value(s_id)
        if read_id != s_id:
            return -1
        deg = X._rosmaster_raw_to_deg(read_id, raw)
        return -1 if deg is None else deg


class BusStubNoArray(BusStub):
    """A driver predating get_uart_servo_angle_array.

    Deleting the method off BusStub would remove it for every later case too, and
    an earlier draft of this file did exactly that — which made the "array answers
    -1" case pass for the wrong reason. Hiding it per-instance keeps the cases
    independent.
    """

    def __getattribute__(self, name):
        if name == "get_uart_servo_angle_array":
            raise AttributeError(name)
        return super().__getattribute__(name)


class BusStubNoValue(BusStub):
    """A driver exposing only the angle wrapper, with no raw get_uart_servo_value.

    The read layer prefers the raw call so it can see which servo answered. If a
    board or an older Rosmaster_Lib does not have it, reading through the wrapper
    is still better than not reading at all, and this pins that path down.
    """

    def __getattribute__(self, name):
        if name == "get_uart_servo_value":
            raise AttributeError(name)
        return super().__getattribute__(name)

    def get_uart_servo_angle(self, s_id):
        self.transactions += 1
        self.per_servo_calls += 1
        if self.always_fail or self._collides():
            return -1
        return self.angles[s_id - 1]


def build(**kw):
    """A ServoController wired to a BusStub, bypassing the real driver."""
    cfg = X.DeployConfig()
    for k in ("bus_quiet_s", "read_attempts", "read_retry_s"):
        if k in kw:
            setattr(cfg, k, kw.pop(k))
    cls = kw.pop("cls", None)
    if cls is None:
        cls = BusStub if kw.pop("has_array", True) else BusStubNoArray
    else:
        kw.pop("has_array", None)
    servo = X.ServoController(cfg, dry_run=True)   # never opens a serial port
    servo.dry_run = False
    servo.readings_are_simulated = False
    servo._device = cls(**kw)
    servo._has_servo = True
    return servo, servo._device


print(__doc__.strip().split("\n")[0])
print()

print("1. the array API is preferred: one bus transaction, not six")
servo, bus = build()
r = servo.read_degrees()
check(r.valid, "read succeeds on a quiet bus")
check(bus.transactions == 1, f"1 bus transaction (got {bus.transactions})")
check(bus.per_servo_calls == 0,
      f"per-servo path untouched (got {bus.per_servo_calls} calls)")
check(r.degrees == [90.0, 67.08, 9.79, 9.79, 90.0, 30.0], "angles come back intact")

print("\n2. the Jetson failure: a read issued straight after a write")
# 40 ms of contention -- longer than the 20 ms quiet window, so the first attempt
# collides and the retry has to carry it. This is the exact reported failure.
servo, bus = build(quiet_needed=0.040)
servo.send_degrees([90.0, 67.08, 9.79, 9.79, 90.0, 30.0], run_time_ms=200)
t0 = time.time()
r = servo.read_degrees()
elapsed = time.time() - t0
check(r.valid, "read now succeeds where the old code reported S1: -1")
check(bus.transactions >= 2, f"it took a retry ({bus.transactions} transactions)")
check(elapsed < 0.5, f"recovery cost {elapsed*1000:.0f} ms, not a run abort")

print("\n3. the quiet window alone handles short contention")
servo, bus = build(quiet_needed=0.015)   # shorter than the 20 ms window
servo.send_degrees([90.0, 67.08, 9.79, 9.79, 90.0, 30.0], run_time_ms=200)
r = servo.read_degrees()
check(r.valid, "read succeeds")
check(bus.transactions == 1,
      f"no retry needed: the wait absorbed it ({bus.transactions} transactions)")

print("\n4. fallback when the board has no array API")
servo, bus = build(has_array=False)
r = servo.read_degrees()
check(r.valid, "falls back to per-servo reads and still succeeds")
check(bus.per_servo_calls == 6, f"read all six individually ({bus.per_servo_calls})")

print("\n5. fallback when the array API exists but answers -1")
servo, bus = build(array_supported=False)
r = servo.read_degrees()
check(r.valid, "per-servo path rescues an unsupported array call")
check(bus.per_servo_calls == 6, f"read all six individually ({bus.per_servo_calls})")

print("\n6. array answers for the arm but not the gripper: fill the hole, not the array")
# The suspected shape of the real failure. The first version of this fix threw the
# whole array response away and re-read all six individually -- 7 transactions per
# step where 2 suffice, and MORE bus traffic than doing nothing at all.
servo, bus = build(array_omits_gripper=True)
r = servo.read_degrees()
check(r.valid, "read succeeds")
check(bus.transactions == 2, f"2 transactions, not 7 (got {bus.transactions})")
check(bus.per_servo_calls == 1,
      f"only the gripper re-read individually (got {bus.per_servo_calls})")
check(r.degrees[5] == 30.0, f"gripper angle correct (got {r.degrees[5]})")

print("\n7. a board that never serves the array request stops being asked")
servo, bus = build(array_supported=False)
for _ in range(6):
    servo.read_degrees()
first = bus.transactions
servo.read_degrees()
check(bus.transactions - first == 6,
      f"steady state is 6 per-servo reads, no wasted array call "
      f"(got {bus.transactions - first})")

print("\n8. read failures are counted per servo for the run summary")
servo, bus = build(always_fail=True, read_attempts=1)
servo.read_degrees()
servo.read_degrees()
summary = servo.read_failure_summary()
check("S6×2" in summary, f"gripper failures counted: {summary!r}")
check("2" in summary, f"summary reports a total: {summary!r}")

print("\n9. a shifted bus pipeline is recovered, not reported as failure")
# The S3+S4 startup failure of 2026-07-31. Every response comes back labelled with
# the PREVIOUS servo, because get_uart_servo_value returns whatever landed first.
# Crediting each answer to the servo that sent it converges in one extra round trip.
servo, bus = build(array_supported=False, shift=True)
r = servo.read_degrees()
check(r.valid, "all six read despite every response arriving off by one")
check(all(math.isfinite(d) for d in r.degrees), f"no NaN got through: {r.degrees}")
check(servo._read_misdirects > 0,
      f"misdirected responses were counted ({servo._read_misdirects})")
summary = servo.read_failure_summary()
check(summary.startswith("servo reads: no failures"),
      f"a recovered shift is not a failure: {summary!r}")
check("misdirected" in summary, f"but it is still reported as evidence: {summary!r}")

print("\n10. and this is what the driver's own wrapper does with that same traffic")
# Not a test of our code -- a record of the defect being worked around, so that
# nobody 'simplifies' _read_one back to get_uart_servo_angle later.
bus2 = BusStub(array_supported=False, shift=True)
lost = [bus2.get_uart_servo_angle(i) for i in range(1, 7)]
check(lost[0] != -1, f"the first read, still in step, works (got {lost[0]})")
check(lost.count(-1) == 5,
      f"the other five good readings are discarded as -1: {lost}")

print("\n11. a driver without the raw call still reads, through the wrapper")
servo, bus = build(cls=BusStubNoValue, array_supported=False)
r = servo.read_degrees()
check(r.valid, "falls back to get_uart_servo_angle and succeeds")
check(bus.per_servo_calls == 6, f"six per-servo reads ({bus.per_servo_calls})")

print("\n12. FAIL-CLOSED: a genuinely unreadable bus is still reported unknown")
servo, bus = build(always_fail=True, read_attempts=2, read_retry_s=0.0)
last_cmd = list(servo._last_deg)
r = servo.read_degrees()
check(not r.valid, "invalid, not optimistic")
check(all(math.isnan(d) for d in r.degrees),
      f"every angle NaN, no last-command substitution (got {r.degrees})")
check(r.degrees != last_cmd, "did not hand back the last command as a measurement")
check("attempts" in r.reason, f"reason names the retries: {r.reason!r}")

print("\n13. FAIL-CLOSED: NaN from the driver is not waved through")
# "nan < 0" is False, so a bare range check would accept NaN as a real angle.
servo, bus = build()
bus.angles = [90.0, float("nan"), 9.79, 9.79, 90.0, 30.0]
r = servo.read_degrees()
check(not r.valid, "NaN rejected rather than treated as a valid reading")
check("S2" in r.reason, f"names the offending servo: {r.reason!r}")

print("\n14. no device: unchanged fail-closed behaviour")
cfg = X.DeployConfig()
servo = X.ServoController(cfg, dry_run=True)
servo.dry_run = False
servo.readings_are_simulated = False
servo._has_servo = False
r = servo.read_degrees()
check(not r.valid, "invalid without a board")
check(all(math.isnan(d) for d in r.degrees), "all NaN without a board")

print()
if failures:
    print(f"{len(failures)} of {checks} checks FAILED:")
    for f in failures:
        print(f"  - {f}")
    sys.exit(1)
print(f"all {checks} checks passed — servo reads survive the half-duplex bus")
sys.exit(0)
