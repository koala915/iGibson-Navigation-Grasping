import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from integration.g2_quarter_turn_calibration import swept_hazard, EnvelopeGate, pulse_duration, directed_yaw, target_limits


def points(x,y):
    a=math.degrees(math.atan2(y,x-.13))-180
    r=math.hypot(x-.13,y)
    return [(a+d,r) for d in [-.2,0,.2]]


def test_body_returns_excluded_but_near_external_cluster_retained():
    assert not swept_hazard(points(0,.10))
    assert swept_hazard(points(0,.20))


def test_far_wall_not_in_rotating_circle():
    assert not swept_hazard(points(0,.4))


def test_approximate_body_dimensions_allow_two_cm_tolerance():
    assert not swept_hazard(points(-.14,.125))


def test_single_speckle_not_cluster():
    assert not swept_hazard(points(0,.20)[:1])


def test_gate_requires_distinct_frames_and_resets():
    gate=EnvelopeGate()
    for stamp,expected in [(1,False),(1,False),(2,False),(3,True)]:
        assert gate.check(SimpleNamespace(stamp=stamp,points=points(0,.20)))==expected
    assert not gate.check(SimpleNamespace(stamp=4,points=[]))


def test_pulses_shorten_near_target():
    assert pulse_duration(70)>pulse_duration(15)>pulse_duration(3)>0


def test_right_turn_has_positive_progress_and_wrong_direction_negative():
    assert abs(directed_yaw(0,-math.pi/2,-1)-90)<1e-9
    assert directed_yaw(0,math.pi/2,-1)<0
    assert directed_yaw(0,0,-1)==0


def test_right_turn_across_wrap():
    assert abs(directed_yaw(math.radians(-170),math.radians(100),-1)-90)<1e-9


def test_small_targets_have_tighter_stop_and_hard_limits():
    assert target_limits(5)==(4.6,5.8,8)
    assert target_limits(10)==(9.6,10.8,13)
    assert target_limits(90)==(89.2,91.5,95)
