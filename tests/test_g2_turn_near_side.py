"""A near wall must not disappear inside the turn probe's old 35 cm band."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration import g2_turn_probe as turn
from integration.nav_rl import NavRLConfig


CFG = NavRLConfig(lidar_yaw_offset_deg=180.0, lidar_forward_offset_m=0.10)


def wall(side, distance):
    # Physical right is raw +90 deg with the verified 180 deg mounting yaw.
    raw_angle = 90.0 if side == 'right' else -90.0
    return [(raw_angle + delta, distance) for delta in (-0.2, 0.0, 0.2)]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('distance', [0.15, 0.20, 0.29, 0.50])
def test_near_external_wall_is_reported(side, distance):
    reason, clearance = turn.side_hazard(wall(side, distance), CFG)
    assert reason == 'side_' + side
    assert clearance == pytest.approx(distance, abs=1e-5)


def test_both_near_walls_are_reported():
    reason, _ = turn.side_hazard(wall('left', 0.20) + wall('right', 0.20), CFG)
    assert reason == 'side_left+side_right'


def test_right_wall_is_retained_alongside_farther_left_returns():
    reason, clearance = turn.side_hazard(wall('left', 0.40) + wall('right', 0.20), CFG)
    assert reason == 'side_left+side_right'
    assert clearance == pytest.approx(0.20, abs=1e-5)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_body_width_and_isolated_speckles_still_excluded(side):
    assert turn.side_hazard(wall(side, 0.10), CFG) == ('', math.inf)
    assert turn.side_hazard(wall(side, 0.20)[:1], CFG) == ('', math.inf)


def test_far_return_is_outside_probe_zone():
    assert turn.side_hazard(wall('right', 0.70), CFG) == ('', math.inf)
