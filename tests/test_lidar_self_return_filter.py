from types import SimpleNamespace
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration.lidar_self_return_filter import SelfReturnFilter


def scans(points):
    return [SimpleNamespace(stamp=i + 1, points=points) for i in range(10)]


def test_only_matching_angle_and_range_removed():
    f = SelfReturnFilter.from_clear_scans(scans([(90, .3)]))
    assert f.apply([(90, .31), (90, .2), (88, .3), (90, .5)]) == ((90, .2), (88, .3), (90, .5))


def test_far_wall_not_learned():
    f = SelfReturnFilter.from_clear_scans(scans([(0, 1.0)]))
    assert f.apply([(0, 1.0)]) == ((0, 1.0),)


def test_intermittent_or_unstable_returns_not_learned():
    data = scans([(0, .3), (90, .2)])
    for i in range(3):
        data[i].points = [(90, .3)]
    f = SelfReturnFilter.from_clear_scans(data)
    assert f.ranges_by_angle == {}


def test_duplicate_frames_rejected():
    data = scans([(0, .3)])
    data[1].stamp = data[0].stamp
    with pytest.raises(ValueError):
        SelfReturnFilter.from_clear_scans(data)


def test_short_capture_rejected():
    with pytest.raises(ValueError):
        SelfReturnFilter.from_clear_scans(scans([])[:9])


def test_mask_does_not_grow_during_apply():
    f = SelfReturnFilter.from_clear_scans(scans([(0, .3)]))
    before = dict(f.ranges_by_angle)
    for _ in range(20):
        assert f.apply([(90, .3)]) == ((90, .3),)
    assert before == f.ranges_by_angle


def test_invalid_return_not_silently_removed():
    f = SelfReturnFilter({0: .3})
    assert len(f.apply([(0, float('nan'))])) == 1


def test_tolerance_cannot_be_widened():
    with pytest.raises(ValueError):
        SelfReturnFilter({}, .1)


def test_side_confirmation_distinct_frames_and_near_stop():
    from integration.g2_filtered_turn_calibration import SideConfirmation
    gate = SideConfirmation()
    assert gate.update(1, 'side_right', .4) == ''
    assert gate.update(1, 'side_right', .4) == ''
    assert gate.update(2, 'side_right', .4) == ''
    assert gate.update(3, 'side_right', .4) == 'side_right'
    assert gate.update(4, '', float('inf')) == ''
    assert gate.update(5, 'side_left', .2) == 'side_left'


def test_changing_side_or_range_resets_confirmation():
    from integration.g2_filtered_turn_calibration import SideConfirmation
    gate = SideConfirmation()
    assert gate.update(1, 'side_left', .4) == ''
    assert gate.update(2, 'side_right', .4) == ''
    assert gate.update(3, 'side_right', .6) == ''
