import importlib.util
import json
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('scan_filter', Path(__file__).resolve().parents[1] / 'deploy/ros/g2_self_return_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_matched_ray_is_unknown_and_other_ranges_retained():
    result, count = module.filter_ranges([.3, .2, .3], 0, math.radians(1), {0: [.3]}, .025)
    assert math.isnan(result[0]) and result[1:] == [.2, .3] and count == 1


def test_metadata_geometry_not_cropped_and_invalid_rays_preserved():
    values = [float('inf'), float('nan'), .5]
    result, count = module.filter_ranges(values, -math.pi, .01, {}, .025)
    assert len(result) == len(values) and math.isinf(result[0]) and math.isnan(result[1])
    assert count == 0


def test_rounding_matches_profile_builder_at_half_degree():
    result, count = module.filter_ranges([.3], math.radians(.5), .01, {0: [.3]}, .025)
    assert count == 1


def test_profile_json_keys_loaded_as_integer(tmp_path):
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps({'schema': 1, 'frame_id': 'laser', 'tolerance_m': .025,
                               'ranges_by_angle_deg': {'90': [.3]}}))
    profile, tolerance = module.load_profile(path)
    assert profile == {90: [.3]} and tolerance == .025


def test_invalid_profile_rejected(tmp_path):
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps({'schema': 1, 'frame_id': 'laser', 'tolerance_m': .1,
                               'ranges_by_angle_deg': {'90': [.3]}}))
    with pytest.raises(ValueError):
        module.load_profile(path)


def test_fixed_profile_does_not_learn_new_obstacles():
    profile = {90: [.3]}
    result, count = module.filter_ranges([.2], math.pi/2, .01, profile, .025)
    assert result == [.2] and count == 0 and profile == {90: [.3]}
