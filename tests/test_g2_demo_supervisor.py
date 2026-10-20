import importlib.util
import sys
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'integration'))
from g2_demo_supervisor import Lease, Recovery, memory_available_mb

spec = importlib.util.spec_from_file_location('mapping_filter', ROOT/'deploy/ros/g2_mapping_filter.py')
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


def test_intent_expiry_and_replay():
    lease = Lease()
    lease.accept(dict(seq=1,vx=0.15,ttl=0.3), 10)
    assert lease.active(10.2)
    assert not lease.active(10.31)
    with pytest.raises(ValueError):
        lease.accept(dict(seq=1,vx=0.15,ttl=0.3), 10.4)
    lease.accept(dict(seq=2,vx=0,ttl=0.3), 10.5)
    assert not lease.active(10.5)


@pytest.mark.parametrize('vx,ttl',[(float('nan'),0.3),(0.15,float('inf')),(-0.1,0.2),(0.2,0.2),(0.15,1)])
def test_invalid_intent(vx,ttl):
    with pytest.raises(ValueError):
        Lease().accept(dict(seq=1,vx=vx,ttl=ttl), 1)


def test_hazard_stops_immediately_and_clearance_debounces():
    recovery = Recovery()
    assert not recovery.tick('lidar_brake',1)
    assert not recovery.tick('',1.1)
    assert not recovery.tick('lidar_brake',1.4)
    assert not recovery.tick('',1.5)
    assert recovery.tick('',2.01)
    with pytest.raises(RuntimeError):
        recovery.tick('stale_stamp',2.1)


def test_recovery_budget():
    recovery = Recovery()
    for t in (1,3):
        assert not recovery.tick('lidar_brake',t)
        assert not recovery.tick('',t+0.1)
        assert recovery.tick('',t+0.7)
    recovery.tick('lidar_brake',5)
    recovery.tick('',5.1)
    with pytest.raises(RuntimeError):
        recovery.tick('',5.7)


def test_mapping_raw_forward_pi_and_masks_do_not_clear_space():
    import math
    raw = [1.0]*360
    result = mapping.filter_ranges(raw,-math.pi,math.pi/180)
    assert result[0] == 1.0  # raw -pi is physical forward
    assert result[180] > 3.0  # raw zero is physical rear
    assert raw == [1.0]*360
    raw[0] = 0.05
    assert mapping.filter_ranges(raw,-math.pi,math.pi/180)[0] > 3.0


def test_mapping_speckle_opt_in():
    assert mapping.filter_ranges([1,0.2,1],0,0.01,front_only=False)[1] == 0.2
    assert mapping.filter_ranges([1,0.2,1],0,0.01,front_only=False,speckle=True)[1] > 3


def test_memory_uses_available_not_free(tmp_path):
    path = tmp_path/'meminfo'
    path.write_text('MemFree: 100 kB\nMemAvailable: 1048576 kB\n')
    assert memory_available_mb(path) == 1024
