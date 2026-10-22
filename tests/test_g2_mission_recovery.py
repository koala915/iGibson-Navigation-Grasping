import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from integration import g2_mission_recovery as recovery
from integration import g2_patrol_reentry as reentry

spec = importlib.util.spec_from_file_location('reentry_fakes', Path(__file__).with_name('test_g2_patrol_reentry.py'))
fakes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fakes)
Plant, Mask, fake_mission = fakes.Plant, fakes.Mask, fakes.fake_mission


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, duration): self.t += duration


def sample(stamp=1., pose=None, context=('boot', 7, 'map')):
    return {'pose': pose or [1., 1., 0.], 'odom': [0., 0., 0.], 'stamp': stamp, 'context': context}


def test_transient_amcl_outage_stops_once_then_requires_five_new_poses():
    clock, stop, events = Clock(), Mock(), []
    values = iter([recovery.RecoveryFault('AMCL_STALE', 'delayed', True, 8.),
                   sample(1.), sample(1.), sample(2.), sample(3.), sample(4.), sample(5.)])
    def read():
        value = next(values)
        if isinstance(value, Exception): raise value
        return value
    result = recovery.wait_snapshot(read, stop, previous=sample(0.), clock=clock.now,
                                    sleep=clock.sleep, events=events)
    assert result['stamp'] == 5. and stop.call_count == 1
    assert events[-1]['status'] == 'READY_TO_REPLAN' and events[-1]['stable_samples'] == 5


def test_existing_supervisor_wait_is_not_reset_by_the_task_layer():
    clock = Clock()
    def read(): raise recovery.RecoveryFault('AMCL_STALE', 'still delayed', True, 20.)
    with pytest.raises(recovery.RecoveryFault, match='RECOVERY_TIMEOUT'):
        recovery.wait_snapshot(read, Mock(), clock=clock.now, sleep=clock.sleep)
    assert 9.9 <= clock.t <= 10.2


def test_expired_reference_does_not_trigger_restarts_or_action_replay():
    stop = Mock()
    def read(): raise recovery.localization_fault({'reason_code': 'REFERENCE_REQUIRED', 'reason': 'seed expired'})
    with pytest.raises(recovery.RecoveryFault, match='REFERENCE_REQUIRED'):
        recovery.wait_snapshot(read, stop)
    stop.assert_not_called()


@pytest.mark.parametrize('context', [('new-boot', 7, 'map'), ('boot', 8, 'map'), ('boot', 7, 'new-map')])
def test_context_change_never_automatically_reanchors_a_task(context):
    with pytest.raises(recovery.RecoveryFault, match='CONTEXT_CHANGED'):
        recovery.wait_snapshot(lambda: sample(context=context), Mock(), previous=sample())


def test_pose_jump_after_recovery_requires_an_absolute_reference():
    clock, values = Clock(), iter([recovery.RecoveryFault('AMCL_STALE', 'restarted', True)]
                                  + [sample(s, pose=[1.3, 1., 0.]) for s in (1., 2., 3., 4., 5.)])
    def read():
        value = next(values)
        if isinstance(value, Exception): raise value
        return value
    with pytest.raises(recovery.RecoveryFault, match='REFERENCE_REQUIRED'):
        recovery.wait_snapshot(read, Mock(), previous=sample(), anchor=sample(),
                               clock=clock.now, sleep=clock.sleep)


def test_drift_since_an_old_snapshot_is_not_a_jump():
    # The snapshot before a grasp and placement is no baseline for a jump: the
    # odometry drift the controllers accumulated meanwhile would stop a healthy
    # task right after a successful release. Without an outage nothing is compared.
    assert recovery.wait_snapshot(lambda: sample(pose=[1.3, 1., 0.]), Mock(),
                                  previous=sample())['pose'] == [1.3, 1., 0.]


def test_repeated_old_odom_cannot_satisfy_recovery_acknowledgement():
    clock, first = Clock(), [True]
    def read():
        if first[0]:
            first[0] = False
            raise recovery.RecoveryFault('SUPERVISOR_STALE', 'old health', True)
        return sample(1.)
    with pytest.raises(recovery.RecoveryFault, match='RECOVERY_TIMEOUT'):
        recovery.wait_snapshot(read, Mock(), seconds=1., clock=clock.now, sleep=clock.sleep)


def test_post_place_glitch_does_not_repeat_the_completed_retreat(tmp_path, monkeypatch):
    plant = Plant()
    mission, health = fake_mission(plant)
    locate, faults = mission.dock.read_localization, [2]
    def delayed(host):
        value = locate(host)
        if plant.commands and faults[0]:
            faults[0] -= 1
            value.update(ready=False, reason_code='AMCL_STALE', reason='AMCL delayed')
        return value
    mission.dock.read_localization = delayed
    clock, wait = Clock(), recovery.wait_snapshot
    monkeypatch.setattr(recovery, 'wait_snapshot', lambda read, stop, **kw:
                        wait(read, stop, clock=clock.now, sleep=clock.sleep, **kw))
    reentry.bind_mission(mission, 'scene', tmp_path, lambda host: health, recovery=recovery)
    room = mission.Room('host', walls=[1], mask=Mask())
    mission.place_held()
    result = room.leave()
    assert result['ok'] and len(plant.commands) == 3
    assert sum(plant.commands) == pytest.approx(.27)
    assert result['distance_m'] == pytest.approx(.27)
    assert any(e['status'] == 'READY_TO_REPLAN' for e in result['recovery_events'])


@pytest.mark.parametrize('reason', ['localization supervisor: stale AMCL',
                                  'AMCL no-motion update worker stuck; supervisor restart required',
                                  'sensor recovery timeout: stale scan/odom'])
def test_patrol_localization_failure_redispatches_only_the_patrol(tmp_path, monkeypatch, reason):
    clock, wait = Clock(), recovery.wait_snapshot
    monkeypatch.setattr(recovery, 'wait_snapshot', lambda read, stop, **kw:
                        wait(read, stop, clock=clock.now, sleep=clock.sleep, **kw))
    stamp = [1.]
    def read():
        stamp[0] += 1
        return sample(stamp[0])
    # A whole lap from the room's last snapshot: 3 m of odometry against a map
    # pose back near it. Only the patrol's own last stopped state is the anchor.
    room = SimpleNamespace(_last_snapshot=dict(sample(0.), odom=[3., 0., 0.]),
                           _snapshot_once=read)
    patrol = Mock(side_effect=[RuntimeError('patrol stopped: ' + reason), {'point': [1., 0.]}])
    mission = SimpleNamespace(patrol_until_sugarbox=patrol, MissionError=RuntimeError,
        patrol_report=lambda host: {'sensor_wait_elapsed_s': 20.,
                                    'trace': [{'pose': [1., 1., 0.], 'odom': [0., 0., 0.]}]},
        runner=SimpleNamespace(stop=Mock()), VisionLink=Mock())
    recovery.bind_patrol(mission, {'host': room}, tmp_path)
    link = SimpleNamespace(alive=lambda: True)
    result = mission.patrol_until_sugarbox('host', 'scene', None, None, link)
    assert result['point'] == [1., 0.] and patrol.call_count == 2
    mission.VisionLink.assert_not_called()


def test_nonlocalization_failure_is_not_hidden_as_a_recovery_retry(tmp_path):
    patrol = Mock(side_effect=RuntimeError('grasp outcome unknown'))
    mission = SimpleNamespace(patrol_until_sugarbox=patrol, MissionError=RuntimeError)
    recovery.bind_patrol(mission, {}, tmp_path)
    with pytest.raises(RuntimeError, match='unknown'):
        mission.patrol_until_sugarbox('host')
    assert patrol.call_count == 1


@pytest.mark.parametrize('second_failure', [False, True])
def test_dead_vision_link_is_replaced_once_and_retry_is_bounded(tmp_path, monkeypatch, second_failure):
    clock, wait = Clock(), recovery.wait_snapshot
    monkeypatch.setattr(recovery, 'wait_snapshot', lambda read, stop, **kw:
                        wait(read, stop, clock=clock.now, sleep=clock.sleep, **kw))
    stamp = [1.]
    def read():
        stamp[0] += 1
        return sample(stamp[0])
    room = SimpleNamespace(_last_snapshot=sample(0.), _snapshot_once=read)
    error = RuntimeError('vision permit link to the robot ended')
    patrol = Mock(side_effect=[error, error if second_failure else {'point': [1., 0.]}])
    old, fresh = Mock(), Mock()
    old.alive.return_value = False
    mission = SimpleNamespace(patrol_until_sugarbox=patrol, MissionError=RuntimeError,
        patrol_report=lambda host: {}, runner=SimpleNamespace(stop=Mock()),
        VisionLink=Mock(return_value=fresh))
    recovery.bind_patrol(mission, {'host': room}, tmp_path)
    if second_failure:
        with pytest.raises(RuntimeError, match='permit link'):
            mission.patrol_until_sugarbox('host', 'scene', None, None, old)
    else:
        assert mission.patrol_until_sugarbox('host', 'scene', None, None, old)['point'] == [1., 0.]
    assert patrol.call_count == 2 and patrol.call_args.args[4] is fresh
    mission.VisionLink.assert_called_once_with('host')
    old.close.assert_called_once()
    fresh.close.assert_called_once()


def test_patrol_does_not_wait_or_retry_when_reference_expired(tmp_path):
    patrol = Mock(side_effect=RuntimeError('localization unavailable'))
    mission = SimpleNamespace(patrol_until_sugarbox=patrol, MissionError=RuntimeError,
        patrol_report=lambda host: {'last_localization': {'status': 'needs_reference'}},
        runner=SimpleNamespace(stop=Mock()))
    recovery.bind_patrol(mission, {'host': Mock()}, tmp_path)
    with pytest.raises(recovery.RecoveryFault, match='REFERENCE_REQUIRED'):
        mission.patrol_until_sugarbox('host')
    assert patrol.call_count == 1


def test_wait_timeout_does_not_change_when_fault_kind_changes():
    clock, count = Clock(), [0]
    def read():
        count[0] += 1
        raise recovery.RecoveryFault('AMCL_STALE' if count[0] % 2 else 'SUPERVISOR_STALE', '', True)
    with pytest.raises(recovery.RecoveryFault, match='RECOVERY_TIMEOUT'):
        recovery.wait_snapshot(read, Mock(), seconds=1., clock=clock.now, sleep=clock.sleep)
    assert 1. <= clock.t <= 1.2
