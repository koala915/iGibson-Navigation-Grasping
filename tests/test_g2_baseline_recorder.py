import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from integration.g2_baseline_recorder import TopicStats


def test_repeated_stamp_not_counted_as_fresh():
    stats=TopicStats()
    stats.accept(10,1)
    stats.accept(10,2)
    result=stats.snapshot(3)
    assert result['count']==1 and result['arrival_age_s']==2


def test_gap_updates_on_new_frame_and_during_outage():
    stats=TopicStats()
    stats.accept(10,1)
    stats.accept(11,1.2)
    stats.accept(12,2)
    assert abs(stats.snapshot(2)['max_gap_s']-.8)<1e-9
    assert stats.snapshot(4)['max_gap_s']==2


def test_missing_topic_explicit():
    result=TopicStats().snapshot(5)
    assert result['count']==0 and result['arrival_age_s'] is None
