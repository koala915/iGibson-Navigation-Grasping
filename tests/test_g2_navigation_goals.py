import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
from g2_navigation_goals import nearest_patrol,nearest_route_entry,resolve_destination,patrol_goals,path_length
from g2_patrol_policy import Policy


def policy():return Policy(str(ROOT/'maps/demo_box_20261018_patrol'))


def test_nearest_uses_legal_path_length_not_euclidean_distance():
    p=policy();goals=patrol_goals(p)
    first,second=goals[:2]
    def plan(start,goal):
        if goal==first[1]:return [start,[10,10],goal]
        if goal==second[1]:return [start,goal]
        raise ValueError('disconnected')
    p.plan=plan
    assert nearest_patrol(p,first[1])['label']==second[0]


def test_preferred_outer_route_is_used_and_duplicate_removed():
    goals=patrol_goals(policy())
    assert len(goals)==6
    assert goals[0][0]=='outer_ccw_0'


def test_cycle_begins_at_nearest_and_returns_there_counterclockwise():
    p=policy();start=patrol_goals(p)[2][1]
    route=resolve_destination(p,start,'patrol-cycle')
    assert route[0]==route[-1] and route[0][0]=='outer_ccw_2'
    assert [label for label,_ in route][:3]==['outer_ccw_2','outer_ccw_3','outer_ccw_4']


def test_random_free_start_can_reach_patrol_and_dropoff_preparation():
    p=policy();count=0
    for i in range(12):
        for j in range(12):
            current=[-.1+i*.12,.05+j*.12]
            if not p.allowed(*current):continue
            a=resolve_destination(p,current,'nearest-patrol')[0][1]
            b=resolve_destination(p,current,'dropoff-preparation')[0][1]
            assert all(p.segment_allowed(x,y) for x,y in zip(p.plan(current,a),p.plan(current,a)[1:]))
            assert p.plan(current,b)
            count+=1
    assert count>=50


def test_outside_mask_is_reported_without_snapping_into_room():
    with pytest.raises(ValueError):nearest_patrol(policy(),[100,100])


def test_dropoff_does_not_use_unregistered_box_dock():
    p=policy();result=resolve_destination(p,[.6,.6],'dropoff-preparation')
    assert result[0][1]==p.layout['bin']['navigation_preparation']['point_map']
    assert p.layout['bin']['registration_verified'] is False


def test_patrol_can_join_mid_segment_without_detouring_to_a_corner():
    p=policy();a,b=patrol_goals(p)[:2]
    midpoint=[(a[1][i]+b[1][i])*.5 for i in (0,1)]
    entry=nearest_route_entry(p,midpoint)
    assert entry['point']==pytest.approx(midpoint)
    assert entry['next_index']==1
    route=resolve_destination(p,midpoint,'patrol-cycle')
    assert route[1][0]=='outer_ccw_1'
