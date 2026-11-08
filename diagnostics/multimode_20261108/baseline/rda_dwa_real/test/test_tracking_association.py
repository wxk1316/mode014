"""Source-time + size matching, bounded CV prediction and three-frame expiry."""
from pathlib import Path
import sys
import math
from unittest.mock import patch
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa_real'),str(ROOT/'rda_dwa')]
import rda_dwa_real.kf_tracker as kf
from rda_dwa_real.kf_tracker import GlobalKFTracker, HungarianMatcher
from rda_dwa.motion_safety import obstacle_from_object, minimum_gaps, VehicleGeometry
from types import SimpleNamespace


def detection(x, length=.28, width=.28):
    return x,0.,'pedestrian',length,width,0.


def test_actual_interval_changes_reachability_at_1m_per_second():
    for dt,same_id in ((.05,False),(.20,True)):
        tracker=GlobalKFTracker()
        tracker.step([detection(0.)],dt=.05)
        out=tracker.step([detection(.30)],dt=dt)
        assert (len(out)==1 and out[0][0]==1)==same_id


def test_prediction_residual_has_half_metre_cap_even_after_long_source_gap():
    tracker=GlobalKFTracker()
    tracker.step([detection(0.)],dt=.05)
    out=tracker.step([detection(.6)],dt=.7)
    assert len(out)==2 and out[-1][0]==2


def test_reassociation_uses_elapsed_source_time_and_preserves_dynamic_id():
    tracker=GlobalKFTracker()
    for i in range(30):previous=tracker.step([detection(i*.08)],dt=.1)[0]
    track=tracker.tracks[1];position=track.position();vx,vy=track.velocity()
    assert previous[5]=='pedestrian' and vx>.7
    for dt in (.08,.17):
        out=tracker.step([],dt=dt)[0]
        assert out[0]==1 and out[5]=='pedestrian'
    out=tracker.step([detection(position[0]+vx*.38)],dt=.13)
    assert len(out)==1 and out[0][0]==1 and out[0][5]=='pedestrian'


def test_first_two_misses_publish_kf_state_at_current_cloud_time_third_deletes():
    tracker=GlobalKFTracker()
    for i in range(30):previous=tracker.step([detection(i*.08)],dt=.1)[0]
    track=tracker.tracks[1];xy=np.array(track.position());velocity=np.array(track.velocity())
    size=previous[6:9];mv,st=track._mv_cnt,track._st_cnt;elapsed=0.
    for dt in (.05,.12):
        elapsed+=dt;out=tracker.step([],dt=dt)[0]
        np.testing.assert_allclose(out[1:3],xy+velocity*elapsed)
        assert out[5]=='pedestrian' and out[3]>.7 and out[6:9]==size
        assert (track._mv_cnt,track._st_cnt)==(mv,st)
    assert tracker.step([],dt=.09)==[]


def test_static_tracks_also_delete_on_third_miss():
    tracker=GlobalKFTracker(static_hold_frames=3)
    for _ in range(5):tracker.step([detection(0.)],dt=.05)
    for _ in range(2):assert tracker.step([])[0][5]=='block'
    assert tracker.step([])==[]


def test_confirmed_static_miss_output_does_not_drift_on_small_kf_velocity_noise():
    tracker=GlobalKFTracker(static_hold_frames=3)
    for _ in range(5):last=tracker.step([detection(0.)],dt=.1)[0]
    track=tracker.tracks[1];track.x[2]=.10
    for _ in range(2):
        out=tracker.step([],dt=.1)[0]
        assert out[1:3]==last[1:3] and out[3]==0. and out[5]=='block'
    assert track.x[0]>.01  # internal association prediction remains independent


@pytest.mark.parametrize('use_hungarian',[False,True])
def test_size_cost_preserves_different_size_ids_when_nearest_centres_cross(use_hungarian):
    # Exercise both assignment branches. Exhaustive optimum is a local oracle
    # for the small matrices; SciPy itself is not installed on this machine.
    import itertools
    def assignment(cost):
        n,m=cost.shape
        if n<=m:
            columns=min(itertools.permutations(range(m),n),key=lambda p:sum(cost[i,j] for i,j in enumerate(p)))
            return np.arange(n),np.array(columns)
        rows,columns=assignment(cost.T)
        return columns,rows
    with patch.object(kf,'_HAS_SCIPY',use_hungarian),patch.object(kf,'linear_sum_assignment',assignment):
        tracker=GlobalKFTracker()
        tracker.step([detection(0.,.28,.28),detection(.12,1.,.6)],dt=.1)
        out=tracker.step([detection(.04,1.,.6),detection(.08,.28,.28)],dt=.1)
        assert len(out)==2
        assert tracker.tracks[1].last_measurement[0]==.08
        assert tracker.tracks[2].last_measurement[0]==.04


def test_weighted_cost_above_distance_cap_does_not_reject_valid_pair():
    matcher=HungarianMatcher(gate=.5,size_weight=.5)
    matches,unmatched=matcher.match([(0.,0.)],[(.1,0.)],size_differences=np.array([[2.]]))
    assert matches==[(0,0)] and unmatched==[]


def test_swapping_box_axes_has_zero_size_cost():
    tracker=GlobalKFTracker()
    tracker.step([detection(0.,.6,.28)])
    out=tracker.step([detection(.02,.28,.6)])
    assert len(out)==1 and out[0][0]==1


def test_size_changes_do_not_expand_reachability_by_more_than_35cm():
    tracker=GlobalKFTracker(gate=1.)
    tracker.step([detection(0.,.28,.28)],dt=.05)
    out=tracker.step([detection(.60,1.,1.)],dt=.05)
    assert len(out)==2


def test_filtered_and_published_velocity_remain_at_or_below_1m_per_second():
    tracker=GlobalKFTracker()
    for i in range(40):out=tracker.step([detection(i*.12)],dt=.1)
    assert len(out)==1
    assert math.hypot(*tracker.tracks[1].velocity())<=1.+1e-12
    assert out[0][3]<=1.+1e-12


def test_missed_dynamic_output_remains_dynamic_in_planner_adapter():
    tracker=GlobalKFTracker()
    for i in range(30):tracker.step([detection(3.-i*.05)],dt=.1)
    out=tracker.step([],dt=.1)[0]
    ob=obstacle_from_object(SimpleNamespace(x=out[1],y=out[2],length=out[6],width=out[7],
        heading=out[4],shape_heading=out[8],speed=out[3],id=out[0],obstacle_type=out[5]))
    assert not ob.uncertain and ob.speed>.3
    poses=np.zeros((1,41,3));times=np.linspace(0.,2.,41)
    assert minimum_gaps(poses,times,[ob],VehicleGeometry()).item()<0.


def test_ros_yaml_and_tracker_default_parameters_agree():
    import yaml
    cfg=yaml.safe_load((ROOT/'rda_dwa_real/config/real_vehicle.yaml').read_text())['lidar_track_node']['ros__parameters']
    tracker=GlobalKFTracker()
    assert cfg['track_gate']==tracker.gate==.5
    assert cfg['obstacle_max_speed']==tracker.max_speed==1.
    assert cfg['track_lost_ttl']==tracker.lost_ttl==3
    assert cfg['track_size_weight']==tracker._matcher.size_weight==.5
    assert cfg['track_shape_slack_max']==tracker.shape_slack_max==.35
    assert cfg['cluster_min_extent']==.15


def test_unknown_matching_prediction_is_stationary_despite_internal_velocity_estimate():
    tracker=GlobalKFTracker()
    tracker.step([detection(0.)],dt=.1)
    track=tracker.tracks[1];track.x[2]=.8
    with patch.object(tracker._matcher,'match',wraps=tracker._matcher.match) as match:
        tracker.step([detection(.04)],dt=.1)
        assert match.call_args.args[0]==[(0.,0.)]
    assert track.velocity()[0]>.1  # retained for future motion confirmation
    assert track.last_measurement==(.04,0.)


def test_confirmed_dynamic_matching_prediction_uses_internal_cv_position():
    tracker=GlobalKFTracker()
    for i in range(30):tracker.step([detection(i*.05)],dt=.1)
    track=tracker.tracks[1];assert track.moving
    expected=np.array(track.position())+np.array(track.velocity())*.1
    with patch.object(tracker._matcher,'match',wraps=tracker._matcher.match) as match:
        tracker.step([detection(expected[0])],dt=.1)
        np.testing.assert_allclose(match.call_args.args[0],[expected])


def test_static_matching_prediction_stays_at_last_filtered_observation():
    tracker=GlobalKFTracker(static_hold_frames=3)
    for _ in range(5):tracker.step([detection(0.)],dt=.1)
    track=tracker.tracks[1];track.x[2]=.1
    with patch.object(tracker._matcher,'match',wraps=tracker._matcher.match) as match:
        tracker.step([detection(0.)],dt=.1)
        assert match.call_args.args[0]==[(0.,0.)]
