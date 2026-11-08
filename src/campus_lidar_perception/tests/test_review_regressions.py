import numpy as np
import pytest
from campus_lidar_perception.coordinates import Pose,matrix,mounting,transform_points
from campus_lidar_perception.pipeline import CloudFrame,SemanticCache
from campus_lidar_perception.tracker import MultiObjectTracker
from campus_lidar_perception.types import Detection


def test_roll_pitch_retained_in_pose_transform():
    transform=matrix(Pose(0.,0.,0.,0.,0.,np.pi/6))
    points=np.array([[1.,0.,0.,10.]])
    result=transform_points(points,transform)
    np.testing.assert_allclose(result[0,:3],[np.cos(np.pi/6),0.,-.5],atol=1e-9)


def test_no_spatial_matches_is_not_semantic_available():
    cache=SemanticCache()
    a=CloudFrame(1,1.,0.,np.array([[5.,0.,0.,1.]]),Pose(0,0,0),mounting())
    cache.update(a,np.array([10]),np.array([.9]))
    b=CloudFrame(2,1.1,0.,np.array([[10.,0.,0.,1.]]),Pose(0,0,0),mounting())
    mask,diag=cache.road_mask(b)
    assert not mask.any() and not diag['semantic_available']
    assert diag['semantic_cache_available'] and diag['semantic_matched_fraction']==0.


def test_unknown_updates_do_not_claim_geometry_score_as_fresh_semantics():
    tracker=MultiObjectTracker()
    tracker.step([Detection(2.,0.,label='CAR',score=.4,source='centerpoint')],1.)
    row=tracker.step([Detection(2.,0.,source='geometry')],1.1)[0]
    assert row['semantic_label']=='CAR' and row['score']==1.
    assert row['semantic_score']==.4 and row['semantic_remembered']
    assert row['semantic_observation_stamp']==1.
    for i in range(1,20):row=tracker.step([Detection(2.,0.,source='geometry')],1.1+i*.1)[0]
    assert row['semantic_label']=='UNKNOWN'
