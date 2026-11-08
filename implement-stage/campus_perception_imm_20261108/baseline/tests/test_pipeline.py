import numpy as np
import pytest
from campus_lidar_perception.pipeline import CloudFrame,PerceptionPipeline,SemanticCache
from campus_lidar_perception.coordinates import Pose,mounting
from campus_lidar_perception.types import Detection


def test_ego_sweeps_have_time_not_intensity():
    inputs = []
    class TestDetector:
        # Fixtures substitute inference solely within unit tests.
        def predict(self,p):inputs.append(p.copy());return [],{}
    class TestGeometry:
        def run(self,*a):return [],dict(data=[-1]),{}
    pipe = PerceptionPipeline(TestDetector(),TestGeometry())
    p = np.array([[5.,0.,0.,200.]],np.float32)
    pipe.process(CloudFrame(1,1.,0.,p,Pose(0.,0.,0.),mounting()))
    pipe.process(CloudFrame(2,1.1,0.,p,Pose(.1,0.,0.),mounting()))
    np.testing.assert_allclose(inputs[-1][:,3],[0.,.1],atol=1e-6)
    assert inputs[-1][1,0] == pytest.approx(4.9)


def test_semantic_cache_is_world_aligned_and_expires():
    cache = SemanticCache()
    p = np.array([[5.,0.,0.,100.]],np.float32)
    a = CloudFrame(1,1.,0.,p,Pose(0.,0.,0.),mounting())
    cache.update(a,np.array([10]),np.array([.9]))
    b = CloudFrame(2,1.1,0.,p-np.array([[.1,0.,0.,0.]]),Pose(.1,0.,0.),mounting())
    assert cache.road_mask(b)[0].tolist() == [True]
    b.stamp = 1.5
    assert not cache.road_mask(b)[0].any()
    cache.clear(2.)
    assert not cache.update(a,np.array([10]),np.array([.9]))


def test_localization_jump_resets_tracks_without_reusing_ids():
    class TestDetector:
        def predict(self,p):return [Detection(5.,0.,label='CAR')],{}
    class TestGeometry:
        def run(self,*a):return [],dict(data=[-1]),{}
    pipe = PerceptionPipeline(TestDetector(),TestGeometry())
    p = np.array([[5.,0.,0.,100.]],np.float32)
    first = pipe.process(CloudFrame(1,1.,0.,p,Pose(0,0,0),mounting()))
    second = pipe.process(CloudFrame(2,1.1,0.,p,Pose(20,0,0),mounting()))
    assert second['tracking_reset'] and first['tracks'][0]['id'] != second['tracks'][0]['id']
    assert second['tracks'][0]['speed'] == 0.
