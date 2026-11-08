from types import SimpleNamespace as S
import math
import numpy as np
import pytest

from campus_lidar_perception.types import Detection
from campus_lidar_perception.tracker import MultiObjectTracker, TrackerConfig
from campus_lidar_perception.coordinates import Pose, PoseCache, matrix, mounting, transform_detection
from campus_lidar_perception.pointcloud import read_xyzi
from campus_lidar_perception.geometry import GeometryDetector, GeometryConfig, build_grid


def test_motion_estimation_and_lost_lifecycle():
    tracker = MultiObjectTracker()
    for i in range(60):
        tracks = tracker.step([Detection(x=.05*i, y=2., label='PEDESTRIAN')], 1.+i*.1)
        assert [t['id'] for t in tracks] == [1]
    t = tracks[0]
    assert t['type'] == 'pedestrian'
    assert t['speed'] == pytest.approx(.5, abs=.03)
    assert t['shape_heading'] == 0.
    miss = tracker.step([], 7.)[0]
    assert miss['predicted'] and miss['observation_stamp'] == pytest.approx(6.9)
    assert miss['x'] > t['x']
    assert tracker.step([], 7.5) == []


def test_crossing_different_classes_and_order_changes():
    tracker = MultiObjectTracker()
    for i in range(50):
        a = Detection(x=-2.+i*.08, y=0., label='PEDESTRIAN')
        b = Detection(x=2.-i*.08, y=0., label='BICYCLE', length=1.5)
        rows = tracker.step([a, b] if i % 2 == 0 else [b, a], 1.+i*.1)
        assert len(rows) == 2
        assert rows[0]['semantic_label'] == 'PEDESTRIAN'
        assert rows[1]['semantic_label'] == 'BICYCLE'


def test_unknown_fixed_shape_and_duplicate_frames():
    tracker = MultiObjectTracker(TrackerConfig(min_motion_hits=6))
    for i in range(4):
        t = tracker.step([Detection(x=2.+i*.001, y=0., yaw=math.pi/2)], 1.+i*.1)[0]
        assert t['x'] == pytest.approx(2.+i*.001)
        assert t['speed'] == 0. and t['type'] == 'unknown'
        assert t['length'] == .5 and t['has_shape_heading']
    with pytest.raises(ValueError):
        tracker.step([], 1.3)
    before = tracker.next_id
    tracker.reset()
    assert tracker.step([Detection(3., 0.)], 2.)[0]['id'] == before


def test_association_dummies_keep_unreachable_detection_separate():
    t = MultiObjectTracker()
    t.step([Detection(0., 0.), Detection(2., 0.)], 1.)
    rows = t.step([Detection(2.02, 0.), Detection(40., 0.)], 1.1)
    assert rows[1]['id'] == 2 and rows[1]['hits'] == 2
    assert rows[2]['id'] == 3 and rows[2]['x'] == 40.


def test_moving_vehicle_localization_does_not_create_object_velocity():
    t = MultiObjectTracker()
    for i in range(60):
        ego = matrix(Pose(i*.1, 0., .2))
        world = transform_detection(Detection(5.-i*.1*math.cos(.2), i*.1*math.sin(.2)), ego)
        rows = t.step([world], 1.+i*.1)
    assert rows[0]['speed'] == 0.
    cache = PoseCache()
    assert cache.receive(Pose(1., 2., 0.), 0., 10.)
    assert cache.sample(10.2, .3) is not None
    assert cache.sample(10.4, .3) is None


def test_sensor_extrinsics_and_shape_yaw():
    t = matrix(Pose(10., 20., math.pi/2))@mounting(x=.2, z=.45)
    d = transform_detection(Detection(2., 0., yaw=.3), t)
    assert d.x == pytest.approx(10.) and d.y == pytest.approx(22.2)
    assert d.z == pytest.approx(.45)
    assert d.yaw == pytest.approx(math.pi/2+.3)


def test_pointcloud_padding_endian_and_intensity():
    dtype = np.dtype({'names':['x','y','z','intensity'], 'formats':['>f4','>f4','>f4','u1'],
                      'offsets':[0,4,8,16], 'itemsize':24})
    data = bytearray(64)
    a = np.ndarray((2,1), dtype=dtype, buffer=data, strides=(32,24))
    for k,v in [('x',1.),('y',2.),('z',3.),('intensity',200)]:
        a[k][:] = v
    msg = S(width=1,height=2,point_step=24,row_step=32,is_bigendian=True,data=data,
            fields=[S(name=k,datatype=2 if k=='intensity' else 7,count=1,offset=dtype.fields[k][1]) for k in dtype.names])
    points, indices = read_xyzi(msg)
    np.testing.assert_allclose(points, [[1,2,3,200]]*2)
    assert indices.tolist() == [0,1]
    msg.data = data[:20]
    with pytest.raises(ValueError):read_xyzi(msg)


def test_grid_unknown_and_semantic_intersection():
    cfg = GeometryConfig(max_range=2., resolution=.2, ground_points_per_cell=2)
    points = np.array([[1.,0.,0.,1.],[1.01,0.,0.,1.],[1.5,0.,.1,1.]])
    grid = np.array(build_grid(points, points[:,2], cfg)['data']).reshape(20,20)
    assert grid[10,15] == 0
    assert grid[0,0] == -1
    assert grid[10,17] == 100
    semantic = np.array(build_grid(points,points[:,2],cfg, np.zeros(3,bool))['data']).reshape(20,20)
    assert semantic[10,15] == -1


def test_curbs_unknown_geometry_even_without_neural_boxes():
    xy = np.array([(x,y) for x in np.arange(-4,4,.15) for y in np.arange(-3,3,.15)])
    ground = np.column_stack([xy, np.full(len(xy),-.36),np.ones(len(xy))])
    curb = np.array([[2., y,-.26,10.] for y in np.arange(-1,1,.05)])
    cloud = np.concatenate([ground,curb])
    class GroundFixture:
        # Test fixture is not used as an operational ground backend.
        def indices(self,p):return np.flatnonzero(p[:,2]<-.34)
    g = GeometryDetector(GroundFixture(),GeometryConfig(max_range=5.))
    unknown, grid, diag = g.run(cloud, [], mounting())
    assert any(abs(d.x-2.) < .1 for d in unknown)
    assert 100 in grid['data'] and diag['geometry_obstacle_points'] > 0
    with pytest.raises(ValueError, match='ground_unreliable'):
        g.run(curb, [], mounting())
