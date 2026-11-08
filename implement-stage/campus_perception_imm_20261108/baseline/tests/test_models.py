from pathlib import Path
import numpy as np
import pytest

from campus_lidar_perception.centerpoint_reference import pillar_features, decode_head, rotated_iou, nms
from campus_lidar_perception.harp_backend import project, interpolate, HarpNeXtBackend
from campus_lidar_perception.types import Detection

ROOT = Path(__file__).resolve().parents[1]


def test_centerpoint_xyzt_layout_and_padding():
    points = np.array([[.01,.02,-.3,0.],[.04,.03,-.2,.1],[200,0,0,0]],np.float32)
    f,cells = pillar_features(points)
    assert f.shape == (1,32,9)
    np.testing.assert_allclose(f[0,:2,:4],points[:2])
    np.testing.assert_allclose(f[0,:2,4:7].sum(axis=0),0.,atol=1e-6)
    assert not f[0,2:].any()
    assert cells[0] == 240*480+240


def test_centerpoint_decode_dimension_order_and_yaw():
    shape = lambda c:np.zeros((1,c,3,3),np.float32)
    outputs = {k:shape(c) for k,c in [('heatmap',5),('reg',2),('height',1),('dim',3),('rot',2)]}
    outputs['heatmap'].fill(-20)
    outputs['heatmap'][0,0,1,2] = 4.
    outputs['dim'][0,:,1,2] = np.log([2.,4.,1.5])
    outputs['rot'][0,:,1,2] = [1.,0.]
    d = decode_head(outputs)[0]
    assert d.label == 'CAR' and d.length == pytest.approx(4.)
    assert d.width == pytest.approx(2.) and d.yaw == pytest.approx(-np.pi)
    assert d.x == pytest.approx(2*.32-76.8)


def test_rotated_nms_retains_distinct_adjacent_classes():
    a = Detection(0,0,length=4,width=2,label='CAR',score=.9)
    b = Detection(.1,0,length=4,width=2,label='CAR',score=.8)
    assert rotated_iou(a,a) == pytest.approx(1.)
    assert rotated_iou(a,b) > .9
    assert len(nms([a,b])) == 1
    b.label = 'PEDESTRIAN'
    assert len(nms([a,b])) == 2


def test_harp_fov_keeps_helios_lower_returns_unknown():
    angle = np.deg2rad(np.array([-50,-20,0,14]))
    points = np.column_stack((np.cos(angle),np.zeros(4),np.sin(angle),np.ones(4)))
    coords, valid = project(points)
    assert valid.tolist() == [False,True,True,False]
    assert coords.shape == (4,3)


def test_interpolation_preserves_measured_order():
    points = np.array([[1.,0.,0.,10.],[0.,1.,0.,20.]],np.float32)
    out = interpolate(points)
    np.testing.assert_array_equal(out[:len(points)],points)


def test_downloaded_onnx_graphs_are_valid():
    import onnx
    for file in (ROOT/'models/centerpoint_base').glob('*.onnx'):
        onnx.checker.check_model(onnx.load(file))
    assert len(list((ROOT/'models/centerpoint_base').glob('*.onnx'))) == 2


def test_real_checkpoint_strict_load_and_cpu_forward():
    import torch
    torch.set_num_threads(2)
    backend = HarpNeXtBackend(ROOT/'models/harpnext_nuscenes',device='cpu')
    assert backend.parameter_count > 5000000
    points = np.array([[5.,0.,0.,50.],[5.,.2,-.2,80.],[-5.,1.,-.3,30.],
                       [2.,1.,-4.,20.]],np.float32)
    labels,score = backend.predict(points)
    assert labels.shape == score.shape == (4,)
    assert labels[-1] == -1 and score[-1] == 0.
    assert np.all((labels[:3]>=0)&(labels[:3]<16))
    assert np.isfinite(score).all()
