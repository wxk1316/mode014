import ast,math,time
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from campus_lidar_perception.pipeline import CloudFrame,PerceptionPipeline,SemanticCache,LatestWorker
from campus_lidar_perception.coordinates import Pose,mounting,rotation,quat_attitude,transform_points,pose_jump
from campus_lidar_perception.geometry import GeometryDetector,GeometryConfig,PatchworkGround,self_mask
from campus_lidar_perception.types import Detection
from campus_lidar_perception.centerpoint_reference import pillar_features

ROOT=Path(__file__).resolve().parents[1]

def _node_method(name):
    source=ROOT/'campus_lidar_perception/node.py';tree=ast.parse(source.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name==name)
    class TFError(Exception):pass
    namespace=dict(math=math,np=np,time=time,Pose=Pose,quat_attitude=quat_attitude,rotation=rotation,
                   TransformException=TFError,Time=lambda:None,Rotation=Rotation)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace);return namespace[name]

def test_nonfinite_tf_cannot_publish_empty_obstacles():
    tf=S(header=S(stamp=S(sec=1,nanosec=0)),transform=S(translation=S(x=float('nan'),y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    node=S(parameters=dict(localization_source='tf',localization_parent_frame='map',localization_child_frame='body',loc_timeout=.35,body_to_base_link_x=0.),
           pose_cache=S(sample=lambda *a:None,source_stamp=None),tf_buffer=S(lookup_transform=lambda *a:tf),
           get_clock=lambda:S(now=lambda:S(nanoseconds=1100000000)))
    assert _node_method('_pose')(node) is None
    node.parameters.update(sensor_frame='rslidar',base_frame='base_link');node.fallback_mount=mounting()
    with pytest.raises(ValueError):_node_method('_mount')(node,'rslidar')
    class Detector:
        def predict(self,*args):pytest.fail('invalid frame reached inference')
    frame=CloudFrame(1,1.,0.,np.array([[5.,0.,-.36,1.]]),Pose(float('nan'),0.,0.),mounting())
    with pytest.raises(ValueError):PerceptionPipeline(Detector(),None).process(frame)

def test_semantic_xyz_rejects_wrong_height():
    cache=SemanticCache();old=CloudFrame(1,1.,0.,np.array([[5.,0.,-.36,1.]]),Pose(0.,0.,0.),mounting())
    cache.update(old,np.array([10]),np.array([.99]))
    new=CloudFrame(2,1.1,0.,np.array([[5.,0.,.5,1.]]),Pose(0.,0.,0.),mounting())
    mask,diag=cache.road_mask(new);assert not mask.any() and not diag['semantic_available']

def test_large_map_preserves_subcell_precision():
    points=np.array([[.012,.013,0.,10.]],np.float32);mapped=transform_points(points,mounting(x=6000000.,z=0.))
    assert mapped.dtype==np.float64 and mapped[0,0]-6000000.==pytest.approx(.012,abs=1e-7)

def test_real_patchwork_vertical_pole_is_planner_visible():
    xy=np.array([(x,y) for x in np.arange(-6,6,.15) for y in np.arange(-5,5,.15)])
    road=np.column_stack((xy,np.full(len(xy),-.36),np.full(len(xy),60.)))
    pole=np.array([[2.,0.,i*.12,60.] for i in range(10)])
    detector=GeometryDetector(PatchworkGround(.36),GeometryConfig(max_range=15.))
    objects,grid,diag=detector.run(np.concatenate((road,pole)),[],mounting())
    assert diag['geometry_obstacle_points']>=10 and any(abs(d.x-2.)<.1 and d.height>=1. for d in objects)
    assert 100 in grid['data']

def test_neural_box_blocks_ground_beneath_it():
    xy=np.array([(x,y) for x in np.arange(-6,6,.15) for y in np.arange(-5,5,.15)])
    road=np.column_stack((xy,np.full(len(xy),-.36),np.full(len(xy),60.)))
    detector=GeometryDetector(PatchworkGround(.36),GeometryConfig(max_range=10.))
    _,grid,_=detector.run(road,[Detection(3.,0.,.3,2.,1.,1.,label='CAR')],mounting())
    assert np.array(grid['data']).reshape(grid['height'],grid['width'])[50,65]==100

def test_self_filter_retains_overhead_structure():
    assert self_mask(np.array([[.1,.1,.2,1.],[.1,.1,1.2,1.]]),GeometryConfig()).tolist()==[True,False]

def test_cpu_upper_boundary_matches_cuda_clamp():
    x=np.nextafter(np.float32(76.8),np.float32(0));f,cells=pillar_features([[x,0.,0.,0.]])
    assert cells.tolist()==[240*480+479] and f[0,0,7]<.17

def test_harp_nonfinite_logits_fail():
    import torch
    from campus_lidar_perception.harp_backend import HarpNeXtBackend
    backend=object.__new__(HarpNeXtBackend);backend.torch=torch;backend.device=torch.device('cpu');backend.fp16=False
    backend.model=lambda inputs,training:dict(seg_logits=torch.full((len(inputs['voxels']['voxels']),16),float('nan')))
    with pytest.raises(ValueError,match='logits'):backend.predict(np.array([[5.,0.,0.,60.],[5.,.2,-.2,30.]],np.float32))

def test_worker_uses_configured_age():
    worker=LatestWorker(lambda:lambda frame:dict(processed=True),'age_test',max_source_age=1.)
    frame=CloudFrame(1,1.,time.monotonic(),np.array([[5.,0.,0.,1.]]),Pose(0,0,0),mounting(),source_age_at_receive=.5)
    try:
        worker.submit(frame);deadline=time.monotonic()+2.;result=None
        while result is None and time.monotonic()<deadline:result=worker.poll();time.sleep(.005)
        assert result and result[1]==dict(processed=True) and result[2] is None
    finally:worker.close()

def test_pure_orientation_relocalization_is_rejected_before_publication():
    source=ROOT/'campus_lidar_perception/node.py';tree=ast.parse(source.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='_poll')
    namespace=dict(time=time,np=np,pose_jump=pose_jump)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    frame=CloudFrame(1,1.,time.monotonic(),np.array([[5.,0.,0.,1.]]),Pose(0,0,0),mounting())
    events=[]
    node=S(semantic_worker=None,worker=S(poll=lambda:(frame,dict(tracks=[]),None)),parameters=dict(max_source_age=.4),
           _pose=lambda:Pose(0,0,np.pi),_diag=lambda reason,**kw:events.append(reason),
           obstacle_pub=S(publish=lambda *args:pytest.fail('stale orientation was published')))
    namespace['_poll'](node)
    assert events==['localization_jump_during_inference']
