import ast
import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest

from campus_lidar_perception.interfaces import obstacle_array,PLANNER_FIELDS
from campus_lidar_perception.tracker import MultiObjectTracker
from campus_lidar_perception.types import Detection

ROOT=Path(__file__).resolve().parents[2]


def array_factory():return S(header=S(frame_id='',stamp=S(sec=0,nanosec=0)),obstacles=[])


def test_planner_message_schema_and_original_nanoseconds():
    tracker=MultiObjectTracker()
    tracks=tracker.step([Detection(5.,2.,label='CAR',length=4.,width=2.,yaw=.6)],1.)
    ns=1794153600*1000000000+181276302
    message=obstacle_array(dict(stamp=ns/1e9,stamp_ns=ns,tracks=tracks),array_factory,S)
    assert message.header.frame_id=='map'
    assert message.header.stamp.sec*1000000000+message.header.stamp.nanosec==ns
    assert set(vars(message.obstacles[0]))==set(PLANNER_FIELDS)
    schema=(ROOT/'rda_dwa_real_msgs/msg/Obstacle.msg').read_text()
    fields={line.split()[1] for line in schema.splitlines() if line and not line.startswith('#')}
    assert set(PLANNER_FIELDS)==fields


def test_actual_existing_planner_callback_accepts_new_tracks():
    # Execute the current consumer method from the untouched source, rather than
    # restating its checks in this test. ROS transport is outside this CPU check.
    source=ROOT/'rda_dwa_real/rda_dwa_real/traj_tracker_node.py'
    tree=ast.parse(source.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
                if isinstance(n,ast.FunctionDef) and n.name=='_cb_obstacles')
    import math
    ns={'math':math,'TrackedObstacle':lambda *args:args}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
    calls=[]
    core=S(set_obstacles=lambda *a,**k:calls.append((a,k)),set_safety=lambda r:pytest.fail(r))
    node=S(core=core,_now=lambda:10.,get_clock=lambda:S(now=lambda:S(nanoseconds=1500000000)))
    track=MultiObjectTracker().step([Detection(2.,1.,label='CAR',yaw=.4)],1.)
    message=obstacle_array(dict(stamp=1.,tracks=track),array_factory,S)
    ns['_cb_obstacles'](node,message)
    assert calls and calls[0][1]['stamp']==1.


def _launch_stubs(monkeypatch):
    class Item:
        def __init__(self,*args,**kwargs):self.args=args;self.kwargs=kwargs
    class Description:
        def __init__(self,items):self.entities=items
    definitions={
        'ament_index_python.packages':{'get_package_share_directory':lambda name:str(ROOT/name)},
        'launch':{'LaunchDescription':Description},
        'launch.actions':{'DeclareLaunchArgument':Item,'IncludeLaunchDescription':Item},
        'launch.conditions':{'IfCondition':Item},
        'launch.launch_description_sources':{'PythonLaunchDescriptionSource':Item},
        'launch.substitutions':{'LaunchConfiguration':Item,'PathJoinSubstitution':Item},
        'launch_ros.actions':{'Node':Item,'SetParameter':Item},
        'launch_ros.parameter_descriptions':{'ParameterValue':Item},
    }
    for name,attributes in definitions.items():
        module=types.ModuleType(name)
        for k,v in attributes.items():setattr(module,k,v)
        monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.delitem(sys.modules,'campus_lidar_perception.launch_support',raising=False)
    return importlib.import_module('campus_lidar_perception.launch_support')


def test_full_launch_excludes_old_detectors_and_has_planning_control(monkeypatch):
    support=_launch_stubs(monkeypatch)
    launch=support.combined_launch(True)
    nodes=[v.kwargs for v in launch.entities if 'executable' in getattr(v,'kwargs',{})]
    names={n['executable'] for n in nodes}
    assert {'campus_perception_node','fastlio_mapping','global_localization.py','transform_fusion.py',
            'bridge_node','traj_tracker_node','car_ctr'}<=names
    assert not {'lidar_track_node','cloud_seg','obs_check_inlane'}&names
    assert sum(n['executable']=='campus_perception_node' for n in nodes)==1
    declared=[v.args[0] for v in launch.entities if getattr(v,'args',()) and isinstance(v.args[0],str)]
    assert len(declared)==len(set(declared))
    assert {'profile','fastlio_map_dir','model_dir','obstacles_topic','planner_config'}<=set(declared)


def test_standalone_launch_scopes(monkeypatch):
    support=_launch_stubs(monkeypatch)
    detection=support.detection_launch()
    assert [v.kwargs['executable'] for v in detection.entities if 'executable' in getattr(v,'kwargs',{})]==['campus_perception_node','campus_perception_record']
    loc=support.localization_launch()
    assert 'campus_perception_node' not in [v.kwargs['executable'] for v in loc.entities if 'executable' in getattr(v,'kwargs',{})]


def test_real_patchwork_flat_road():
    from campus_lidar_perception.geometry import PatchworkGround,GeometryDetector,GeometryConfig
    from campus_lidar_perception.coordinates import mounting
    xy=np.array([(x,y) for x in np.arange(-5,5,.15) for y in np.arange(-4,4,.15)])
    points=np.column_stack((xy,np.full(len(xy),-.36),np.full(len(xy),60.)))
    backend=GeometryDetector(PatchworkGround(.36),GeometryConfig(max_range=10.))
    unknown,grid,diag=backend.run(points,[],mounting())
    assert diag['ground_points']>100 and not unknown and grid['data'].count(0)>100


def test_dynamic_unknown_remains_unknown_semantic_but_passes_velocity():
    tracker=MultiObjectTracker()
    for i in range(50):rows=tracker.step([Detection(.05*i,2.)],1.+i*.1)
    row=rows[0]
    assert row['semantic_label']=='UNKNOWN' and row['type']=='pedestrian'
    assert row['dynamic_unknown_proxy'] and row['speed']>.4


def test_actual_poll_reports_successful_semantic_timing():
    import json
    source=ROOT/'campus_lidar_perception/campus_lidar_perception/node.py'
    tree=ast.parse(source.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
                if isinstance(n,ast.FunctionDef) and n.name=='_poll')
    ns={}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
    events=[]
    node=S(semantic_worker=S(poll=lambda:(S(stamp=1.),dict(prediction_s=.071,duration_s=.075,
           scatter_backend='torch_native',classified_points=100),None)),worker=S(poll=lambda:None),
           _diag=lambda reason,**values:events.append(dict(reason=reason,**values)))
    ns['_poll'](node)
    assert events[0]['semantic_prediction_s']==.071
    assert events[0]['semantic_processing_s']==.075


def test_declared_launch_mount_fields_are_shared_with_tf(monkeypatch):
    support=_launch_stubs(monkeypatch)
    launch=support.combined_launch(False)
    tf=next(v for v in launch.entities if getattr(v,'kwargs',{}).get('name')=='campus_base_to_lidar')
    args=tf.kwargs['arguments']
    for flag,name in [('--x','sensor_x'),('--y','sensor_y'),('--z','sensor_z'),
                      ('--roll','sensor_roll'),('--pitch','sensor_pitch'),('--yaw','sensor_yaw')]:
        assert args[args.index(flag)+1].args== (name,)


def test_independent_detection_launch_does_not_lookup_car_launch(monkeypatch):
    support=_launch_stubs(monkeypatch)
    def share(name):
        assert name!='car_launch'
        return str(ROOT/name)
    monkeypatch.setattr(support,'get_package_share_directory',share)
    support.detection_launch()
