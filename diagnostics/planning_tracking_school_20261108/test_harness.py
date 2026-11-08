"""Independent analytic checks for the audit plant and collision oracle."""
import math,textwrap
from pathlib import Path
import numpy as np
from run_suite import polygon_gap,corners,CONFIG,scenes,TrackedObstacle
from cloud_pipeline import CloudPipeline

def test_ack_substep_against_circular_motion():
    # Execute the actual plant substep text used by the campaign.
    source=Path(__file__).with_name('run_suite.py').read_text()
    start=source.index("   desired=last['speed'] if active")
    end=source.index('   body=corners',start)
    context=dict(last=dict(speed=.5,angle=20.),active=True,mode=0,v=.5,delta=math.radians(20),omega=0.,x=.345,y=0.,yaw=0.,math=math)
    exec(textwrap.dedent(source[start:end]),context)
    rate=.5*math.tan(math.radians(20))/.69;angle=rate*.01;radius=.5/rate
    assert abs(context['yaw']-angle)<1e-12
    assert abs(context['x']-(radius*math.sin(angle)+.345*math.cos(angle)))<1e-8
    assert abs(context['y']-(radius*(1-math.cos(angle))+.345*math.sin(angle)))<1e-8

def test_oracle_clearance_and_overlap():
    car=corners(0.,0.,0.,1.2,.8)
    assert abs(polygon_gap(car,corners(1.42,0.,0.,1.2,.8))-.22)<1e-12
    assert polygon_gap(car,corners(.7,0.,0.,1.2,.8))<0
    assert abs(polygon_gap(car,corners(0.,1.02,0.,1.2,.8))-.22)<1e-12

def test_cloud_transformation_and_ground_anchor():
    case=dict(name='test',objects=[TrackedObstacle(2.,.5,.6,.4,ident=1)],goal=(5.,0.))
    for pose in ((0.,0.,0.),(.5,.1,.7)):
        pipe=CloudPipeline(CONFIG,case,False,31);dets,m,_=pipe.process(0.,pose)
        assert m['valid'],m
        candidates=[q for q in dets if abs(q[0]-2.)<.2 and abs(q[1]-.5)<.2]
        assert len(candidates)==1
        assert np.allclose(candidates[0][:2],(2.,.5),atol=.02)
        assert abs(m['ground_plane'][2]-CONFIG['lidar_track_node']['ros__parameters']['ground_reference_z'])<1e-8
