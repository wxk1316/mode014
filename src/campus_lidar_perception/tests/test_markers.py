from types import SimpleNamespace as S
import math
import pytest
from campus_lidar_perception.markers import TrackMarkers

class Marker:
    CUBE=1;TEXT_VIEW_FACING=9;ARROW=0;LINE_STRIP=4;ADD=0;DELETE=2;DELETEALL=3
    def __init__(self):
        self.header=S(frame_id='',stamp=S(sec=0,nanosec=0));self.pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=0.))
        self.scale=S(x=0.,y=0.,z=0.);self.color=S(r=0.,g=0.,b=0.,a=0.);self.lifetime=S(sec=0,nanosec=0)

def row():return dict(id=1,x=2.,y=3.,z=.5,length=1.,width=.5,height=1.,shape_heading=math.pi/2,
                      vx=1.,vy=.5,speed=math.hypot(1,.5),moving=True,semantic_label='PEDESTRIAN',
                      active_filter='imm',filter_model_probabilities=[.3,.7])

def test_marker_types_and_namespace_deletion():
    renderer=TrackMarkers();data=dict(stamp=1.,stamp_ns=1000000001,tracks=[row()])
    first=renderer.render(data,Marker,lambda:S(markers=[]),S)
    assert {m.type for m in first.markers}=={Marker.CUBE,Marker.TEXT_VIEW_FACING,Marker.ARROW}
    arrow=next(m for m in first.markers if m.type==Marker.ARROW)
    assert arrow.points[1].x-arrow.points[0].x==pytest.approx(1.)
    assert arrow.points[1].y-arrow.points[0].y==pytest.approx(.5)
    text=next(m for m in first.markers if m.type==Marker.TEXT_VIEW_FACING)
    assert 'IMM' in text.text and 'CA=0.70' in text.text
    data['stamp_ns']=1100000001;data['tracks'][0]['x']=2.1
    second=renderer.render(data,Marker,lambda:S(markers=[]),S)
    assert any(m.type==Marker.LINE_STRIP for m in second.markers)
    empty=renderer.render(dict(stamp=1.2,tracks=[]),Marker,lambda:S(markers=[]),S)
    assert len(empty.markers)==4 and all(m.action==Marker.DELETE for m in empty.markers)
    assert all(m.action!=Marker.DELETEALL for m in first.markers+second.markers+empty.markers)
