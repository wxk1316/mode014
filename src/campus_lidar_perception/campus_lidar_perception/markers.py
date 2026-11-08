"""MarkerArray rendering with isolated namespaces and bounded track history."""
from collections import deque
import math


class TrackMarkers:
    namespaces=('campus_tracks/boxes','campus_tracks/labels','campus_tracks/velocity','campus_tracks/history')
    def __init__(self,history_length=20,velocity_horizon=1.,lifetime=.4):
        if history_length<2 or velocity_horizon<=0 or lifetime<=0:raise ValueError('invalid marker configuration')
        self.history_length,self.velocity_horizon,self.lifetime=history_length,velocity_horizon,lifetime
        self.histories={};self.last_stamp=None;self.previous=set()

    def render(self,result,marker_factory,array_factory,point_factory):
        message=array_factory();current=set()
        stamp_ns=result.get('stamp_ns',round(result['stamp']*1e9))
        live_ids={row['id'] for row in result['tracks']}
        self.histories={k:v for k,v in self.histories.items() if k in live_ids}
        def marker(ns,tid,typ):
            item=marker_factory();item.header.frame_id='map'
            item.header.stamp.sec,item.header.stamp.nanosec=divmod(stamp_ns,1000000000)
            item.ns=ns;item.id=int(tid);item.type=typ;item.action=item.ADD
            item.pose.orientation.w=1.
            item.lifetime.sec=int(self.lifetime);item.lifetime.nanosec=round((self.lifetime-int(self.lifetime))*1e9)
            current.add((ns,tid));return item
        fresh=self.last_stamp is None or stamp_ns>self.last_stamp
        for row in result['tracks']:
            tid=row['id'];xyz=[float(row['x']),float(row['y']),float(row['z'])]
            if not all(math.isfinite(v) for v in xyz):raise ValueError('non-finite marker position')
            history=self.histories.setdefault(tid,deque(maxlen=self.history_length))
            if fresh:history.append(xyz)
            color=(.5,.5,.5) if row.get('predicted') else ((.15,.95,.3) if row.get('moving') else (.95,.55,.15))
            box=marker(self.namespaces[0],tid,marker_factory.CUBE)
            box.pose.position.x,box.pose.position.y,box.pose.position.z=xyz
            box.pose.orientation.z=math.sin(row['shape_heading']/2);box.pose.orientation.w=math.cos(row['shape_heading']/2)
            box.scale.x,box.scale.y,box.scale.z=float(row['length']),float(row['width']),max(.1,float(row['height']))
            box.color.r,box.color.g,box.color.b=color;box.color.a=.45;message.markers.append(box)
            label=marker(self.namespaces[1],tid,marker_factory.TEXT_VIEW_FACING)
            label.pose.position.x,label.pose.position.y=xyz[:2];label.pose.position.z=xyz[2]+row['height']/2+.25
            label.scale.z=.22;label.color.r=label.color.g=label.color.b=label.color.a=1.
            mode=row.get('active_filter','cv').upper()
            probabilities=row.get('filter_model_probabilities',[1.])
            detail=f" CV={probabilities[0]:.2f} CA={probabilities[1]:.2f}" if mode=='IMM' else ''
            label.text=f"#{tid} {row.get('semantic_label','UNKNOWN')} {row['speed']:.2f} m/s\n{mode}{detail} age={row.get('observation_age',0.):.2f}s"
            if row.get('predicted'):label.text+=' PREDICTED'
            message.markers.append(label)
            if row['speed']>.02:
                arrow=marker(self.namespaces[2],tid,marker_factory.ARROW)
                end=[xyz[0]+row['vx']*self.velocity_horizon,xyz[1]+row['vy']*self.velocity_horizon,xyz[2]]
                arrow.points=[point_factory(x=p[0],y=p[1],z=p[2]) for p in (xyz,end)]
                arrow.scale.x=.06;arrow.scale.y=.13;arrow.scale.z=.2
                arrow.color.r,arrow.color.g,arrow.color.b=color;arrow.color.a=1.;message.markers.append(arrow)
            if len(history)>1:
                line=marker(self.namespaces[3],tid,marker_factory.LINE_STRIP)
                line.points=[point_factory(x=p[0],y=p[1],z=p[2]) for p in history]
                line.scale.x=.035;line.color.r,line.color.g,line.color.b=color;line.color.a=.9;message.markers.append(line)
        for ns,tid in sorted(self.previous-current):
            item=marker_factory();item.header.frame_id='map';item.ns=ns;item.id=int(tid);item.action=item.DELETE
            item.header.stamp.sec,item.header.stamp.nanosec=divmod(stamp_ns,1000000000)
            message.markers.append(item)
        self.previous=current;self.last_stamp=stamp_ns
        return message
