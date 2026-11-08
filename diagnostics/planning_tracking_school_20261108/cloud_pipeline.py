"""Synthetic sensor data; perception and association are production algorithms.
Dense object surfaces are idealized returns, without ray occlusion simulation.
"""
import math,time
from dataclasses import fields
import numpy as np
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams
from rda_dwa_real.kf_tracker import GlobalKFTracker

def school_scenes(existing,Box):
    selected=[c for c in existing if c['name'] in ('open','campus_bollards','campus_bench','campus_bins','campus_planter','campus_chicane','campus_wall_gap','campus_full_block','u_window','u_exit_only','u_spin_blocked','pedestrian_0.4','car_0.5')]
    for c in selected:
        if c['name']=='campus_full_block':c['blocked']=True
    selected.extend([
      dict(name='school_tree_cone',objects=[Box(2.,.4,.4,.4,ident=1),Box(3.6,-.5,.28,.28,ident=2)],goal=(5.,0.),expected='Avoid tree and tapered cone with full body clearance.'),
      dict(name='school_corner',objects=[Box(2.5,.8,2.,.15,ident=1),Box(3.5,.65,.15,1.7,ident=2)],goal=(5.,0.),expected='Pass outside L-shaped wall without occupying its cavity.'),
      dict(name='school_crossing',objects=[Box(2.5,-1.8,.45,.45,vy=.5,ident=1)],goal=(5.,0.),expected='Yield safely or pass with 12cm minimum clearance; no false trap.')])
    return selected

class CloudPipeline:
    def __init__(self,cfg,case,noise,seed):
        p=cfg['lidar_track_node']['ros__parameters'];self.case=case;self.noise=noise;self.rng=np.random.default_rng(seed+701)
        params=PerceptionParams()
        aliases={'max_obstacle_range':'max_range','pass_y_min':'y_min','pass_y_max':'y_max','ground_filter_enable':'ground_enabled','obstacle_height_min':'height_min','obstacle_height_max':'height_max'}
        for key,val in p.items():
            target=aliases.get(key,key)
            if hasattr(params,target):setattr(params,target,val)
        self.core=LidarPerceptionCore(params);self.params=params;self.durations=[];self.last=None
        self.kf=GlobalKFTracker(dt=.1,gate=p['track_gate'],lost_ttl=p['track_lost_ttl'],q=p['kf_q'],r=p['kf_r'],max_speed=p['obstacle_max_speed'],dyn_speed_thr=p['dyn_speed_thr'],dyn_hold_frames=p['dyn_hold_frames'],static_speed_thr=p['static_speed_thr'],static_hold_frames=p['static_hold_frames'],static_small_thr=p['static_small_thr'],position_slack=p['track_position_slack'],size_weight=p['track_size_weight'],shape_slack_max=p['track_shape_slack_max'])
        self.templates=[]
        for o in case['objects']:
            levels=(.08,.18,.30,.40,.49) if case['name']=='school_tree_cone' and o.ident==2 else (.08,.25,.5,.9,1.2)
            parts=[]
            for h in levels:
                shrink=max(.02,1-h/.5) if case['name']=='school_tree_cone' and o.ident==2 else 1.
                l,w=o.length*shrink,o.width*shrink
                nx=max(3,int(math.ceil(o.length/.075))+1);ny=max(3,int(math.ceil(o.width/.075))+1)
                perimeter=np.vstack([np.c_[np.linspace(-l/2,l/2,nx),np.full(nx,-w/2)],np.c_[np.linspace(-l/2,l/2,nx),np.full(nx,w/2)],np.c_[np.full(ny,-l/2),np.linspace(-w/2,w/2,ny)],np.c_[np.full(ny,l/2),np.linspace(-w/2,w/2,ny)]])
                c,s=math.cos(o.heading),math.sin(o.heading);perimeter=perimeter@np.array([[c,s],[-s,c]])+[o.x,o.y]
                parts.append(np.c_[perimeter,np.full(len(perimeter),params.ground_reference_z+h)])
            self.templates.append((o,np.vstack(parts)))
        gx,gy=np.meshgrid(np.arange(-12.,14.,.20),np.arange(-2.,2.01,.20))
        ground=np.c_[gx.ravel(),gy.ravel(),np.full(gx.size,params.ground_reference_z)]
        # 10cm raised curb strips: road inner edges remain y=+/-2m.
        sx,sy=np.meshgrid(np.arange(-12.,14.,.08),[-2.10,-2.05,-2.,2.,2.05,2.10])
        curb=np.c_[sx.ravel(),sy.ravel(),np.full(sx.size,params.ground_reference_z+.1)]
        self.environment=np.vstack([ground,curb])
    def process(self,t,pose):
        xyz=np.vstack([self.environment]+[points+[o.vx*t,o.vy*t,0.] for o,points in self.templates])
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        dx=xyz[:,0]-x;dy=xyz[:,1]-y
        xyz[:,0]=c*dx+s*dy;xyz[:,1]=-s*dx+c*dy;xyz[:,2]-=self.params.lidar_height
        if self.noise:
            xyz+=self.rng.normal(0.,.005,xyz.shape)
            xyz=xyz[self.rng.random(len(xyz))>.03]
        start=time.perf_counter();dets,m=self.core.process(xyz,100.+t,pose);duration=time.perf_counter()-start
        self.durations.append(duration)
        world=[(x+c*d[0]-s*d[1],y+s*d[0]+c*d[1],d[2],d[3],d[4],d[5]+yaw) for d in dets]
        # drivable_grid is not consumed by the present planner; retain summary.
        m=dict(m);m.pop('drivable_grid',None)
        if t==0:m['initial_detections_world']=world
        return world,m,duration
    def track(self,dets,stamp):
        dt=.1 if self.last is None else stamp-self.last;self.last=stamp
        return self.kf.step(dets,dt=dt)
