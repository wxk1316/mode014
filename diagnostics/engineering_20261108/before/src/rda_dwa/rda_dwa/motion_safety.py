"""Physical 1.2 x 0.8 m rectangles and bounded, time-indexed motion checks.

All poses in this module are GEOMETRIC CENTRES. Safety clearance is separate
from physical dimensions. Circular radii are used only in conservative bounds.
"""
from dataclasses import dataclass
import math
import numpy as np
from .collision_checker import _rect_corners_vec, _rect_rect_gaps_vec

@dataclass(frozen=True)
class VehicleGeometry:
    length: float = 1.2
    width: float = .8
    wheelbase: float = .69
    @property
    def rear_offset(self): return self.wheelbase / 2.
    @property
    def radius(self): return math.hypot(self.length/2, self.width/2)

@dataclass(frozen=True)
class TrackedObstacle:
    x: float
    y: float
    length: float
    width: float
    heading: float = 0.
    vx: float = 0.
    vy: float = 0.
    ident: int = 0
    uncertain: bool = False
    uncertainty_speed: float = 0.
    @property
    def speed(self): return math.hypot(self.vx,self.vy)


def obstacle_from_object(o):
    typ=str(getattr(o,'obstacle_type','static_large'))
    speed=float(getattr(o,'speed',0.) or 0.)
    direction=float(getattr(o,'heading',0.) or 0.)
    # Upstream dynamic heading is velocity direction; static footprint heading
    # is explicitly preserved when available. Do not rotate a box with velocity.
    heading=float(getattr(o,'shape_heading',getattr(o,'theta',direction)) or 0.)
    length=float(getattr(o,'length',0.) or 0.)
    width=float(getattr(o,'width',0.) or 0.)
    radius=float(getattr(o,'radius',.15) or .15)
    if length<=0 or width<=0: length=width=2*radius
    uncertain=typ=='unknown'
    if typ in ('static_large','block','static_small'):speed=0.
    return TrackedObstacle(float(getattr(o,'x',getattr(o,'fx',0.))),
                           float(getattr(o,'y',getattr(o,'fy',0.))),length,width,
                           heading,speed*math.cos(direction),speed*math.sin(direction),
                           int(getattr(o,'id',0)),uncertain,1.5 if uncertain else 0.)


def minimum_gaps(poses, times, obstacles, geometry, age=0., interval_bound=0.):
    """Exact OBB distance at samples, minus uncertainty and sweep bound.

    poses: (..., T, 3), times: (T,). Returns shape (...,), minimum gap over T.
    Moving obstacles are translated at EVERY trajectory time, not only at end.
    """
    poses=np.asarray(poses,dtype=float);times=np.asarray(times,dtype=float)
    shape=poses.shape[:-2]; count=int(np.prod(shape)) if shape else 1
    flat=poses.reshape((-1,3)); T=poses.shape[-2]
    corners=_rect_corners_vec(flat[:,0],flat[:,1],flat[:,2],geometry.length,geometry.width)
    gaps=np.full((count,T),np.inf)
    for o in obstacles:
        if not all(math.isfinite(x) for x in (o.x,o.y,o.length,o.width,o.vx,o.vy,o.heading)):
            return np.full(shape,-np.inf)
        tt=np.maximum(0.,times+age)
        ox=np.tile(o.x+o.vx*tt,count);oy=np.tile(o.y+o.vy*tt,count)
        a=corners-np.stack((ox,oy),axis=1)[:,None,:]
        b=_rect_corners_vec(np.array([0.]),np.array([0.]),np.array([o.heading]),o.length,o.width)[0]
        lower=(np.hypot(flat[:,0]-ox,flat[:,1]-oy)-geometry.radius-math.hypot(o.length/2,o.width/2))
        near=lower<.6
        d=lower.copy()
        if np.any(near):d[near]=_rect_rect_gaps_vec(a[near],b)
        d=d.reshape(count,T)
        d-=o.uncertainty_speed*tt[None,:]
        d-=interval_bound + o.speed*(times[1]-times[0])/2 if len(times)>1 else interval_bound
        gaps=np.minimum(gaps,d)
    return np.min(gaps,axis=-1).reshape(shape)


def rollout_rear(x,y,yaw,speed,steer,vs,ds,geometry,horizon=3.,dt=.1,
                 acc=.7,dec=1.2,steer_rate=math.radians(35)):
    """Ramp-constrained bicycle rollout. Returns rear poses and actual speeds."""
    vs=np.asarray(vs,dtype=float);ds=np.asarray(ds,dtype=float);n=len(vs)
    times=np.linspace(0.,horizon,int(math.ceil(horizon/dt))+1);step=times[1]-times[0]
    rear=np.empty((n,len(times),3));vv=np.empty((n,len(times)))
    xx=np.full(n,x,dtype=float);yy=np.full(n,y,dtype=float);th=np.full(n,yaw,dtype=float);v=np.full(n,max(0.,speed),dtype=float);delta=np.full(n,steer,dtype=float)
    rear[:,0,:]=np.stack((xx,yy,th),axis=1);vv[:,0]=v
    for j in range(1,len(times)):
        v+=np.clip(vs-v,-dec*step,acc*step)
        delta+=np.clip(ds-delta,-steer_rate*step,steer_rate*step)
        omega=v*np.tan(delta)/geometry.wheelbase
        mid=th+omega*step/2
        xx+=v*np.cos(mid)*step; yy+=v*np.sin(mid)*step; th+=omega*step
        rear[:,j,:]=np.stack((xx,yy,th),axis=1);vv[:,j]=v
    return rear,vv,times


def to_centres(rear,geometry):
    out=np.array(rear,copy=True)
    out[...,0]+=geometry.rear_offset*np.cos(out[...,2])
    out[...,1]+=geometry.rear_offset*np.sin(out[...,2])
    return out


def spin_is_safe(x,y,yaw,target,obstacles,geometry,age=0.,wait=3.,speed=.6,margin=.12):
    err=(target-yaw+math.pi)%(2*math.pi)-math.pi
    duration=wait+abs(err)/max(speed,.05)+1.
    times=np.linspace(0,duration,int(math.ceil(duration/.1))+1)
    fraction=np.clip((times-wait)/max(abs(err)/max(speed,.05),.01),0,1)
    poses=np.stack((np.full(len(times),x),np.full(len(times),y),yaw+err*fraction),axis=1)
    bound=(speed*geometry.radius)*.05
    return bool(minimum_gaps(poses,times,obstacles,geometry,age,bound)>=margin)

def rollout_stop(x,y,yaw,speed,steer,vs,ds,geometry,horizon=3.,dt=.1,reaction=.25,dec=.8):
    """Candidate's first action, delayed braking, then stationary occupancy.

    This guards safe stopping locations for moving obstacles: a nominal cruise
    path alone can become unsafe if tracking or the next plan brakes early.
    Braking assumption is deliberately slower than the commanded 1.2 m/s².
    """
    vs=np.asarray(vs,dtype=float);ds=np.asarray(ds,dtype=float);n=len(vs)
    tt=np.linspace(0.,horizon,int(math.ceil(horizon/dt))+1);h=tt[1]-tt[0]
    rear=np.empty((n,len(tt),3));xx=np.full(n,x,dtype=float);yy=np.full(n,y,dtype=float);th=np.full(n,yaw,dtype=float);vv=np.full(n,max(0.,speed),dtype=float);delta=np.full(n,steer,dtype=float)
    rear[:,0]=np.column_stack((xx,yy,th))
    for j,t in enumerate(tt[1:],1):
        target=vs if t<=reaction else np.zeros(n)
        vv+=np.clip(target-vv,-dec*h,.7*h)
        if t<=reaction:delta+=np.clip(ds-delta,-math.radians(30)*h,math.radians(30)*h)
        omega=vv*np.tan(delta)/geometry.wheelbase;mid=th+omega*h/2
        xx+=vv*np.cos(mid)*h;yy+=vv*np.sin(mid)*h;th+=omega*h
        rear[:,j]=np.column_stack((xx,yy,th))
    return rear,tt
