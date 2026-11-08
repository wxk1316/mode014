"""Mode 1 translation: yaw held, continuous +/-90 degree wheel direction.
All poses here use the SAME rear reference as the existing ACK planner.
Collision callers convert to the physical centre. Never use tan(beta).
"""
from dataclasses import dataclass
import math
import numpy as np
from .motion_safety import speed_step,steering_step

@dataclass
class CrabParams:
    beta_max: float = math.pi/2
    beta_rate: float = math.radians(30)
    beta_accel: float = math.radians(100)
    v_max: float = .5
    acc: float = .7
    dec: float = 1.2
    jerk: float = 2.
    direction_acc: float = .6


def rollout_crab(x,y,yaw,speed,beta,vs,bs,horizon=3.,dt=.05,params=None,
                 initial_acc=0.,initial_beta_rate=0.,delay=.1,initial_sign=1.):
    p=params or CrabParams();vs=np.asarray(vs,dtype=float).reshape(-1);bs=np.asarray(bs,dtype=float).reshape(-1)
    n=max(len(vs),len(bs));vs=np.broadcast_to(vs,(n,));bs=np.broadcast_to(bs,(n,))
    if not np.all(np.isfinite(vs)) or not np.all(np.isfinite(bs)):raise ValueError('non-finite CRAB command')
    if np.any(np.abs(bs)>p.beta_max+1e-8):raise ValueError('CRAB angle outside physical range')
    ts=np.linspace(0,horizon,int(math.ceil(horizon/dt))+1);h=ts[1]-ts[0]
    poses=np.empty((n,len(ts),3));vel=np.empty((n,len(ts)));angles=np.empty_like(vel)
    xx=np.full(n,float(x));yy=np.full(n,float(y));v=np.full(n,max(0.,speed));b=np.full(n,float(beta));a=np.full(n,float(initial_acc));r=np.full(n,float(initial_beta_rate))
    poses[:,0]=np.column_stack((xx,yy,np.full(n,yaw)));vel[:,0]=v;angles[:,0]=b
    for i in range(1,len(ts)):
        sign=np.where(np.abs(vs)>.0001,np.sign(vs),initial_sign)
        target_v=np.minimum(np.abs(vs),p.v_max);target_b=bs
        if ts[i]<=delay:target_v=np.full(n,speed);target_b=np.full(n,float(beta))
        # Direction changes are bounded by v*beta_dot as well as servo rate.
        max_rate=np.minimum(p.beta_rate,p.direction_acc/np.maximum(v,.05))
        b,r=steering_step(b,r,target_b,h,max_rate,p.beta_accel)
        b=np.clip(b,-p.beta_max,p.beta_max)
        v,a=speed_step(v,a,target_v,h,p.acc,p.dec,p.jerk)
        xx+=sign*v*np.cos(yaw+b)*h;yy+=sign*v*np.sin(yaw+b)*h
        poses[:,i]=np.column_stack((xx,yy,np.full(n,yaw)));vel[:,i]=v;angles[:,i]=b
    return poses,vel,ts,angles
