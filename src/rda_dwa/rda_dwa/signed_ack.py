"""Signed ACK rear-reference forecast. Direction reversals require body stop.
Magnitude actuator dynamics match the forward model; yaw uses signed velocity.
"""
import math
import numpy as np
from .motion_safety import speed_step,steering_step


def rollout_reverse(x,y,yaw,speed,steer,vtargets,dtargets,geometry,horizon=7.,dt=.05,initial_acc=0.,initial_rate=0.,delay=.1,rate=math.radians(45)):
 vt=np.asarray(vtargets,float);ds=np.asarray(dtargets,float)
 if vt.ndim==1:vt=vt[:,None]
 if ds.ndim==1:ds=ds[:,None]
 n=max(len(vt),len(ds));ts=np.linspace(0,horizon,int(math.ceil(horizon/dt))+1);h=ts[1]-ts[0]
 vt=np.broadcast_to(vt,(n,len(ts))) if vt.shape[1]==1 else vt;ds=np.broadcast_to(ds,(n,len(ts))) if ds.shape[1]==1 else ds
 xx=np.full(n,float(x));yy=np.full(n,float(y));th=np.full(n,float(yaw));v=np.full(n,abs(speed));a=np.full(n,float(initial_acc));b=np.full(n,float(steer));r=np.full(n,float(initial_rate))
 poses=np.empty((n,len(ts),3));vel=np.empty((n,len(ts)));angles=np.empty_like(vel)
 poses[:,0]=np.c_[xx,yy,th];vel[:,0]=v;angles[:,0]=b
 for i in range(1,len(ts)):
  j=max(0,int(np.searchsorted(ts,ts[i]-delay,side="right")-1))
  target_v=vt[:,j] if ts[i]>delay else abs(speed);target_b=ds[:,j] if ts[i]>delay else steer
  v,a=speed_step(v,a,target_v,h,.7,1.2,2.);b,r=steering_step(b,r,target_b,h,rate,math.radians(100))
  w=-v*np.tan(b)/geometry.wheelbase;xx-=v*np.cos(th+w*h/2)*h;yy-=v*np.sin(th+w*h/2)*h;th+=w*h
  poses[:,i]=np.c_[xx,yy,th];vel[:,i]=v;angles[:,i]=b
 return poses,vel,ts,angles
