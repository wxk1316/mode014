"""Bounded closed-loop reverse tracking forecast for a committed macro path.
References use rear coordinates and body yaw. Plant integration is separated
from the speed/steering controller, including its command transport delay.
"""
import math
from collections import deque
import numpy as np
from .motion_safety import speed_step,steering_step


def forecast(reference,x,y,yaw,speed,steer,command_speed,acc,steer_rate,geometry,
             horizon=7.,dt=.05,phase=0.,rate=math.radians(45),ky=1.5,kth=1.8):
 q=np.asarray(reference,float);schedule=q[:,[4,3,5]].copy();keep=np.r_[True,np.hypot(np.diff(q[:,0]),np.diff(q[:,1]))>1e-5];q=q[keep]
 if len(q)<2:raise ValueError('reverse reference has no geometry')
 arc=np.r_[0.,np.cumsum(np.hypot(np.diff(q[:,0]),np.diff(q[:,1])))];angles=np.unwrap(q[:,2])
 sx=q[:-1,0];sy=q[:-1,1];dx=np.diff(q[:,0]);dy=np.diff(q[:,1]);l2=np.maximum(dx*dx+dy*dy,1e-12)
 ts=np.linspace(0,horizon,int(math.ceil(horizon/dt))+1);h=ts[1]-ts[0]
 cmd_v=abs(command_speed);cmd_delta=steer;cmd_a=acc;cmd_rate=steer_rate
 lag_speed=max(0.,abs(command_speed)-acc*.08)
 v=max(0.,speed) if speed<.1 else .3*max(0.,speed)+.7*lag_speed
 delta=steer-steer_rate*.08;actual_target=(v,delta);pending=deque()
 poses=[];vel=[]
 for t in ts:
  poses.append((x,y,yaw));vel.append(v)
  u=np.clip(((x-sx)*dx+(y-sy)*dy)/l2,0.,1.);distance=(x-sx-u*dx)**2+(y-sy-u*dy)**2
  i=int(np.argmin(distance));s=arc[i]+float(u[i])*(arc[i+1]-arc[i])
  xr=np.interp(s,arc,q[:,0]);yr=np.interp(s,arc,q[:,1]);theta=np.interp(s,arc,angles)
  vr=abs(float(np.interp(phase+t,schedule[:,0],schedule[:,1])));ff=float(np.interp(phase+t,schedule[:,0],schedule[:,2]))
  remaining=arc[-1]-s
  if vr<.01 and remaining>.08:
   vr=min(.4,math.sqrt(2.*1.2*max(0.,remaining-.05)))
   ff=float(np.interp(s,arc,q[:,5]))
  ey=-(x-xr)*math.sin(theta)+(y-yr)*math.cos(theta);eth=(yaw-theta+math.pi)%(2*math.pi)-math.pi
  correction=max(-math.radians(8),min(math.radians(8),-ky*ey+kth*eth))
  target=max(-math.pi/6,min(math.pi/6,ff+correction))
  cmd_delta,cmd_rate=steering_step(cmd_delta,cmd_rate,target,h,rate,math.radians(100))
  cmd_v,cmd_a=speed_step(cmd_v,cmd_a,vr,h,.7,1.2,2.)
  pending.append((t+.08,float(cmd_v),float(cmd_delta)))
  while pending and pending[0][0]<=t+1e-9:_,tv,td=pending.popleft();actual_target=(tv,td)
  tv,td=actual_target;limit=.7 if tv>v else 1.2
  v+=max(-limit*h,min(limit*h,tv-v));v=max(0.,v)
  delta+=max(-rate*h,min(rate*h,td-delta))
  w=-v*math.tan(delta)/geometry.wheelbase
  x-=v*math.cos(yaw+w*h/2)*h;y-=v*math.sin(yaw+w*h/2)*h;yaw+=w*h
 return np.asarray(poses)[None,:,:],np.asarray(vel)[None,:],ts
