"""Deterministic current-interface engineering validation, not hardware proof.
50 Hz tracker, delayed serial worker, 20 Hz quantized/delayed actuator commands,
noisy/irregular localization, 100 ms obstacle age, independent corner clearance.
"""
import sys,math,json,time,queue,types,collections,argparse
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams,AckermannModel
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.motion_safety import TrackedObstacle,VehicleGeometry
from rda_dwa.collision_checker import _rect_corners_vec,_rect_rect_gaps_vec
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig
from rda_dwa_real.traj_tracker_core import TrajTrackerCore,TrajTrackerParams
from rda_dwa_real.async_planner import AsyncPlanner
CONFIG=yaml.safe_load((ROOT/'src/rda_dwa_real/config/real_vehicle.yaml').read_text())
G=VehicleGeometry()

class DelayedWorker(AsyncPlanner):
 def __init__(self,planner,delay):
  self.base=planner;self.max_age=.8;self.reference=[];self.telemetry={};self.current_steer=0.;self.obstacle_age=0.
  self._pending=False;self._ready=True;self._seq=0;self._result=None;self._input=queue.Queue(1);self._output=queue.Queue(2)
  self._process=types.SimpleNamespace(is_alive=lambda:True);self.due=None;self.packet=None;self.delay=delay;self.now=0.;self.durations=[]
 def plan(self,robot,goal,obs,trap=None):
  if self.due is not None and self.now>=self.due:
   self._output.put(('result',self.packet));self.due=None
  with patch('rda_dwa_real.async_planner.time.monotonic',return_value=self.now):
   result=super().plan(robot,goal,obs,trap)
  if not self._input.empty():
   seq,stamp,r,g,o,tr,steer,age,dt,acc,rate=self._input.get()
   p=self.base;p.current_steer=steer;p.obstacle_age=age;p.params.dt=dt;p.current_acc=acc;p.current_steer_rate=rate
   begin=time.perf_counter();answer=p.plan(r,g,o,tr);duration=time.perf_counter()-begin;self.durations.append(duration)
   self.packet=(seq,stamp,r,g,answer,list(p.reference),dict(p.telemetry,worker_duration_s=duration));self.due=self.now+self.delay
  return result


def system(delay):
 b=CONFIG['rda_dwa_real_bridge']['ros__parameters'];p=module_config('v15_latesc')
 for k in ('planning_budget_s','safety_margin','heading_weight','steer_weight','steer_change_weight','guide_extra','command_acc','command_dec','command_jerk','actuator_delay_s','velocity_horizon_targets','cruise_velocity','lateral_acc_max','staged_avoidance','sim_time','goal_tolerance'):setattr(p,k,b[k])
 p.max_linear_velocity=b['v_max'];p.command_steer_rate=math.radians(b['command_steer_rate_deg']);p.command_steer_accel=math.radians(b['command_steer_accel_deg']);p.dt=.1
 ack=AckermannParams(length=1.2,width=.8,wheelbase=.69,delta_max=math.radians(30.))
 planner=RealtimeAckermannPlanner(p,'ENHANCED_DWA',collision_model='rectangle',ack_params=ack)
 worker=DelayedWorker(planner,delay) if delay>0 else planner
 cfg=BridgeConfig()
 for k,v in b.items():
  if hasattr(cfg,k) or k in ('near_goal_spin_distance','near_goal_spin_angle_deg','blocked_spin_time','spin_speed','command_acc','command_steer_rate_deg','safety_margin','goal_replay_epsilon','motion_stall_time','recovery_subgoal_distance','recovery_max_retries','goal_max_spin_requests'):setattr(cfg,k,v)
 cfg.control_dt=.1
 core=RealtimeVehicleBridgeCore(worker,p,config=cfg,ack_model=AckermannModel(ack))
 tp=TrajTrackerParams()
 for k,v in CONFIG['traj_tracker_node']['ros__parameters'].items():
  if hasattr(tp,k):setattr(tp,k,v)
 tracker=TrajTrackerCore(tp)
 return core,tracker,worker


def scenario(name):
 if name.endswith('_unconfirmed'):return scenario(name[:-12])
 goal=(7.,0.);objects=[]
 if name=='static_barrier':objects=[TrackedObstacle(3.,0.,.6,.5,ident=1)]
 if name=='building_corridor':objects=[TrackedObstacle(3.,2.,10.,.2,ident=1),TrackedObstacle(3.,-2.,10.,.2,ident=2),TrackedObstacle(3.,.2,.7,.5,ident=3)]
 if name=='courtyard_corner':
  goal=(7.,3.);objects=[TrackedObstacle(3.,.7,4.,.2,ident=1),TrackedObstacle(4.9,1.8,.2,2.2,ident=2)]
 if name.startswith('head_on'):
  speed={'head_on_slow':.7,'head_on':1.,'head_on_fast':1.5,'head_on_early':1.5,'head_on_narrow':1.}[name]
  objects=[TrackedObstacle(7.5 if name=='head_on_early' else 5.,0.,.45,.45,vx=-speed,ident=3)]
  if name=='head_on_narrow':objects += [TrackedObstacle(3.,1.,10.,.2,ident=1),TrackedObstacle(3.,-1.,10.,.2,ident=2)]
 if name=='crossing':objects=[TrackedObstacle(3.,-2.5,.45,.45,vy=.7,ident=3)]
 if name=='bicycle_crossing':objects=[TrackedObstacle(4.,-3.5,1.6,.55,heading=math.pi/2,vy=1.3,ident=3)]
 if name=='unknown_nearby':objects=[TrackedObstacle(3.,1.,.45,.45,ident=3,uncertain=True,uncertainty_speed=1.5)]
 return goal,objects


def run(name,delay=.25,noise=True,duration=45.):
 core,tracker,planner=system(delay);goal,objects=scenario(name);rng=np.random.default_rng(19)
 state=np.array([-.345,0.,0.]);speed=delta=0.;records=[];reason_counts=collections.Counter();mode=0
 min_gap=math.inf;seq=0;pending=collections.deque();last_cmd={'speed':0.,'angle':0.,'turnmode':0};ready_at=-1.;next_pose=0.
 info={'reason':'no_goal'};next_supervisor=0.;now=0.;errors=[]
 for i in range(int(duration/.02)):
  now=i*.02;cx=state[0]+.345*math.cos(state[2]);cy=state[1]+.345*math.sin(state[2])
  physical=[replace(o,x=o.x+o.vx*now,y=o.y+o.vy*now) for o in objects]
  if now>=next_pose:
   px=cx+(rng.normal(0,.003) if noise else 0.);py=cy+(rng.normal(0,.003) if noise else 0.);yaw=state[2]+(rng.normal(0,math.radians(.2)) if noise else 0.)
   core.set_localization(px,py,yaw,stamp=100.+now,arrival=now);tracker.set_pose(px,py,yaw,stamp=100.+now,arrival=now)
   next_pose=now+(rng.choice([.02,.04,.06]) if noise else .02)
  if i==10:core.set_goal(*goal,frame='map')
  observed=[replace(o,x=o.x+o.vx*(now-.1),y=o.y+o.vy*(now-.1)) for o in objects]
  if name.endswith('_unconfirmed') and now<.6:
   observed=[replace(o,vx=0.,vy=0.,uncertain=True,uncertainty_speed=1.5) if o.speed>.1 else o for o in observed]
  tracker.set_obstacles(observed,now-.1,stamp=100.+now)
  core._last_received_obs=observed;core._t_obs=now-.1
  if now>=next_supervisor-1e-9:
   if isinstance(planner,DelayedWorker):planner.now=now
   begin=time.perf_counter();info=core.step(.1,now)
   reason_counts[info['reason']]+=1;seq+=1;next_supervisor=now+.1
   healthy=info['reason'] in ('motion','goal_braking','dynamic_braking','dynamic_wait','spin_requested','spin_active','spin_reached','reached','mode_return_wait')
   tracker.set_safety('' if healthy else info['reason']);tracker.set_traj_mode(info['traj_mode'],now,info['spin_session'],info['spin_target'])
   if info['traj_mode']==0:tracker.set_reference(info['road_base'],now,seq)
  cmd=tracker.step(now,.02);core.set_tracker_cmd_speed(cmd['speed'],cmd['angle'],cmd['turnmode'],arrival=now)
  # Driver forwards commands at 20 Hz; targets are quantized to 0.001 m/s and 0.01 deg.
  if i%5 in (0,3):pending.append((now+.08,dict(speed=round(cmd['speed'],3),angle=round(cmd['angle'],2),turnmode=cmd['turnmode'])))
  while pending and pending[0][0]<=now:
   _,last_cmd=pending.popleft()
   if mode!=last_cmd['turnmode']:mode=last_cmd['turnmode'];ready_at=now+3.
  tracker.set_mode_ready(mode if now>=ready_at else -1,now);tracker.set_speed_feedback([speed]*4,now)
  core.set_bridge_state(tracker.bridge_state(),session=tracker._session)
  if mode==4 and now>=ready_at:
   speed=0.;state[2]+=last_cmd['speed']*.02
   state[0]=cx-.345*math.cos(state[2]);state[1]=cy-.345*math.sin(state[2])
  elif mode==0 and now>=ready_at:
   speed=max(0.,last_cmd['speed']);delta=math.radians(last_cmd['angle']);omega=speed*math.tan(delta)/.69
   mid=state[2]+omega*.01;state[0]+=speed*math.cos(mid)*.02;state[1]+=speed*math.sin(mid)*.02;state[2]+=omega*.02
  else:speed=0.
  body=_rect_corners_vec(np.array([cx]),np.array([cy]),np.array([records[-1][3] if records else state[2]]),1.2,.8)
  gap=math.inf
  for o in physical:
   corners=_rect_corners_vec(np.array([o.x]),np.array([o.y]),np.array([o.heading]),o.length,o.width)[0]
   gap=min(gap,float(_rect_rect_gaps_vec(body,corners)[0]))
  min_gap=min(min_gap,gap)
  records.append([now,cx,cy,state[2],cmd['speed'],math.radians(cmd['angle']),gap if math.isfinite(gap) else 99.,core._v_meas,tracker.diag.get('e_y',0.),tracker.diag.get('e_th',0.),tracker.diag.get('branch',''),info['reason']])
  if info.get('reached'):break
 a=np.array([r[:10] for r in records],dtype=float);dv=np.diff(a[:,4])/.02;dr=np.diff(a[:,5])/.02
 # Report emergency stops separately: immediate stops are intentionally exempt from comfort bounds.
 smooth=np.array([r[10]=='LQR' and records[k-1][10]=='LQR' for k,r in enumerate(records) if k>0])
 jerk=np.diff(dv)/.02;steer_acc=np.diff(dr)/.02
 smooth2=smooth[1:]&smooth[:-1]
 result=dict(scenario=name,worker_delay_s=delay,pose_noise=noise,reached=bool(info.get('reached')),duration_s=now,goal_distance_m=math.hypot(records[-1][1]-goal[0],records[-1][2]-goal[1]),min_physical_gap_m=min_gap if math.isfinite(min_gap) else None,
  final_reason=info['reason'],reasons=dict(reason_counts),max_speed_m_s=float(max(a[:,4])),max_command_acc_m_s2=float(max(abs(dv[smooth]))) if any(smooth) else 0.,max_command_jerk_m_s3=float(max(abs(jerk[smooth2]))) if any(smooth2) else 0.,max_steer_rate_deg_s=float(np.rad2deg(max(abs(dr[smooth])))) if any(smooth) else 0.,max_steer_accel_deg_s2=float(np.rad2deg(max(abs(steer_acc[smooth2])))) if any(smooth2) else 0.,steering_sign_changes=int(sum((a[1:,5]*a[:-1,5]<0)&(np.maximum(abs(a[1:,5]),abs(a[:-1,5]))>math.radians(1.)))),max_tracking_lateral_m=float(max(abs(a[:,8]))),planning_duration_p95_s=float(np.percentile(planner.durations,95)) if isinstance(planner,DelayedWorker) and planner.durations else None)
 Path(__file__).with_name(name+'_trace.json').write_text(json.dumps(records))
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--scenarios',default='straight,static_barrier,building_corridor,courtyard_corner,head_on_slow,head_on,head_on_early,head_on_fast,crossing,bicycle_crossing,head_on_narrow,unknown_nearby');ap.add_argument('--delay',type=float,default=.25);ap.add_argument('--no-noise',action='store_true');args=ap.parse_args()
 results=[]
 for name in args.scenarios.split(','):
  r=run(name,args.delay,not args.no_noise);results.append(r);print(json.dumps(r),flush=True)
 Path(__file__).with_name('closed_loop_results.json').write_text(json.dumps(results,indent=2)+'\n')
