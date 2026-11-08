"""Reproducible production-core closed loop; no ROS or hardware connection.
Independent actuator integration and polygon distance oracle. Assumptions in
implement-stage/ASSUMPTIONS.md. Scene expectations are written BEFORE runs.
"""
from pathlib import Path
import sys,math,json,time,argparse,collections,ctypes,queue,types
from unittest.mock import patch
from dataclasses import replace
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
BASELINE='--baseline' in sys.argv
SOURCE=OUT/'baseline' if BASELINE else ROOT/'src'
sys.path[:0]=[str(SOURCE/'rda_dwa'),str(SOURCE/'rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams,AckermannModel
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.motion_safety import TrackedObstacle,VehicleGeometry
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig
from rda_dwa_real.traj_tracker_core import TrajTrackerCore,TrajTrackerParams
CONFIG=yaml.safe_load((SOURCE/'rda_dwa_real/config/real_vehicle.yaml').read_text());G=VehicleGeometry()


def system(overrides=None,worker_delay=0.):
 b=dict(CONFIG['rda_dwa_real_bridge']['ros__parameters']);b.update(overrides or {});p=module_config('v15_latesc')
 for k,v in b.items():setattr(p,k,v)
 for k,v in (overrides or {}).items():setattr(p,k,v)
 p.max_linear_velocity=b['v_max'];p.command_steer_rate=math.radians(b['command_steer_rate_deg']);p.command_steer_accel=math.radians(b['command_steer_accel_deg']);p.dt=.1
 ack=AckermannParams(length=1.2,width=.8,wheelbase=.69,delta_max=math.radians(30.))
 planner=RealtimeAckermannPlanner(p,'ENHANCED_DWA',collision_model='rectangle',ack_params=ack)
 if not BASELINE:
  from rda_dwa.multimode_planner import MultiModePlanner
  planner=MultiModePlanner(planner)
 if worker_delay>0:planner=DelayedModeWorker(planner,worker_delay)
 cfg=BridgeConfig()
 for k,v in b.items():setattr(cfg,k,v)
 for k,v in (overrides or {}).items():setattr(cfg,k,v)
 cfg.control_dt=.1
 core=RealtimeVehicleBridgeCore(planner,p,config=cfg,ack_model=AckermannModel(ack))
 tp=TrajTrackerParams()
 for k,v in CONFIG['traj_tracker_node']['ros__parameters'].items():
  if hasattr(tp,k):setattr(tp,k,v)
 for k,v in (overrides or {}).items():
  key='max_steer_rate_deg' if k=='command_steer_rate_deg' else k
  if hasattr(tp,key):setattr(tp,key,v)
 tracker=TrajTrackerCore(tp)
 return core,tracker,planner


def scenes():
 def box(x,y,l=.6,w=.6,ident=1):return TrackedObstacle(x,y,l,w,ident=ident)
 cases=[dict(name='open',objects=[],expected='Keep ACK; no trap or spin; brake at goal.')]
 for speed in (.4,.45,.5):cases.append(dict(name=f'pedestrian_{speed}',objects=[TrackedObstacle(3.,0.,.45,.45,vx=-speed,ident=1)],expected='Early safe side avoidance; compare ACK with 3s CRAB waiting. No trap for a passing actor.'))
 for speed in (.5,.75,1.):cases.append(dict(name=f'car_{speed}',objects=[TrackedObstacle(4.,0.,1.2,.8,vx=-speed,ident=1)],expected='Early side avoidance and checked waiting refuge. If switch cannot finish before contact, use ACK or stop safely; no unnecessary spin.'))
 cases += [dict(name='single_barrier',objects=[box(2.5,0.)],expected='ACK side arc if cheaper; otherwise one CRAB segment, hold side until obstacle passed.'),dict(name='staggered_barriers',objects=[box(1.5,.65,ident=1),box(3.7,-.65,ident=2)],expected='Continuous safe S/translation route; no left-right mode chattering or false trap.')]
 # U inner width=1.8m, wall thickness=.1, rear opening. Robot inside facing closed end.
 u=[box(.1,.95,2.4,.1,1),box(.1,-.95,2.4,.1,2),box(1.35,0.,.1,2.,3)]
 cases += [dict(name='u_window',objects=u,expected='Check full spin sweep first. Rotate toward rear opening if safe, exit, then route around U to default goal; road-side corridors may be too narrow at .12m clearance.',exit_x=-1.8),dict(name='u_exit_only',objects=u,goal=(-2.,0.),expected='Compare CRAB backing with SPIN+ACK; preserve body yaw when backing is cheaper.',exit_x=-1.8),dict(name='u_spin_blocked',objects=[box(.1,.7,2.4,.1,1),box(.1,-.7,2.4,.1,2),box(1.35,0.,.1,1.5,3)],goal=(-2.,0.),expected='Refuse unsafe rotation; compare CRAB backing exit without rotating body.')]
 cases += [dict(name='staggered_tight',objects=[box(2.,.65,ident=1),box(3.4,-.65,ident=2)],expected='Retain original tight geometry as pressure test; safe failure allowed when route does not fit.') ]
 cases += [dict(name='campus_bollards',objects=[box(2.,.7,.25,.25,1),box(2.,-.7,.25,.25,2),box(3.5,1.15,.3,.3,3)],expected='ACK through central opening; no mode switch.'),dict(name='campus_bench',objects=[box(2.7,.6,1.8,.5,1)],expected='Pass on right within road boundaries.'),dict(name='campus_bins',objects=[box(2.1,-.6,.5,.5,1),box(3.7,.75,.5,.5,2)],expected='Smooth alternating avoidance; avoid repeated switching.'),dict(name='campus_planter',objects=[box(2.8,0.,1.2,1.0,1)],expected='Side passage with full body clearance; return toward goal.'),dict(name='campus_chicane',objects=[box(1.9,1.1,.6,1.5,1),box(3.5,-1.1,.6,1.5,2)],expected='Use feasible S passage or CRAB segments; evaluate 3s cost.'),dict(name='campus_wall_gap',objects=[box(2.6,1.4,.15,1.2,1),box(2.6,-1.4,.15,1.2,2)],expected='Straight ACK through wide door, no trap.'),dict(name='campus_full_block',objects=[box(2.7,0.,.4,4.,1)],expected='Safe bounded failure; do not claim goal reachable or spin indefinitely.'),dict(name='lateral_goal',objects=[],goal=(0.,1.2),expected='One CRAB +90deg switch, translate without yaw; remain CRAB at arrival.')]
 for c in cases:c.setdefault('goal',(5.,0.));c.setdefault('exit_x',None)
 return cases


def corners(x,y,yaw,l,w):
 c,s=math.cos(yaw),math.sin(yaw)
 return [(x+c*a-s*b,y+s*a+c*b) for a,b in [(-l/2,-w/2),(l/2,-w/2),(l/2,w/2),(-l/2,w/2)]]


def polygon_gap(a,b):
 separations=[]
 for p in (a,b):
  for i in range(4):
   dx,dy=p[(i+1)%4][0]-p[i][0],p[(i+1)%4][1]-p[i][1];norm=math.hypot(dx,dy);n=(-dy/norm,dx/norm)
   aa=[q[0]*n[0]+q[1]*n[1] for q in a];bb=[q[0]*n[0]+q[1]*n[1] for q in b]
   separations.append(max(min(aa)-max(bb),min(bb)-max(aa)))
 if max(separations)<=0:return max(separations)
 def point_seg(p,a,b):
  dx,dy=b[0]-a[0],b[1]-a[1];u=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
  return math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)
 return min(point_seg(p,q[i],q[(i+1)%4]) for pset,q in [(a,b),(b,a)] for p in pset for i in range(4))


from rda_dwa_real.async_planner import AsyncPlanner
class DelayedModeWorker(AsyncPlanner):
 def __init__(self,base,delay):
  self.base=base;self.base.params.actuator_delay_s+=delay;self.delay=delay;self.max_age=.8;self.reference=[];self.telemetry={};self.current_steer=0.;self.obstacle_age=0.;self.current_acc=0.;self.current_steer_rate=0.
  self._pending=False;self._ready=True;self._seq=0;self._result=None;self._input=queue.Queue(1);self._output=queue.Queue(2)
  self._process=types.SimpleNamespace(is_alive=lambda:True);self.now=0.;self.due=None;self.packet=None
 def plan(self,robot,goal,obs,trap=None):
  if self.due is not None and self.now>=self.due:self._output.put(('result',self.packet));self.due=None
  with patch('rda_dwa_real.async_planner.time.monotonic',return_value=self.now):answer=super().plan(robot,goal,obs,trap)
  if not self._input.empty():
   seq,stamp,r,g,o,tr,steer,age,dt,acc,rate=self._input.get()
   b=self.base;b.current_mode=getattr(r,'motion_mode',0);b.mode_epoch=getattr(r,'mode_epoch',0);b.current_steer=steer;b.obstacle_age=age;b.current_acc=acc;b.current_steer_rate=rate;b.params.dt=dt
   start=time.perf_counter();result=b.plan(r,g,o,tr);duration=time.perf_counter()-start
   self.packet=(seq,stamp,r,g,result,list(b.reference),dict(b.telemetry,worker_duration_s=duration));self.due=self.now+max(self.delay,duration)
  return answer

class DriverPolicy:
 def __init__(self):
  self.lib=ctypes.CDLL(str(OUT/'libdriver_policy.dylib'));self.lib.mode_create.restype=ctypes.c_void_p;self.raw=self.lib.mode_create()
  self.lib.mode_tick.argtypes=[ctypes.c_void_p,ctypes.c_double,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int];self.lib.mode_tick.restype=ctypes.c_char_p
  self.lib.mode_ready.argtypes=[ctypes.c_void_p];self.lib.mode_phase.argtypes=[ctypes.c_void_p];self.lib.mode_phase.restype=ctypes.c_char_p
  self.lib.mode_destroy.argtypes=[ctypes.c_void_p]
 def tick(self,t,cmd,actual,fresh,valid,stopped):return self.lib.mode_tick(self.raw,t,cmd['turnmode'],cmd['speed'],cmd['angle'],actual,fresh,valid,stopped).decode()
 @property
 def ready(self):return bool(self.lib.mode_ready(self.raw))
 @property
 def phase(self):return self.lib.mode_phase(self.raw).decode()
 def close(self):self.lib.mode_destroy(self.raw)


def run(case,duration=60.,noise=False,seed=31,overrides=None,save=True,worker_delay=0.):
 core,tracker,planner=system(overrides,worker_delay);core.set_goal(*case['goal'],frame='map')
 walls=[TrackedObstacle(1.,2.05,60.,.1,ident=100),TrackedObstacle(1.,-2.05,60.,.1,ident=101)]
 objects=case['objects']+walls;rng=np.random.default_rng(seed);x=y=yaw=v=delta=omega=0.;mode=0;target=0;request_at=-10.;echo_at=0.;ready_at=-10.
 driver=None if BASELINE else DriverPolicy();mode_pending=None;driver_feedback_at=0.;driver_ready=True;seq=0;next_feedback=next_pose=0.;rows=[];switches=[];durations=[];reasons=collections.Counter();pending=collections.deque();last={'speed':0.,'angle':0.,'turnmode':0};info={};min_gap=math.inf;exit_at=None;collided=False;max_yaw=0.
 for tick in range(int(duration/.02)):
  t=tick*.02
  if t>=next_pose:
   px=x+(rng.normal(0,.002) if noise else 0.);py=y+(rng.normal(0,.002) if noise else 0.);th=yaw+(rng.normal(0,.002) if noise else 0.)
   core.set_localization(px,py,th,stamp=100.+t,arrival=t);tracker.set_pose(px,py,th,stamp=100.+t,arrival=t)
   next_pose=t+(float(rng.choice([.02,.04,.06])) if noise else .04)
  if t>=next_feedback:
   tracker.set_mode_ready(mode,t);tracker.set_speed_feedback([v if mode!=4 else omega*.429]*4,t);core.set_mode_ready(mode,arrival=t)
   if not BASELINE:
    state=dict(actual_mode=mode,target_mode=target,ready=driver_ready,phase=driver.phase)
    tracker.set_driver_state(state,t);core.set_driver_state(state,t)
   driver_feedback_at=t
   next_feedback=t+(float(rng.choice([.06,.08,.12,.18])) if noise else .08)
  if tick%5==0:
   age=.12 if noise else 0.
   observed=[replace(o,x=o.x+o.vx*max(0.,t-age),y=o.y+o.vy*max(0.,t-age)) for o in objects]
   if not(noise and tick%35==0):
    tracker.set_obstacles(observed,t-age,stamp=100.+t-age);core._last_received_obs=observed;core._t_obs=t-age
   if isinstance(planner,DelayedModeWorker):planner.now=t
   start=time.perf_counter();info=core.step(.1,t);durations.append(time.perf_counter()-start);reasons[info['reason']]+=1;seq+=1
   if BASELINE:
    tracker.set_traj_mode(info['traj_mode'],t,info['spin_session'],info['spin_target'])
    if info['traj_mode']==0:tracker.set_reference(info['road_base'],t,seq)
   else:tracker.set_plan(info,t,seq)
   healthy=info['reason'] in ('motion','goal_braking','dynamic_braking','dynamic_wait','spin_requested','spin_active','spin_reached','reached','mode_return_wait','crab_motion','crab_prepare','crab_requested','mode_switch_wait','crab_hold','recovery_motion')
   tracker.set_safety('' if healthy else info['reason'])
  cmd=tracker.step(t,.02);core.set_tracker_cmd_speed(cmd['speed'],cmd['angle'],cmd['turnmode'],arrival=t);core.set_bridge_state(tracker.bridge_state(),session=tracker._session)
  if BASELINE:
   if tick%2==0:pending.append((t+.08,dict(cmd)))
   while pending and pending[0][0]<=t+1e-9:
    _,last=pending.popleft()
    if last['turnmode']!=target and t>=ready_at and abs(v)<.035 and abs(omega)<.06:
     target=last['turnmode'];mode=target;request_at=t;ready_at=t+3.;switches.append(dict(t=t,mode=mode))
   active=t>=ready_at and last['turnmode']==mode
  else:
   if mode_pending is not None and t>=mode_pending[0]:mode=mode_pending[1];mode_pending=None
   if tick%2==0:
    packet=driver.tick(t,cmd,mode,t-driver_feedback_at<.5,tracker._velocity_estimator.valid and t-tracker._pose_at<.35,tracker._velocity_estimator.speed_norm<.035 and abs(tracker._velocity_estimator.omega)<.06)
    if packet.startswith('Car SteerModeCtrl'):
     target=int(packet.split()[-1]);mode_pending=(t+.08,target);switches.append(dict(t=t,mode=target));ready_at=t+3.08
    elif packet.startswith('Car Run'):
     _,_,sv,sa=packet.split();pending.append((t+.08,dict(speed=float(sv),angle=float(sa),turnmode=mode)))
   while pending and pending[0][0]<=t+1e-9:_,last=pending.popleft()
   driver_ready=driver.ready;active=last['turnmode']==mode
  desired=last['speed'] if active and mode in (0,1) else 0.
  limit=.7 if abs(desired)>abs(v) and desired*v>=0 else 1.2
  v+=max(-limit*.02,min(limit*.02,desired-v))
  requested_angle=math.radians(last['angle']) if mode in (0,1) else 0.
  delta+=max(-math.radians(45)*.02,min(math.radians(45)*.02,requested_angle-delta))
  wanted_w=last['speed'] if active and mode==4 else 0.
  omega+=max(-.8*.02,min(.8*.02,wanted_w-omega))
  if mode==4:yaw+=omega*.02
  elif mode==1:x+=v*math.cos(yaw+delta)*.02;y+=v*math.sin(yaw+delta)*.02
  else:
   rx=x-.345*math.cos(yaw);ry=y-.345*math.sin(yaw);w=v*math.tan(delta)/.69
   rx+=v*math.cos(yaw+w*.01)*.02;ry+=v*math.sin(yaw+w*.01)*.02;yaw+=w*.02
   x=rx+.345*math.cos(yaw);y=ry+.345*math.sin(yaw)
  body=corners(x,y,yaw,1.2,.8);gap=min(polygon_gap(body,corners(o.x+o.vx*(t+.02),o.y+o.vy*(t+.02),o.heading,o.length,o.width)) for o in objects)
  min_gap=min(min_gap,gap);max_yaw=max(max_yaw,abs(yaw))
  if case.get('exit_x') is not None and x<case['exit_x'] and exit_at is None:exit_at=t
  if tick%5==0:rows.append(dict(t=round(t,3),x=x,y=y,yaw=yaw,v=v,beta_or_delta=delta,mode=mode,ready=(active if BASELINE else driver_ready),command_in_actual_mode=active,command=cmd,reason=info.get('reason'),controller=tracker.diag,arbit=core._arbit,trap=bool(info.get('trap_active')),gap=gap,planner=info.get('planner',{})))
  if gap<0:collided=True;break
  if info.get('reached') and abs(v)<.035 and abs(omega)<.06:break
 result=dict(case=case['name'],baseline=BASELINE,noise=noise,seed=seed,reached=bool(info.get('reached')),duration_s=round(t+.02,3),goal_distance_m=math.dist((x,y),case['goal']),min_gap_m=min_gap,collision=collided,switches=switches,spin_count=sum(s['mode']==4 for s in switches),crab_count=sum(s['mode']==1 for s in switches),trap_ticks=sum(r['trap'] for r in rows),exit_at_s=exit_at,max_abs_yaw_rad=max_yaw,reasons=dict(reasons),final_reason=info.get('reason'),plan_p95_ms=float(np.percentile(durations,95)*1000),plan_max_ms=max(durations)*1000,worker_delay_s=worker_delay,time_model='virtual time with delayed AsyncPlanner result delivery' if worker_delay else 'virtual time excluding synchronous solve wall time',driver_model='legacy modeled 3s with truth stop gate' if BASELINE else 'compiled C++ policy with localization-estimated stop gate',scope='preserved baseline Python cores + legacy modeled driver + independent geometry; no ROS/UDP' if BASELINE else 'production Python cores + compiled C++ policy + independent actuator/geometry; no ROS/UDP or real lidar')
 if driver is not None:driver.close()
 if save:
  label=(overrides or {}).get('experiment_label',('baseline' if BASELINE else 'candidate'))+('_noise' if noise else '')+f'_{seed}'
  folder=OUT/label;folder.mkdir(exist_ok=True);(folder/(case['name']+'_trace.json')).write_text(json.dumps(rows,allow_nan=False))
  (folder/(case['name']+'_result.json')).write_text(json.dumps(result,indent=2,allow_nan=False))
 return result


def main():
 ap=argparse.ArgumentParser();ap.add_argument('--baseline',action='store_true');ap.add_argument('--smoke',action='store_true');ap.add_argument('--cases');ap.add_argument('--noise',action='store_true');ap.add_argument('--worker-delay',type=float,default=0.);ap.add_argument('--duration',type=float,default=60.);ap.add_argument('--seed',type=int,default=31);args=ap.parse_args()
 cases=scenes();(OUT/'scenario_expectations.json').write_text(json.dumps([{**c,'objects':[vars(o) for o in c['objects']]} for c in cases],indent=2))
 selected=[c for c in cases if (c['name']=='open' if args.smoke else not args.cases or c['name'] in args.cases.split(','))]
 results=[]
 for case in selected:
  r=run(case,args.duration,args.noise,args.seed,worker_delay=args.worker_delay);results.append(r);print(json.dumps(r),flush=True)
 (OUT/(('baseline' if BASELINE else 'candidate')+('_noise' if args.noise else '')+'_results.json')).write_text(json.dumps(results,indent=2))
 if args.smoke:assert results[0]['reached'] and not results[0]['collision']
 return 0

if __name__=='__main__':raise SystemExit(main())
