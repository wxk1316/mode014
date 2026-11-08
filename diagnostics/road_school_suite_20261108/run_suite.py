"""Current production core simulation: KF -> delayed planner -> tracker -> plant.
Road boundaries and object detections are supplied; raw LiDAR/ROS/hardware are
not replayed. Independent OBB oracle checks 100 Hz physical states, stops at
first collision. Main goal remains 5 m ahead; U exit is recorded separately.
"""
import sys,math,json,time,argparse,collections,hashlib
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
import queue
from unittest.mock import patch
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams,AckermannModel
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig
from rda_dwa_real.traj_tracker_core import TrajTrackerCore,TrajTrackerParams
from rda_dwa_real.async_planner import AsyncPlanner
from rda_dwa_real.kf_tracker import GlobalKFTracker
from rda_dwa_real.obstacle_fusion import infer_type
from rda_dwa.motion_safety import TrackedObstacle
CFG=yaml.safe_load((ROOT/'src/rda_dwa_real/config/real_vehicle.yaml').read_text())
CONFIG=CFG

class DelayedWorker(AsyncPlanner):
 def __init__(self,planner,delay):
  self.base=planner;self.max_age=.8;self.reference=[];self.telemetry={};self.current_steer=0.;self.obstacle_age=0.
  self._pending=False;self._ready=True;self._seq=0;self._result=None;self._input=queue.Queue(1);self._output=queue.Queue(2)
  self._process=SimpleNamespace(is_alive=lambda:True);self.due=None;self.packet=None;self.delay=delay;self.now=0.;self.durations=[]
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



@dataclass
class Box:
 x:float;y:float;length:float;width:float;heading:float=0.;vx:float=0.;vy:float=0.;name:str='object'
 def centre(self,t):return self.x+self.vx*(t-.2),self.y+self.vy*(t-.2)

def definitions():
 cases=[]
 def case(name,group,objects,**extra):cases.append(dict(name=name,group=group,objects=objects,**extra))
 for speed in (.4,.45,.5):
  case(f'pedestrian_3m_{speed:.2f}','dynamic',[Box(3.,0.,.45,.45,vx=-speed,name='pedestrian')])
 for speed in (.5,.75,1.):
  case(f'vehicle_4m_{speed:.2f}','dynamic',[Box(4.,0.,1.2,.8,vx=-speed,name='vehicle')])
 case('single_cone_2p5m','static',[Box(2.5,0.,.28,.28,name='cone')])
 case('two_cones_centre_open','static',[Box(2.5,.8,.28,.28,name='left_cone'),Box(2.5,-.8,.28,.28,name='right_cone')])
 case('two_cones_centre_closed','static',[Box(2.5,.55,.28,.28,name='left_cone'),Box(2.5,-.55,.28,.28,name='right_cone')])
 case('two_cones_staggered','static',[Box(2.,.5,.28,.28,name='left_cone'),Box(3.25,-.5,.28,.28,name='right_cone')])
 case('school_gate_pillars','school',[Box(2.5,1.25,.4,.4,name='pillar'),Box(2.5,-1.25,.4,.4,name='pillar')])
 case('school_bollards_narrow','school',[Box(2.5,y,.2,.2,name='bollard') for y in (-1.35,-.65,.65,1.35)])
 case('school_tree_and_cone','school',[Box(2.5,.4,.4,.4,name='tree'),Box(3.6,-.5,.28,.28,name='cone')])
 case('school_parked_vehicle','school',[Box(2.7,.6,1.8,.9,name='parked_vehicle')])
 case('school_building_corner','school',[Box(2.5,.8,2.,.15,name='wall'),Box(3.5,.65,.15,1.7,name='wall')])
 case('school_bench_planter','school',[Box(2.3,.3,1.5,.45,name='bench'),Box(3.7,-.9,.45,.45,name='planter')])
 case('school_road_blocked','school',[Box(2.5,y,.45,.45,name='barrier') for y in (-1.5,-.75,0.,.75,1.5)],expected_blocked=True)
 for depth in (2.,3.):
  for clearance in (.15,.4,.7):
   end=.6+clearance;mouth=end-depth;thickness=.1
   objects=[Box((end+mouth)/2,.9+thickness/2,depth,thickness,name='u_side'),
            Box((end+mouth)/2,-.9-thickness/2,depth,thickness,name='u_side'),
            Box(end+thickness/2,0.,thickness,2.,name='u_back')]
   case(f'u_depth{depth:.0f}_front{clearance:.2f}','u',objects,u_mouth=mouth,u_depth=depth,front_clearance=clearance,u_width=1.8)
 return cases


def corners(x,y,yaw,length,width):
 local=np.array([[.5,.5],[.5,-.5],[-.5,-.5],[-.5,.5]])*np.array([length,width])
 c,s=math.cos(yaw),math.sin(yaw);return local@np.array([[c,s],[-s,c]])+np.array([x,y])


def exact_gaps(body,boxes):
 """Independent SAT penetration + vertex-to-segment Euclidean clearance."""
 if not len(boxes):return np.empty(0)
 B=np.asarray(boxes);n=len(B)
 edges_a=np.roll(body,-1,axis=0)-body;edges_b=np.roll(B,-1,axis=1)-B
 ua=np.stack((-edges_a[:2,1],edges_a[:2,0]),axis=-1);ua/=np.linalg.norm(ua,axis=-1)[:,None]
 ub=np.stack((-edges_b[:,:2,1],edges_b[:,:2,0]),axis=-1);ub/=np.linalg.norm(ub,axis=-1)[...,None]
 axes=np.concatenate((np.broadcast_to(ua,(n,2,2)),ub),axis=1)
 ap=np.einsum('pd,nad->npa',body,axes);bp=np.einsum('npd,nad->npa',B,axes)
 amin,amax=ap.min(axis=1),ap.max(axis=1);bmin,bmax=bp.min(axis=1),bp.max(axis=1)
 separated=np.any(np.maximum(bmin-amax,amin-bmax)>0.,axis=1)
 penetration=np.min(np.minimum(amax,bmax)-np.maximum(amin,bmin),axis=1)
 rel=body[None,:,None,:]-B[:,None,:,:];seg=edges_b[:,None,:,:]
 u=np.clip(np.sum(rel*seg,axis=-1)/np.maximum(np.sum(seg*seg,axis=-1),1e-12),0.,1.)
 d1=np.min(np.linalg.norm(rel-u[...,None]*seg,axis=-1),axis=(1,2))
 rel=B[:,:,None,:]-body[None,None,:,:];seg=edges_a[None,None,:,:]
 u=np.clip(np.sum(rel*seg,axis=-1)/np.maximum(np.sum(seg*seg,axis=-1),1e-12),0.,1.)
 d2=np.min(np.linalg.norm(rel-u[...,None]*seg,axis=-1),axis=(1,2))
 return np.where(separated,np.minimum(d1,d2),-penetration)


def run(case,seed=19,noise=True,warm=False,duration=45.,plant_lag=True,boundary_model='tiled'):
 start_wall=time.perf_counter();rng=np.random.default_rng(seed);core,tracker,worker=system(.25)
 p=CFG['lidar_track_node']['ros__parameters']
 kf=GlobalKFTracker(dt=.05,gate=p['track_gate'],lost_ttl=p['track_lost_ttl'],q=p['kf_q'],r=p['kf_r'],max_speed=p['obstacle_max_speed'],
  dyn_speed_thr=p['dyn_speed_thr'],dyn_hold_frames=p['dyn_hold_frames'],static_speed_thr=p['static_speed_thr'],static_hold_frames=p['static_hold_frames'],
  static_small_thr=p['static_small_thr'],position_slack=p['track_position_slack'],size_weight=p['track_size_weight'],shape_slack_max=p['track_shape_slack_max'])
 objects=case['objects'];road=[Box(3.,2.1,20.,.2,name='road'),Box(3.,-2.1,20.,.2,name='road')]
 if boundary_model=='tiled':
  road=[Box(x,y,1.2,.2,name='road_tile') for x in np.arange(-7.2,13.2,1.2)+.6 for y in (-2.1,2.1)]
 state=np.array([0.,0.,0.]);actual_v=actual_delta=actual_spin=0.;mode=0;ready_at=-1.
 command={'speed':0.,'angle':0.,'turnmode':0};driver=collections.deque();last_kf_stamp=None
 tracks=[];obs_stamp=-1e9;next_source=0.;next_pose=0.;next_supervisor=0.;sequence=0;observations=collections.deque()
 records=[];plans=[];kf_rows=[];reasons=collections.Counter();branches=collections.Counter();min_gap=math.inf;min_road=math.inf
 info={'reason':'no_goal'};collision_at=None;exit_at=None;goal_reached=False;spin_sessions=set();first_dynamic_at=None;steer_sign_changes=0;sign=0
 max_yaw_change=0.;motion_starts=0;previous_moving=False
 def measurements(stamp):
  common=rng.normal(0,.004,2) if noise else np.zeros(2);dets=[]
  for o in case.get('detections_override',objects):
   x,y=o.centre(stamp);error=rng.normal(0,.01,2)+common if noise else np.zeros(2)
   length=o.length*max(.8,1.+(rng.normal(0,.025) if noise else 0.));width=o.width*max(.8,1.+(rng.normal(0,.025) if noise else 0.))
   heading=o.heading+(rng.normal(0,math.radians(.8)) if noise else 0.)
   dets.append((x+error[0],y+error[1],infer_type(length,width),length,width,heading))
  return dets
 def publish(stamp,dets,now):
  nonlocal tracks,last_kf_stamp,obs_stamp,first_dynamic_at
  dt=.05 if last_kf_stamp is None else stamp-last_kf_stamp;last_kf_stamp=stamp
  tracks=kf.step(dets,dt=dt);obs_stamp=stamp
  if first_dynamic_at is None and any(q[5] not in ('unknown','block','static_large') and q[3]>.1 for q in tracks):first_dynamic_at=now
  packet=[(q[1],q[2],q[5],q[6],q[7],q[4],q[3],q[0],q[8] if len(q)>8 else q[4]) for q in tracks]
  packet += [(r.x,r.y,'static_large',r.length,r.width,r.heading,0.,1000+i,r.heading) for i,r in enumerate(road) if boundary_model=='continuous' or math.hypot(r.x-state[0],r.y-state[1])<8.]
  core.set_obstacles(packet,stamp=100.+stamp,arrival=now,source_age=now-stamp)
  tracker.set_obstacles(core._last_received_obs,stamp,stamp=100.+stamp)
  kf_rows.append({'t':now,'source_t':stamp,'tracks':tracks})
 if warm:
  for stamp in np.arange(-1.,-.049,.05):publish(float(stamp),measurements(float(stamp)),0.)
 for tick in range(round(duration/.02)):
  now=round(tick*.02,8)
  if now>=next_pose-1e-9:
   xy=state[:2]+(rng.normal(0,.003,2) if noise else 0.);yaw=state[2]+(rng.normal(0,math.radians(.2)) if noise else 0.)
   core.set_localization(*xy,yaw,stamp=100.+now,arrival=now);tracker.set_pose(*xy,yaw,stamp=100.+now,arrival=now)
   next_pose=now+(float(rng.choice((.02,.04,.06))) if noise else .02)
  while next_source<=now+1e-9:
   observations.append((next_source+.1,next_source,measurements(next_source)));next_source=round(next_source+.05,8)
  while observations and observations[0][0]<=now+1e-9:
   _,stamp,dets=observations.popleft();publish(stamp,dets,now)
  if tick==10:core.set_goal(5.,0.,frame='map')
  if now>=next_supervisor-1e-9:
   worker.now=now;info=core.step(.1,now);sequence+=1;next_supervisor=now+.1;reasons[info['reason']]+=1
   healthy=info['reason'] in ('motion','goal_braking','dynamic_braking','dynamic_wait','spin_requested','spin_active','spin_reached','reached','mode_return_wait')
   tracker.set_safety('' if healthy else info['reason']);tracker.set_traj_mode(info['traj_mode'],now,info['spin_session'],info['spin_target'])
   if info['traj_mode']==0:tracker.set_reference(info['road_base'],now,sequence)
   if info['traj_mode']==1:spin_sessions.add(info['spin_session'])
   plans.append({'t':now,'reason':info['reason'],'planner':info.get('planner',{}),'spin_session':info['spin_session'],'spin_target':info['spin_target'],
    'recovery_goal':info.get('recovery_goal'),'pose':state.tolist()})
  cmd=tracker.step(now,.02);branches[tracker.diag.get('branch','')]+=1
  core.set_tracker_cmd_speed(cmd['speed'],cmd['angle'],cmd['turnmode'],arrival=now)
  if tick%5 in (0,3):driver.append((now+.08,dict(speed=round(cmd['speed'],3),angle=round(cmd['angle'],2),turnmode=cmd['turnmode'])))
  while driver and driver[0][0]<=now+1e-9:
   _,command=driver.popleft()
   if mode!=command['turnmode']:mode=command['turnmode'];ready_at=now+3.
  tracker.set_mode_ready(mode if now>=ready_at else -1,now)
  tracker.set_speed_feedback([abs(actual_v)+abs(actual_spin)*math.hypot(.6,.4)]*4,now)
  core.set_bridge_state(tracker.bridge_state(),session=tracker._session)
  max_yaw_change=max(max_yaw_change,abs(state[2]));branch=tracker.diag.get('branch','')
  meaningful_sign=1 if cmd['angle']>1. else (-1 if cmd['angle']< -1. else 0)
  if meaningful_sign and sign and meaningful_sign!=sign:steer_sign_changes+=1
  if meaningful_sign:sign=meaningful_sign
  moving=actual_v>.05
  if moving and not previous_moving:motion_starts+=1
  previous_moving=moving
  record={'t':now,'x':float(state[0]),'y':float(state[1]),'yaw':float(state[2]),'v':actual_v,'delta_deg':math.degrees(actual_delta),
   'command_v':float(cmd['speed']) if cmd['turnmode']==0 else 0.,'command_steer_deg':float(cmd['angle']),'mode':mode,'reason':info['reason'],
   'tracking':branch,'min_gap':None,'road_gap':None,'unknown':sum(q[5]=='unknown' for q in tracks),'moving_tracks':sum(q[5] not in ('unknown','block','static_large') for q in tracks),
   'e_y':tracker.diag.get('e_y',0.),'e_th':tracker.diag.get('e_th',0.)}
  for substep in range(2):
   t=now+substep*.01;body=corners(*state,1.2,.8)
   physical=[corners(*o.centre(t),o.heading,o.length,o.width) for o in objects]
   gaps=exact_gaps(body,physical);gap=float(gaps.min()) if len(gaps) else math.inf
   road_gap=2.-float(np.max(np.abs(body[:,1])));min_gap=min(min_gap,gap);min_road=min(min_road,road_gap)
   record['min_gap']=gap if math.isfinite(gap) else None;record['road_gap']=road_gap
   if 'u_mouth' in case and exit_at is None and float(body[:,0].max())<case['u_mouth']-.12:exit_at=t
   if gap < -1e-7 or road_gap < -1e-7:collision_at=t;break
   dt=.01;target_v=command['speed'] if mode==0 and t>=ready_at else 0.
   if plant_lag:
    actual_v+=np.clip((target_v-actual_v)/.08,-1.2,.8)*dt;actual_v=max(0.,actual_v)
    target_delta=math.radians(command['angle']) if mode==0 else actual_delta
    actual_delta+=np.clip((target_delta-actual_delta)/.06,-math.radians(35),math.radians(35))*dt
   else:actual_v=max(0.,target_v);actual_delta=math.radians(command['angle']) if mode==0 else actual_delta
   target_spin=command['speed'] if mode==4 and t>=ready_at else 0.
   actual_spin+=np.clip((target_spin-actual_spin)/.06,-.8,.8)*dt
   if mode==4:
    state[2]+=actual_spin*dt
   else:
    rx=state[0]-.345*math.cos(state[2]);ry=state[1]-.345*math.sin(state[2]);omega=actual_v*math.tan(actual_delta)/.69
    mid=state[2]+omega*dt/2;rx+=actual_v*math.cos(mid)*dt;ry+=actual_v*math.sin(mid)*dt;state[2]+=omega*dt
    state[0]=rx+.345*math.cos(state[2]);state[1]=ry+.345*math.sin(state[2])
  records.append(record)
  if collision_at is not None:break
  if info.get('reached'):goal_reached=True;break
  if exit_at is not None and case['group']=='u':break
  if now>15. and info['reason'] in ('recovery_exhausted','recovery_no_safe_heading') and actual_v<.01 and abs(actual_spin)<.01:break
 if collision_at is not None:outcome='collision'
 elif min_road < .12-1e-6 or min_gap < .12-1e-6:outcome='clearance_below_12cm'
 elif case['group']=='u':outcome='exited_u' if exit_at is not None else 'not_exited_u'
 elif case.get('expected_blocked'):outcome='safe_blocked' if not goal_reached and actual_v<.02 else 'unexpected_motion'
 else:outcome='reached' if goal_reached else 'not_reached'
 a=np.asarray([[r['command_v'],r['command_steer_deg']] for r in records]);dv=np.diff(a[:,0])/.02;dr=np.diff(a[:,1])/.02
 smooth=np.asarray([records[i]['tracking']=='LQR' and records[i-1]['tracking']=='LQR' and records[i]['mode']==0 for i in range(1,len(records))]);smooth2=smooth[1:]&smooth[:-1]
 result=dict(name=case['name'],group=case['group'],seed=seed,noise=noise,warm_kf=warm,plant_lag=plant_lag,boundary_model=boundary_model,input_mode=case.get('input_mode','detection_boxes'),outcome=outcome,
  duration_s=records[-1]['t'],reached=goal_reached,goal_distance_m=float(np.linalg.norm(state[:2]-[5.,0.])),
  min_obstacle_gap_m=min_gap if math.isfinite(min_gap) else None,min_road_gap_m=min_road,collision_at_s=collision_at,exit_at_s=exit_at,
  spin_requests=len(spin_sessions),max_yaw_change_deg=math.degrees(max_yaw_change),first_dynamic_at_s=first_dynamic_at,
  final_reason=info['reason'],reason_counts=dict(reasons),tracking_counts=dict(branches),max_speed_m_s=max(r['v'] for r in records),
  command_acc_max=float(np.max(np.abs(dv[smooth]))) if np.any(smooth) else 0.,command_jerk_max=float(np.max(np.abs(np.diff(dv)/.02)[smooth2])) if np.any(smooth2) else 0.,
  command_steer_rate_max_deg_s=float(np.max(np.abs(dr[smooth]))) if np.any(smooth) else 0.,
  command_steer_accel_max_deg_s2=float(np.max(np.abs(np.diff(dr)/.02)[smooth2])) if np.any(smooth2) else 0.,
  steering_sign_changes=steer_sign_changes,motion_starts=motion_starts,max_lateral_error_m=max(abs(r['e_y']) for r in records),
  planner_duration_p95_s=float(np.percentile(worker.durations,95)) if worker.durations else None,wall_duration_s=time.perf_counter()-start_wall)
 tag=f"{case['name']}_seed{seed}_{'noise' if noise else 'clean'}_{'warm' if warm else 'cold'}_{'lag' if plant_lag else 'ideal'}_{boundary_model}_{case.get('input_mode','detection_boxes')}"
 folder=OUT/'runs';folder.mkdir(exist_ok=True)
 (folder/(tag+'.json')).write_text(json.dumps(dict(result=result,scenario={k:v for k,v in case.items() if k not in ('objects','detections_override')},objects=[vars(o) for o in objects],records=records,plans=plans,kf=kf_rows),separators=(',',':')))
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--group',default='all');ap.add_argument('--seeds',default='19');ap.add_argument('--clean',action='store_true');ap.add_argument('--warm',action='store_true');ap.add_argument('--ideal',action='store_true');ap.add_argument('--duration',type=float,default=45.);ap.add_argument('--names',default='');ap.add_argument('--boundary-model',choices=('continuous','tiled'),default='tiled');args=ap.parse_args()
 cases=[c for c in definitions() if (args.group=='all' or c['group'] in args.group.split(',')) and (not args.names or c['name'] in args.names.split(','))]
 results=[]
 for case in cases:
  for seed in map(int,args.seeds.split(',')):
   r=run(case,seed,not args.clean,args.warm,args.duration,not args.ideal,args.boundary_model);results.append(r);print(json.dumps(r),flush=True)
 label=f"{args.group}_{args.seeds}_{'clean' if args.clean else 'noise'}_{'warm' if args.warm else 'cold'}_{'ideal' if args.ideal else 'lag'}_{args.boundary_model}"
 if args.names:label+='_'+hashlib.sha256(args.names.encode()).hexdigest()[:8]
 (OUT/(label+'_results.json')).write_text(json.dumps(results,indent=2))
