"""Mode opportunity/recovery arbitration for the existing realtime bridge.
One mode intent per epoch; bounded recovery, fresh execution after READY.
"""
import math,time


def step_multimode(c,dt,now):
 now=time.monotonic() if now is None else now;c._t_now=now
 if not hasattr(c,'_mm_desired'):
  c._mm_desired=0;c._mm_epoch=0;c._mm_beta=0.;c._mm_stall=0.;c._mm_last=now;c._mm_wait=now;c._mm_goal=None;c._mm_candidate_at=None;c._mm_anchor=c.centre();c._mm_motion_at=now
 elapsed=max(0.,now-c._mm_last);c._mm_last=now
 if c._goal!=c._mm_goal:
  c._mm_goal=c._goal;c._mm_epoch+=1;c._mm_recovered=False;c._mm_stall=0.;c._mm_candidate_at=None;c._mm_anchor=c.centre();c._mm_motion_at=now
 info={'ok':False,'plan_ok':False,'collision':False,'reached':False,'n_obs':len(c._last_received_obs),'dist_goal':math.inf}
 def finish(reason,v=0.,w=0.,road=()):
  out=c._finish(info,reason,v,w,road)
  out.update(body_yaw_ref=getattr(c,"_mm_crab_yaw",c.robot.theta),spin_tolerance_deg=getattr(c,'_mm_spin_tolerance',4.),direction=-1 if (info.get('planner',{}).get('reverse') or info.get('planner',{}).get('crab_reverse')) else 1,schema_version=2,motion_mode={0:'ACK',1:'CRAB',4:'SPIN'}[c._mm_desired],mode_epoch=c._mm_epoch,prepare_angle_rad=info.get('planner',{}).get('selected_beta',c._mm_beta))
  return out
 def request(mode,beta=0.,yaw=None,reason='mode_switch_wait'):
  c._mm_desired=mode;c._mm_epoch+=1;c._session+=1;c._mm_beta=beta;c._mm_wait=now;c._clear_traj_pending=True
  c._mm_candidate_at=None;c._mm_stall=0.;c._arbit={0:'ACK',1:'CRAB',4:'SPIN_REQUEST'}[mode]
  if mode==1:c._mm_crab_yaw=c.robot.theta
  if yaw is not None:c._session_target=yaw;c._request_since=now;c._last_spin_result=None
  return finish(reason)
 if not c._goal_received:return finish('no_goal')
 if c._pose_fault or now-c._t_loc>c.cfg.localization_timeout:return finish('localization_fault')
 if now-c._t_obs>c.cfg.obstacle_timeout:return finish('obstacle_timeout')
 obs=c._last_received_obs;c._last_obs=obs;age=now-c._t_obs;centre=c.centre();dg=math.dist(centre,c._goal);info['dist_goal']=dg
 if math.dist(centre,c._mm_anchor)>.05:c._mm_anchor=centre;c._mm_motion_at=now
 actual_stall=now-c._mm_motion_at>2.
 c._v_meas=c._velocity_estimator.speed_norm;c.robot.v=max(0.,c._v_meas);c.robot.omega=c._omega_meas;c.robot.planning_stamp=now;c.robot.command_speed=float(c._tracker_cmd_v or 0.)
 if dg<c.pp.goal_tolerance and c._velocity_estimator.valid and c._v_meas<.035:
  info['reached']=True;return finish('reached')
 driver=getattr(c,'_driver_state',None)
 if driver is None or now-getattr(c,'_driver_arrival',-1e9)>.5:return finish('driver_feedback_stale')
 if c._mm_desired==4:
  c._mm_motion_at=now;c._mm_anchor=centre
  from rda_dwa.motion_safety import spin_is_safe
  if not spin_is_safe(*centre,c.robot.theta,c._session_target,obs,c.geometry,age,speed=getattr(c.cfg,'spin_speed',.6),margin=getattr(c.cfg,'safety_margin',.12)):
   return finish('spin_safety_abort')
  if c._last_spin_result=='reached':
   c._last_spin_result=None;return request(0,reason='spin_reached')
  if now-c._request_since>getattr(c.cfg,'spin_request_timeout',18.):return finish('spin_request_timeout')
  info['ok']=True;return finish('spin_active')
 if c._bridge_state==1 or int(driver.get('actual_mode',-1))!=c._mm_desired or not driver.get('ready'):
  if not getattr(c,'_mm_wait_seen',False):c._mm_wait=now
  c._mm_wait_seen=True
  c._mm_motion_at=now;c._mm_anchor=centre
  if now-c._mm_wait>12.:return finish('mode_switch_timeout')
  return finish('mode_switch_wait')
 c._mm_wait_seen=False
 if c._mm_desired==1 and abs((c.robot.theta-getattr(c,'_mm_crab_yaw',c.robot.theta)+math.pi)%(2*math.pi)-math.pi)>math.radians(8):
  c._goal_requests+=1;c._mm_spin_tolerance=1.;return request(4,yaw=c._mm_crab_yaw,reason='spin_requested')
 if c._recovery_failed:return finish('recovery_exhausted')
 if c._recovery_goal is not None:
  c._recovery_elapsed+=elapsed
  if c._recovery_elapsed>getattr(c.cfg,'trap_max_duration',45.):
   c._recovery_failed=True;return finish('recovery_exhausted')
 if c._recovery_goal is not None and math.dist(centre,c._recovery_goal)<.3:
  c._recovery_goal=None;c._mm_stall=0.;c._episode_retries=0;c._mm_recovered=True
 target=c._recovery_goal or c._goal
 p=c.planner;p.current_mode=c._mm_desired;p.mode_epoch=c._mm_epoch;p.current_steer=c._tracker_angle;p.obstacle_age=age
 p.current_acc=c._tracker_acc;p.current_steer_rate=c._tracker_steer_rate;p.params.dt=max(.02,min(.2,dt or .1))
 urgent=False
 if hasattr(p,'base') and getattr(p.base,'supports_multimode',False) and c._mm_desired==0:
  forward=(math.cos(c.robot.theta),math.sin(c.robot.theta))
  for o in obs:
   along=(o.x-centre[0])*forward[0]+(o.y-centre[1])*forward[1]
   closing=-(o.vx*forward[0]+o.vy*forward[1])+max(c.robot.v,min(.8,getattr(p.params,"cruise_velocity",1.)))
   lateral=abs(-(o.x-centre[0])*forward[1]+(o.y-centre[1])*forward[0])
   if not o.uncertain and o.speed>.1 and along>.3 and closing>.2 and along/closing<6. and lateral<1.4+o.width/2:urgent=True
  urgent=urgent or p.base._reverse_schedule is not None
 if urgent:
  for key in ('current_steer','obstacle_age','current_acc','current_steer_rate','current_mode','mode_epoch'):setattr(p.base,key,getattr(p,key,0.))
  original_delay=p.base.params.actuator_delay_s;p.base.params.actuator_delay_s=min(.1,original_delay)
  start=time.perf_counter()
  try:ok,v,u=p.base.plan(c.robot,target,obs,None)
  finally:p.base.params.actuator_delay_s=original_delay
  p.reference=p.base.reference;p.telemetry=dict(p.base.telemetry,urgent_parent_planning=True,urgent_duration_s=time.perf_counter()-start)
  c._mm_urgent=True
 else:
  if getattr(c,'_mm_urgent',False):
   c._mm_urgent=False;c._mm_epoch+=1;p.mode_epoch=c._mm_epoch
   if hasattr(p,'_result'):p._result=None
  ok,v,u=p.plan(c.robot,target,obs,None)
 tele=dict(p.telemetry);info.update(plan_ok=ok,planner=tele)
 reason=tele.get('reason','blocked');local=tele.get('guide_target',target)
 if reason=='crab_prepare':
  c._mm_motion_at=now;info['ok']=True;return finish(reason)
 if reason in ('planning_budget','planning_pending','planning_result_stale','planning_worker_failed','planning_exception'):
  return finish(reason)
 if tele.get('route_yaw') is not None:
  if c._goal_requests>=getattr(c.cfg,'goal_max_spin_requests',4):
   c._recovery_failed=True;return finish('recovery_exhausted')
  c._goal_requests+=1;c._mm_spin_tolerance=1.;return request(4,yaw=tele['route_yaw'],reason='spin_requested')
 if tele.get('guide_no_route'):
  from rda_dwa.motion_safety import minimum_gaps
  import numpy as np
  probes=[]
  for offset in (0.,math.pi/2,-math.pi/2):
   probes.append([centre[0]+.8*math.cos(c.robot.theta+offset),centre[1]+.8*math.sin(c.robot.theta+offset),c.robot.theta])
  blocked=[float(minimum_gaps(np.array([pose]),[0.],obs,c.geometry,age))<getattr(c.cfg,'safety_margin',.12) for pose in probes]
  if getattr(c,'_mm_recovered',False) or not all(blocked):
   c._recovery_failed=True;return finish('bounded_static_no_route')
 if c._mm_desired==0:
  bad=not ok or abs(v)<.005 or tele.get('forecast_progress',0.)<.05 or actual_stall
  choice=p.switch_candidate(c.robot,target,obs,local) if c.robot.v<.035 else None
  if choice is not None:
   distance=math.dist(centre,local);progress=max(.01,tele.get('forecast_progress',0.));ack_time=3.*distance/progress
   reference=getattr(p,'reference',[])
   axis=math.atan2(target[1]-centre[1],target[0]-centre[0]);sides=[]
   for obstacle in obs:
    if obstacle.speed>.1 or obstacle.length>=4. or obstacle.width>=2.:continue
    dx,dy=obstacle.x-centre[0],obstacle.y-centre[1]
    if -.5<dx*math.cos(axis)+dy*math.sin(axis)<math.dist(centre,target)+1.:sides.append(-dx*math.sin(axis)+dy*math.cos(axis))
   paired=any(y>.2 for y in sides) and any(y<-.2 for y in sides)
   if reference and paired:
    yaw_peak=max((point[2] for point in reference),key=lambda yaw:abs((yaw-c.robot.theta+math.pi)%(2*math.pi)-math.pi))
    yaw_change=abs((yaw_peak-c.robot.theta+math.pi)%(2*math.pi)-math.pi)
    if yaw_change>.2:
     from rda_dwa.local_route import route_waypoint
     _,future_route=route_waypoint(centre,yaw_peak,target,obs,c.geometry,getattr(c.cfg,'safety_margin',.12))
     if not future_route:ack_time+=6.+yaw_change/.6
   # Comparable waypoint time; include an ordinary change penalty.
   worthwhile=bad or choice['time_s']+getattr(c.cfg,'mode_switch_gain_s',1.)<ack_time
   if worthwhile:
    c._mm_candidate_at=now if c._mm_candidate_at is None else c._mm_candidate_at
    if bad or now-c._mm_candidate_at>=getattr(c.cfg,'mode_confirm_s',.4):return request(1,choice['beta'],reason='crab_requested')
    return finish('mode_switch_wait')
   else:c._mm_candidate_at=None
  else:c._mm_candidate_at=None
 if c._mm_desired==1 and reason=='crab_heading_unreachable':
  # Fixed-yaw CRAB has no backward direction in the forward-only first version.
  return request(0)
 dynamic=any(o.speed>.1 and not o.uncertain for o in obs)
 stagnant=(not ok or abs(v)<.005 or tele.get('forecast_progress',0.)<.02 or actual_stall)
 if reason in ('planning_budget','planning_pending','planning_result_stale','planning_worker_failed','planning_exception'):
  return finish(reason)
 c._mm_stall=c._mm_stall+elapsed if stagnant and not dynamic else 0.
 if c._mm_stall>=getattr(c.cfg,'blocked_spin_time',1.5) and getattr(c.cfg,'enable_trap',True):
  if c._episode_retries>=getattr(c.cfg,'recovery_max_retries',2) or c._goal_requests>=getattr(c.cfg,'goal_max_spin_requests',4):
   c._recovery_failed=True;return finish('recovery_exhausted')
  desired=math.atan2(local[1]-centre[1],local[0]-centre[0]);choice=c._recovery_heading(desired,obs,age,True)
  if choice is None:
   c._recovery_failed=True;return finish('recovery_no_safe_heading')
  yaw,sub=choice;c._recovery_goal=sub;c._recovery_elapsed=0.;c._episode_retries+=1;c._goal_requests+=1
  c._mm_spin_tolerance=4.
  return request(4,yaw=yaw,reason='spin_requested')
 if not ok:return finish(reason)
 if c._mm_desired==0 and v>=0. and not tele.get("reverse") and not c._safe_motion(v,u,obs,age):
  choice=p.switch_candidate(c.robot,target,obs,local) if c.robot.v<.035 else None
  if choice is not None:return request(1,choice['beta'],reason='crab_requested')
  return finish('execution_collision_guard')
 info['ok']=True;c.state.v=v
 if c._mm_desired==0:p.set_last_cmd(v,u)
 return finish(reason,v,0. if c._mm_desired==1 else u,getattr(p,'reference',[]))
