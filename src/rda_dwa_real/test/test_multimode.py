"""Behavioural tests for sideways motion, readiness and mode-specific safety."""
from pathlib import Path
import sys,math
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from rda_dwa.crab_model import rollout_crab,CrabParams
from rda_dwa_real.pose_velocity import PoseVelocityEstimator
from rda_dwa_real.traj_tracker_core import TrajTrackerCore


def test_lateral_velocity_is_observable_and_warmup_unknown():
 e=PoseVelocityEstimator();e.update(0,0,0,0);assert not e.valid
 for t in np.arange(.05,.8,.05):e.update(0,.3*t,0,t)
 assert e.valid and abs(e.vy_body-.3)<1e-6 and abs(e.speed_norm-.3)<1e-6
 assert abs(e.vx_body)<1e-6
 e.reset();assert not e.valid


def test_ninety_degree_translation_preserves_yaw_and_full_distance():
 poses,v,t,b=rollout_crab(0,0,0,.3,math.pi/2,[.3],[math.pi/2],horizon=2,delay=0)
 assert max(abs(poses[0,:,0]))<1e-6
 assert abs(poses[0,-1,1]-.6)<1e-6
 assert max(abs(poses[0,:,2]))<1e-6


def test_angle_step_is_not_instantaneous_translation():
 poses,v,t,b=rollout_crab(0,0,0,.3,0,[.3],[math.pi/2],horizon=1,delay=0)
 assert b[0,-1]<math.radians(31)
 assert poses[0,-1,0]>.2 and poses[0,-1,1]<.12
 assert max(abs(np.diff(b[0])/np.diff(t)))<=math.radians(30)+1e-8


def test_crab_stop_retains_mode_and_angle():
 c=TrajTrackerCore();c._v2=True;c._v2_mode=1;c._last_steer=math.pi/2
 cmd=c.stop_cmd();assert cmd['turnmode']==1 and cmd['speed']==0 and cmd['angle']==90


def test_driver_stale_never_allows_crab_motion():
 c=TrajTrackerCore();c.set_plan({'motion_mode':'CRAB','mode_epoch':1,'road_base':[]},0,1)
 cmd=c.step(.1,.02);assert cmd['speed']==0 and c.diag['branch']=='DRIVER_STALE'


def test_switch_waits_for_physical_ready_and_new_plan():
 c=TrajTrackerCore();c.set_plan({'motion_mode':'CRAB','mode_epoch':1,'prepare_angle_rad':math.pi/2},0,1)
 first_request=None
 for i in range(340):
  t=i*.02;c.set_pose(0,0,0,stamp=t,arrival=t)
  actual=1 if first_request is not None else 0
  ready=first_request is None or t-first_request>=3
  c.set_driver_state({'actual_mode':actual,'target_mode':actual,'ready':ready},t)
  cmd=c.step(t,.02)
  if cmd['turnmode']==1 and first_request is None:first_request=t
  assert cmd['speed']==0 # no fresh executable reference after READY
 assert first_request is not None and c._v2_phase=='RUN' and c.mode=='CRAB'
 assert c._last_steer>math.radians(89)


def test_reverse_forecast_moves_back_without_relabeling_body_yaw():
 from rda_dwa.signed_ack import rollout_reverse
 from rda_dwa.motion_safety import VehicleGeometry
 poses,v,t,b=rollout_reverse(0,0,0,.3,0,[.3],[0],VehicleGeometry(),horizon=1,delay=0)
 assert poses[0,-1,0]<-.29 and max(abs(poses[0,:,1]))<1e-8
 assert max(abs(poses[0,:,2]))<1e-8


def test_async_rejects_old_mode_epoch_at_same_pose():
 import queue,types
 from rda_dwa_real.async_planner import AsyncPlanner
 c=AsyncPlanner.__new__(AsyncPlanner)
 c.base=types.SimpleNamespace(params=types.SimpleNamespace(dt=.1));c.max_age=10.;c.current_mode=1;c.mode_epoch=2;c._pending=True;c._ready=True;c._seq=1
 c._output=queue.Queue();c._input=queue.Queue();c._process=types.SimpleNamespace(is_alive=lambda:True)
 import time
 old=types.SimpleNamespace(x=0.,y=0.,theta=0.,motion_mode=0,mode_epoch=1)
 c._result=(1,time.monotonic(),old,(5.,0.),(True,.5,0.),[(0,0,0,.5,0)],{'reason':'motion'})
 ok,_,_=c.plan(types.SimpleNamespace(x=0.,y=0.,theta=0.),(5.,0.),[])
 assert not ok and 'mode_changed' in c.telemetry['stale_causes']


def test_empty_wait_plan_replaces_previous_motion():
 c=TrajTrackerCore();c.set_driver_state({'actual_mode':0,'ready':True},0.)
 c.set_plan({'motion_mode':'ACK','mode_epoch':1,'road_base':[(0,0,0,.5,0,0),(1,0,0,.5,2,0)]},0,1)
 assert c._ref
 c.set_plan({'motion_mode':'ACK','mode_epoch':1,'reason':'dynamic_wait','road_base':[]},.1,2)
 assert not c._ref


def test_signed_zero_brake_reference_stays_reverse():
 c=TrajTrackerCore();c.set_driver_state({'actual_mode':0,'ready':True},0.)
 c.set_plan({'motion_mode':'ACK','mode_epoch':1,'road_base':[(0,0,0,-0.,0,0),(-1,0,0,-0.,2,0)]},0,1)
 assert c._reverse_ref


def test_spin_does_not_complete_with_nonzero_command():
 c=TrajTrackerCore();c._v2=True;c._v2_mode=c._v2_desired=4;c._spin_target=.01;c._spin_w=.2
 for t in np.arange(0,1.01,.04):c.set_pose(0,0,0,stamp=float(t),arrival=float(t))
 c.set_driver_state({'actual_mode':4,'ready':True},1.)
 cmd=c.step(1.,.02)
 assert cmd['speed']>.1 and c._spin_result!='reached'


def test_ninety_degree_boundary_tolerates_pose_noise_without_reverse():
 import types,json
 from rda_dwa.module_config import module_config
 from rda_dwa.ackermann_model import AckermannParams
 from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
 from rda_dwa.multimode_planner import MultiModePlanner
 p=MultiModePlanner(RealtimeAckermannPlanner(module_config('v15_latesc'),'ENHANCED_DWA',collision_model='rectangle',ack_params=AckermannParams()))
 robot=types.SimpleNamespace(x=-.343,y=0.,theta=0.,v=0.,planning_stamp=0.)
 choice=p.switch_candidate(robot,(0.,1.2),[])
 assert choice and abs(choice['beta']-math.pi/2)<1e-6
 p.current_mode=1;p.current_steer=math.pi/2
 ok,_,_=p.plan(robot,(0.,1.2),[])
 assert ok
 json.dumps(p.telemetry,allow_nan=False)


def test_crab_reverse_reference_commands_reverse_translation():
 c=TrajTrackerCore()
 for t in np.arange(0,.25,.05):c.set_pose(0,0,0,stamp=float(t),arrival=float(t))
 c.set_driver_state({'actual_mode':1,'ready':True},.2)
 c.set_plan({'motion_mode':'CRAB','mode_epoch':1,'direction':-1,'road_base':[(-.345,0,0,-.3,0,0),(-2.345,0,0,-.3,3,0)]},.2,1)
 cmd=c.step(.2,.02)
 assert cmd['turnmode']==1 and cmd['speed']<0 and abs(cmd['angle'])<1e-6
