"""Reproduce supervisor edge cases without changing production algorithms."""
import json,math
from pathlib import Path
from unittest.mock import patch
from run_suite import system
from rda_dwa.motion_safety import TrackedObstacle
OUT=Path(__file__).resolve().parent

def ready(c,t,mode=0,yaw=0.,obs=()):
    c.set_localization(0.,0.,yaw,stamp=100.+t,arrival=t)
    c.set_obstacles([(o.x,o.y,'block',o.length,o.width,o.heading,0.,o.ident,o.heading) for o in obs],stamp=100.+t,arrival=t)
    c.set_driver_state(dict(actual_mode=mode,ready=True),arrival=t)
    c._velocity_estimator.valid=True;c._velocity_estimator.speed_norm=0.

def core():
    c,_,_=system(worker_delay=0.);c.set_goal(5.,0.,frame='map');ready(c,0.)
    # Initialize the supervisor's mode fields independently of a plan result.
    with patch.object(c.planner,'plan',return_value=(False,0.,0.)):
        c.planner.telemetry={'reason':'planning_pending'};c.step(.1,0.)
    return c

rows=[]
c=core();c._mm_desired=4;c._arbit='SPIN_REQUEST';c._session_target=math.pi;c._request_since=0.
for t in (19.,20.,25.):
    ready(c,t,4);info=c.step(.1,t)
    rows.append(dict(probe='spin_timeout',t=t,reason=info['reason'],desired_mode=c._mm_desired,recovery_failed=c._recovery_failed))
assert all(r['reason']=='spin_request_timeout' and r['desired_mode']==4 for r in rows)

c=core();c._mm_desired=4;c._arbit='SPIN_REQUEST';c._session_target=math.pi;c._request_since=0.
for t in (1.,2.,20.):
    ready(c,t,4,obs=[TrackedObstacle(.7,0.,.1,.2,ident=1)]);info=c.step(.1,t)
    rows.append(dict(probe='spin_safety_abort',t=t,reason=info['reason'],desired_mode=c._mm_desired,recovery_failed=c._recovery_failed))
assert all(r['desired_mode']==4 for r in rows if r['probe']=='spin_safety_abort')

c=core();c.cfg.enable_trap=False;ready(c,1.)
with patch.object(c.planner,'plan',return_value=(False,0.,0.)):
    c.planner.telemetry={'reason':'blocked','route_yaw':math.pi/2};info=c.step(.1,1.)
rows.append(dict(probe='enable_trap_false_route_yaw',reason=info['reason'],desired_mode=c._mm_desired,spin_requests=c._goal_requests))
assert c._mm_desired==4

c=core();c._mm_desired=1;c._arbit='CRAB';c._mm_crab_yaw=0.;c._goal_requests=4
ready(c,1.,1,math.radians(10));info=c.step(.1,1.)
rows.append(dict(probe='crab_yaw_over_spin_limit',reason=info['reason'],desired_mode=c._mm_desired,spin_requests=c._goal_requests,configured_max=c.cfg.goal_max_spin_requests))
assert c._goal_requests>c.cfg.goal_max_spin_requests

c=core();ready(c,1.)
with patch.object(c.planner,'plan',return_value=(False,0.,0.)):
    c.planner.telemetry={'reason':'blocked','guide_no_route':True};first=c.step(.1,1.)
ready(c,1.1)
with patch.object(c.planner,'plan',return_value=(True,.2,0.)):
    c.planner.telemetry={'reason':'motion'};second=c.step(.1,1.1)
rows.append(dict(probe='one_frame_no_route_latched',first=first['reason'],next=second['reason'],recovery_failed=c._recovery_failed))
assert first['reason']=='bounded_static_no_route' and second['reason']=='recovery_exhausted'
(OUT/'state_probes.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
