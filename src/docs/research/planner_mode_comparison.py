"""Offline only: compare full multi-mode schedules, including waits and footprint."""
import json,math,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real'),str(ROOT/'tests')]
from test_planner_safety import planner,obstacle
from rda_dwa.robot_model import RobotState
from rda_dwa.hybrid_primitives import evaluate_maneuver


def main():
    scenarios={
      'open_lateral_target':[],
      'narrow_corridor':[obstacle(0,.75,length=6,width=.1,kind='block'),obstacle(0,-.75,length=6,width=.1,kind='block')],
      'rear_blocked':[obstacle(-1.1,0,length=.4,width=2,kind='block')],
      'dynamic_crossing_wait':[obstacle(0,3,heading=-math.pi/2,speed=1.)],
      'side_blocked':[obstacle(0,1.,length=.4,width=.4,kind='block')],
    }
    requests={
      'ACK_STRAIGHT':dict(mode='ACK',speed=.2,steering=0.,motion_time=3.),
      'ACK_FORWARD':dict(mode='ACK',speed=.2,steering=.35,motion_time=3.),
      'ACK_REVERSE':dict(mode='ACK',speed=-.2,steering=0.,motion_time=3.),
      'SPIN_ALIGN':dict(mode='SPIN',yaw_change=math.pi/2),
      'CRAB':dict(mode='CRAB',speed=.2,translation_angle=math.pi/4,motion_time=3.,simulation=True),
      'TRANSLATE':dict(mode='TRANSLATE',speed=.2,translation_angle=math.pi/2,motion_time=3.,simulation=True),
    }
    reports=[]
    for name,obs in scenarios.items():
      for action,args in requests.items():
        a,_=planner();before=time.perf_counter();result=evaluate_maneuver(a.cc,RobotState(),obs,**args);cpu=time.perf_counter()-before
        path=float(np.linalg.norm(np.diff(result.trajectory[:,:2],axis=0),axis=1).sum()) if len(result.times) else None
        phases={phase:result.phases[1:].count(phase)*.05 for phase in set(result.phases)}
        reports.append(dict(scenario=name,action=action,status=result.status,safe=result.safe,
            protocol_verified=result.protocol_verified,minimum_model_gap_m=result.minimum_gap_m,
            full_action_duration_s=result.nominal_duration_s,geometric_path_m=path,
            final_pose=result.trajectory[-1].tolist(),phase_time_s=phases,cpu_ms=cpu*1000))
    for ready in (None,5.):
      a,_=planner();r=evaluate_maneuver(a.cc,RobotState(),[],mode='SPIN',yaw_change=math.pi/2,ready_at_s=ready)
      reports.append(dict(scenario='ready_failure' if ready is None else 'late_ready',action='SPIN_ALIGN',status=r.status,safe=r.safe,full_action_duration_s=r.nominal_duration_s))
    payload={'date':'2026-11-06','scope':'synthetic planar model schedules; not ROS2 or actual wheel protocol',
      'unverified_modes':'CRAB/TRANSLATE only simulate user-provided physical capability; command/feedback/units remain disabled',
      'comparison_limit':'Different endpoint tasks; results compare capability/risk/dwell, not equal-goal optimality',
      'results':reports}
    (ROOT/'docs/research/planner_mode_comparison.json').write_text(json.dumps(payload,indent=2)+'\n')
    for x in reports:print(json.dumps(x),flush=True)


if __name__=='__main__':main()
