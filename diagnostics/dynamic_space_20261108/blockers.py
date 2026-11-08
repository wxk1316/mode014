import sys,json,math,collections
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
import rda_dwa.realtime_ackermann as implementation
from rda_dwa.motion_safety import TrackedObstacle,minimum_gaps,VehicleGeometry
import types
samples=json.loads((OUT/'matched_frames.json').read_text());pp=module_config('v15_latesc');pp.dt=.1;pp.sim_time=3.;pp.max_linear_velocity=1.;pp.planning_budget_s=.5;pp.velocity_horizon_targets=True;pp.steer_weight=.4;pp.steer_change_weight=.12
planner=RealtimeAckermannPlanner(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=AckermannParams(length=1.2,width=.8,wheelbase=.69))
original=implementation.minimum_gaps;calls=[]
def capture(*args,**kwargs):
 calls.append((args,kwargs));return original(*args,**kwargs)
implementation.minimum_gaps=capture
rows=[];universal=collections.Counter()
for sample in samples[::5]:
 t=sample['planner'];x,y,yaw=t['sampled_pose'];robot=types.SimpleNamespace(x=x,y=y,theta=yaw,v=t['sampled_velocity'],planning_stamp=sample['t'])
 obs=[]
 for q in sample['obstacles']['obstacles']:
  v=q['speed'];heading=q['shape_heading'] if q['has_shape_heading'] else q['heading']
  obs.append(TrackedObstacle(q['x'],q['y'],q['length'],q['width'],heading,v*math.cos(q['heading']),v*math.sin(q['heading']),q['id'],q['type']=='unknown',1.5 if q['type']=='unknown' else 0.))
 planner.current_steer=t['sampled_steer'];planner.obstacle_age=t['sampled_obstacle_age_s'];calls.clear();result=planner.plan(robot,sample['goal'],obs)
 args,_=calls[0];poses,times,_,geometry,age,bound=args
 ids=[]
 for o in obs:
  if not o.uncertain:continue
  gaps=original(poses,times,[o],geometry,age,bound)
  current=float(original(poses[0,:1],times[:1],[o],geometry,age,0.))
  if float(np.max(gaps))<.12:universal[o.ident]+=1
  ids.append({'id':o.ident,'centre_distance_m':math.hypot(o.x-(x+.345*math.cos(yaw)),o.y-(y+.345*math.sin(yaw))),'current_gap_m':current,'best_candidate_gap_m':float(np.max(gaps)),'size':[o.length,o.width]})
 # Diagnostic counterfactual: use measured occupied boxes only, retaining known CV motion.
 measured=[TrackedObstacle(o.x,o.y,o.length,o.width,o.heading,o.vx,o.vy,o.ident,False,0.) for o in obs]
 result_measured=planner.plan(robot,sample['goal'],measured)
 rows.append({'t':sample['t'],'original_reason':result and calls and 'dynamic_yield' if not result[0] else planner.telemetry.get('reason'),'measured_box_reason':planner.telemetry['reason'],'measured_box_speed':result_measured[1],'unknowns':ids,'obstacles':sample['obstacles']['obstacles'],'robot':[x,y,yaw],'goal':sample['goal']})
(OUT/'blocking_obstacles.json').write_text(json.dumps({'sampled_frames':len(rows),'universal_blockers':universal,'measured_only_motion_frames':sum(r['measured_box_speed']>.005 for r in rows),'frames':rows},indent=2))
print(json.dumps({'frames':len(rows),'universal_blockers':universal.most_common(12),'measured_only_motion_frames':sum(r['measured_box_speed']>.005 for r in rows),'sample':rows[len(rows)//2]},indent=2)[:9000])
