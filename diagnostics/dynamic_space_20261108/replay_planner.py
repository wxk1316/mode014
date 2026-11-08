"""Frozen real snapshots: same pose, goals, observations, old vs new policy.
This does not simulate a new vehicle trajectory or claim physical goal arrival.
"""
import sys,json,math,importlib.util,time,collections,types
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.motion_safety import TrackedObstacle
spec=importlib.util.spec_from_file_location('rda_dwa.realtime_before',OUT/'before/src/rda_dwa/rda_dwa/realtime_ackermann.py');old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
def make(cls):
 pp=module_config('v15_latesc');pp.dt=.1;pp.sim_time=3.;pp.max_linear_velocity=1.;pp.planning_budget_s=1.;pp.velocity_horizon_targets=True;pp.steer_weight=.4;pp.steer_change_weight=.12
 return cls(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=AckermannParams(length=1.2,width=.8,wheelbase=.69))
planners=[make(old.RealtimeAckermannPlanner),make(RealtimeAckermannPlanner)];rows=[]
for sample in json.loads((OUT/'matched_frames.json').read_text()):
 t=sample['planner'];r=types.SimpleNamespace(x=t['sampled_pose'][0],y=t['sampled_pose'][1],theta=t['sampled_pose'][2],v=t['sampled_velocity'],planning_stamp=sample['t'])
 obs=[]
 for q in sample['obstacles']['obstacles']:
  direction=q['heading'];hd=q['shape_heading'] if q['has_shape_heading'] else direction;sp=q['speed']
  obs.append(TrackedObstacle(q['x'],q['y'],q['length'],q['width'],hd,sp*math.cos(direction),sp*math.sin(direction),q['id'],q['type']=='unknown',1.5 if q['type']=='unknown' else 0.))
 outputs=[]
 for p in planners:
  p.current_steer=t['sampled_steer'];p.obstacle_age=t['sampled_obstacle_age_s'];p.params.dt=t['sampled_control_dt_s'];start=time.perf_counter();ans=p.plan(r,sample['goal'],obs)
  outputs.append({'reason':p.telemetry['reason'],'speed':ans[1],'gap':p.telemetry.get('min_gap'),'duration_s':time.perf_counter()-start,'unknown_stop_horizon_s':p.telemetry.get('unknown_stop_horizon_s'),'unknown_stop_min_gap':p.telemetry.get('unknown_stop_min_gap')})
 rows.append({'t':sample['t'],'before':outputs[0],'after':outputs[1]})
result={'note':'497 formerly yielding snapshots; recorded obstacles matched approximately to planner request receipt time; initial acceleration/rate unavailable and set to zero in both comparisons. Greedy/Hungarian tracking is not rerun. Frozen poses do not demonstrate closed-loop vehicle movement.','frames':len(rows),'before_reasons':dict(collections.Counter(r['before']['reason'] for r in rows)),'after_reasons':dict(collections.Counter(r['after']['reason'] for r in rows)),'before_positive':sum(r['before']['speed']>.005 for r in rows),'after_positive':sum(r['after']['speed']>.005 for r in rows),'after_speed_p50':float(np.median([r['after']['speed'] for r in rows])),'rows':rows}
(OUT/'planner_replay.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
