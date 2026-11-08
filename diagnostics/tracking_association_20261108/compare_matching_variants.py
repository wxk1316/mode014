"""Inspect the effect of redundant residual reachability and shape slack."""
import sys,json,types,collections,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa')]
import rda_dwa_real.kf_tracker as current
source=(ROOT/'src/rda_dwa_real/rda_dwa_real/kf_tracker.py').read_text().replace('distance <= min(self.gate, reach_limit)','distance <= self.gate')
module=types.ModuleType('fixed_residual');exec(compile(source,'variant.py','exec'),module.__dict__)
frames=json.loads((OUT/'tracker_replay.json').read_text())['frames']
# Fetch the same detection stream (recorded boxes, not raw clouds).
RUN=Path('/Users/bytedance/Downloads/run_19700110_080449_336119475');detections={}
for path in sorted(RUN.glob('messages_*.jsonl')):
 for line in path.open():
  row=json.loads(line)
  if row['topic']=='/rda/perception_diag':
   diag=json.loads(row['payload']['data'])
   if 'detections_world' in diag:detections[diag['source_stamp']]=diag['detections_world']
double=types.ModuleType('double_residual');exec(compile(source.replace('distance <= self.gate and reachable','distance <= min(self.gate, reach_limit) and reachable'),'double.py','exec'),double.__dict__)
trackers=[double.GlobalKFTracker(dt=.05,shape_slack_max=.15),module.GlobalKFTracker(dt=.05,shape_slack_max=.15),module.GlobalKFTracker(dt=.05,shape_slack_max=.35)]
results=[[],[],[]]
for row in frames:
 for tracker,output in zip(trackers,results):
  if row['dt_s']>2.:tracker.reset()
  tracks=tracker.step(detections[row['source_stamp']],dt=row['dt_s'])
  if row['t']>=44.8:output.append((len(tracks),sum(q[5]=='unknown' for q in tracks),sum(q[5] not in ('unknown','block','static_large') and q[3]>.1 for q in tracks)))
summary={}
for name,tracker,output in zip(('double_time_gate_slack15','fixed_half_metre_slack15','fixed_half_metre_slack35'),trackers,results):
 a=np.array(output);summary[name]={'created_tracks':tracker._next_id-1,'mean_tracks':float(a[:,0].mean()),'mean_unknown':float(a[:,1].mean()),'mean_moving':float(a[:,2].mean())}
(OUT/'matching_variants.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
