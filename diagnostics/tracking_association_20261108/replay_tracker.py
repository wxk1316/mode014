"""Same recorded detections through before/new tracking; no raw-cloud replay."""
import sys,json,math,importlib.util,collections,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa')]
from rda_dwa_real.kf_tracker import GlobalKFTracker
spec=importlib.util.spec_from_file_location('kf_before',OUT/'before/src/rda_dwa_real/rda_dwa_real/kf_tracker.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
RUN=Path('/Users/bytedance/Downloads/run_19700110_080449_336119475')
frames=[]
for path in sorted(RUN.glob('messages_*.jsonl')):
 for line in path.open():
  row=json.loads(line)
  if row['topic']=='/rda/perception_diag':
   diag=json.loads(row['payload']['data'])
   if 'detections_world' in diag:frames.append((row['t'],diag))
trackers=[old.GlobalKFTracker(dt=.05),GlobalKFTracker(dt=.05)]
last=None;rows=[];runtimes=[[],[]]
for timestamp,frame in frames:
 stamp=frame['source_stamp']
 if last is not None and stamp<=last:continue
 dt=stamp-last if last is not None else .05;last=stamp
 pair=[]
 for i,tracker in enumerate(trackers):
  if dt>2.:tracker.reset()
  start=time.perf_counter();tracks=tracker.step(frame['detections_world'],dt=dt);runtimes[i].append(time.perf_counter()-start)
  pair.append({'tracks':tracks,'unknown':sum(q[5]=='unknown' for q in tracks),'moving':sum(q[5] not in ('unknown','block','static_large') and q[3]>.1 for q in tracks),
   'missed_dynamic':sum(t.moving and t.miss>0 for t in tracker.tracks.values()),'next_id':tracker._next_id})
 rows.append({'t':timestamp,'source_stamp':stamp,'dt_s':dt,'before':pair[0],'after':pair[1]})
active=[r for r in rows if r['t']>=44.8]
summary={'note':'Recorded detections only, identical input for both trackers. No raw clouds, no object identity/velocity ground truth. SciPy absent: greedy branch. Motion counts are estimates, not evidence of actual moving objects.','frames':len(rows),'goal_active_frames':len(active)}
for i,name in enumerate(('before','after')):
 summary[name]={
  'unknown_p50':float(np.median([r[name]['unknown'] for r in active])),
  'unknown_mean':float(np.mean([r[name]['unknown'] for r in active])),
  'moving_mean':float(np.mean([r[name]['moving'] for r in active])),
  'track_count_mean':float(np.mean([len(r[name]['tracks']) for r in active])),
  'missed_dynamic_frames':sum(r[name]['missed_dynamic']>0 for r in active),
  'created_tracks':rows[-1][name]['next_id']-1,
  'runtime_p50_s':float(np.median(runtimes[i])),
  'runtime_p99_s':float(np.percentile(runtimes[i],99))}
(OUT/'tracker_replay.json').write_text(json.dumps({'summary':summary,'frames':rows},separators=(',',':')))
(OUT/'tracker_replay_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
