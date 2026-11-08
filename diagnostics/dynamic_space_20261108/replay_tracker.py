import sys,json,collections,time,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa')]
from rda_dwa_real.kf_tracker import GlobalKFTracker
RUN=Path('/Users/bytedance/Downloads/run_19700110_080449_336119475');frames=[]
for path in sorted(RUN.glob('messages_*.jsonl')):
 for line in path.open():
  r=json.loads(line)
  if r['topic']=='/rda/perception_diag':frames.append((r['t'],json.loads(r['payload']['data'])))
tracker=GlobalKFTracker(dt=.05);last=None;output=[]
for t,p in frames:
 stamp=p['source_stamp'];dt=stamp-last if last is not None else .05;last=stamp
 if dt>2.:tracker.reset()
 tracks=tracker.step(p['detections_world'],dt=dt)
 output.append({'t':t,'source_stamp':stamp,'tracks':tracks,'old_unknown_count':p['unknown_count'],'new_unknown_count':sum(q[5]=='unknown' for q in tracks)})
sub=[r for r in output if r['t']>=44.8]
summary={'replay_note':'Recorded detections through new tracker; optional SciPy unavailable here, greedy association fallback. No raw cloud bag supplied.','frames_after_goal':len(sub),'old_unknown_p50':float(np.median([r['old_unknown_count'] for r in sub])),'new_unknown_p50':float(np.median([r['new_unknown_count'] for r in sub])),'old_unknown_mean':float(np.mean([r['old_unknown_count'] for r in sub])),'new_unknown_mean':float(np.mean([r['new_unknown_count'] for r in sub])),'no_unknown_frames':sum(r['new_unknown_count']==0 for r in sub)}
(OUT/'tracker_replay.json').write_text(json.dumps({'summary':summary,'frames':output},separators=(',',':')));print(json.dumps(summary,indent=2))
