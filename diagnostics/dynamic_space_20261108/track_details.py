import json,collections,math,bisect
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;RUN=Path('/Users/bytedance/Downloads/run_19700110_080449_336119475')
rows=collections.defaultdict(list)
for path in sorted(RUN.glob('messages_*.jsonl')):
 for line in path.open():
  r=json.loads(line)
  if r['topic'] not in ('/rda/control_diag','/rda/diag','/rda/perception_diag','/rda/obstacles','/car_cmd','/localization'):continue
  p=r['payload']
  if isinstance(p,dict) and isinstance(p.get('data'),str):p=json.loads(p['data'])
  r['d']=p;rows[r['topic']].append(r)
controls=rows['/rda/control_diag'];bad=[r for r in controls if r['t']>44.8 and r['d']['safety_reason']=='localization_timeout']
print('control loc timeout range',[(r['t'],r['d']['centre'],r['d']['pose_arrival_age_s'],r['d']['localization_input']) for r in (bad[0],bad[len(bad)//2],bad[-1])])
print('motion commands',[(r['t'],r['d']['speed'],r['d']['angle']) for r in rows['/car_cmd'] if r['d']['speed']>.005][::25])
# Follow every geometric detection / track id over time; list frequent unknowns.
tracks=collections.defaultdict(list)
for r in rows['/rda/perception_diag']:
 p=r['d']
 for t in p.get('track_outputs',[]):tracks[t[0]].append((r['t'],*t[1:]))
rank=[]
for id,history in tracks.items():
 a=np.array([r[:5] for r in history]);ty=collections.Counter(r[5] for r in history);age=history[-1][0]-history[0][0]
 unknown=sum(r[5]=='unknown' and r[0]>=44.8 for r in history)
 if unknown:rank.append({'id':id,'unknown_frames_after_goal':unknown,'lifetime_s':age,'samples':len(history),'types':dict(ty),'xy_start':history[0][1:3],'xy_end':history[-1][1:3],'xy_range_m':np.ptp(a[:,1:3],axis=0).tolist(),'speed_p95':float(np.percentile(a[:,3],95)),'size_start':history[0][6:8],'size_end':history[-1][6:8]})
rank.sort(key=lambda x:-x['unknown_frames_after_goal']);print('most unknowns',json.dumps(rank[:12],indent=2))
keys=[r['monotonic_ns']*1e-9 for r in rows['/rda/obstacles']]
# Match planner samples against exact captured obstacle source stamp, not latest frame.
obs_by_source={r['source_stamp_ns']:r for r in rows['/rda/obstacles']};matched=[]
for r in rows['/rda/diag']:
 p=r['d'];planner=p.get('planner',{})
 if p.get('reason')!='dynamic_yield' or not planner.get('sampled_pose'):continue
 sampled_stamp=r['monotonic_ns']*1e-9-planner['result_age_s'];i=bisect.bisect_right(keys,sampled_stamp)-1
 if i<0:continue
 obs=rows['/rda/obstacles'][i]
 matched.append({'t':r['t'],'planner':planner,'goal':p['goal'],'obstacles':obs['d'],'obstacle_receive_ns':obs['monotonic_ns']})
(OUT/'matched_frames.json').write_text(json.dumps(matched,separators=(',',':')))
(OUT/'unknown_tracks.json').write_text(json.dumps(rank,indent=2))
(OUT/'track_history.json').write_text(json.dumps(tracks,separators=(',',':')))
print('matched',len(matched),'first obstacles',matched[0]['obstacles']['obstacles'][:2])
