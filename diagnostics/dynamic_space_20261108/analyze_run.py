import json,collections,hashlib,bisect,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
RUN=Path('/Users/bytedance/Downloads/run_19700110_080449_336119475')
rows=collections.defaultdict(list)
for path in sorted(RUN.glob('messages_*.jsonl')):
 for line in path.open():
  r=json.loads(line);p=r['payload']
  if isinstance(p,dict) and isinstance(p.get('data'),str):
   try:p=json.loads(p['data'])
   except (ValueError,TypeError):pass
  r['decoded']=p;rows[r['topic']].append(r)
def quant(v):
 return {str(p):float(np.percentile(v,p)) for p in (0,50,90,99,100)} if v else {}
goals=[{'t':r['t'],'payload':r['decoded']} for r in rows['/move_base_simple/goal']]
start=goals[0]['t'] if goals else 0.
diag=[r for r in rows['/rda/diag'] if r['t']>=start];per=[r for r in rows['/rda/perception_diag'] if r['t']>=start];control=[r for r in rows['/rda/control_diag'] if r['t']>=start]
summary={'goals':goals,'duration_s':max(r['t'] for group in rows.values() for r in group),'after_goal_frames':len(diag),'planning_reasons':dict(collections.Counter(r['decoded'].get('reason') for r in diag)),
 'control_reasons':dict(collections.Counter(r['decoded'].get('safety_reason') for r in control)),'perception_reasons':dict(collections.Counter(r['decoded'].get('reason') for r in per)),
 'obstacle_rate_hz':(len(rows['/rda/obstacles'])-1)/(rows['/rda/obstacles'][-1]['t']-rows['/rda/obstacles'][0]['t']),
 'perception_duration_s':quant([r['decoded'].get('duration_s',0.) for r in per]),'obstacle_age_s':quant([r['decoded'].get('obstacle_age_s',0.) for r in diag]),
 'unknown_count':quant([r['decoded'].get('unknown_count',0) for r in per]),'track_count':quant([r['decoded'].get('track_count',0) for r in per]),
 'nonzero_cmds':sum(abs(r['decoded'].get('speed',0))>1e-5 for r in rows['/car_cmd']),'nonempty_references':sum(bool(r['decoded'].get('reference')) for r in rows['/rda/plan']),
 'planner_duration_s':quant([r['decoded'].get('planner',{}).get('worker_duration_s',0.) for r in diag]),'track_reset_frames':sum(bool(r['decoded'].get('track_reset')) for r in per)}
summary['samples']=[r['decoded'] for r in diag[::max(1,len(diag)//5)]]
provenance=json.loads((RUN/'provenance.jsonl').read_text().splitlines()[0]);matches=[]
for file in provenance['files']:
 parts=file['path'].split('/rda_dwa/')[-1] if '/rda_dwa/' in file['path'] else file['path'].split('/rda_dwa_real/')[-1]
 candidates=list((ROOT/'src').glob('*/'+parts))
 if len(candidates)==1:matches.append({'file':str(candidates[0].relative_to(ROOT)),'hash_matches':hashlib.sha256(candidates[0].read_bytes()).hexdigest()==file['sha256']})
summary['provenance_matches']=matches
(OUT/'run_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
# Keep an indexed diagnostic/obstacle/perception subset for matched-frame analysis.
(OUT/'diagnostic_rows.json').write_text(json.dumps({'diag':diag,'obstacles':rows['/rda/obstacles'],'perception':per},separators=(',',':')))
