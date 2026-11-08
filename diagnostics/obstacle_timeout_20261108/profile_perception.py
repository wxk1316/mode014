"""Identical-input geometry and phase timing comparison, no ROS/hardware."""
import sys,time,sqlite3,importlib.util,json,collections
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(ROOT/'diagnostics/engineering_20261108')]
from cloud_stats import decode_cloud,xyz
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams
spec=importlib.util.spec_from_file_location('rda_dwa_real.perception_before',OUT/'before/src/rda_dwa_real/rda_dwa_real/perception_core.py');old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
path=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
with sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True) as db:
 tid=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
 rows=db.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit 30',(tid,)).fetchall()[::-1]
cores=[old.LidarPerceptionCore(old.PerceptionParams(max_range=8.,y_min=-8.,y_max=8.)),LidarPerceptionCore(PerceptionParams(max_range=8.,y_min=-8.,y_max=8.))]
outputs=[];differences=[]
for ts,data in rows:
 points=xyz(decode_cloud(data));values=[];timings=[];metrics=[]
 for core in cores:
  t=time.perf_counter();det,m=core.process(points,ts*1e-9,pose=(.65,.1,-.022));timings.append(time.perf_counter()-t);metrics.append(m);values.append(det)
 # Compare independent geometric centres/dimensions up to sign-equivalent PCA.
 unmatched=list(values[1]);max_change=0.
 for d in values[0]:
  k=min(range(len(unmatched)),key=lambda j:np.linalg.norm(np.asarray(d[:2])-np.asarray(unmatched[j][:2]))) if unmatched else None
  if k is None:max_change=float('inf');break
  q=unmatched.pop(k);max_change=max(max_change,float(np.max(np.abs(np.asarray([d[0],d[1],d[3],d[4]])-np.asarray([q[0],q[1],q[3],q[4]])))))
 differences.append(max_change)
 outputs.append({'old_s':timings[0],'new_s':timings[1],'old_detections':len(values[0]),'new_detections':len(values[1]),'box_change_m':max_change,'new_phases':{k:v for k,v in metrics[1].items() if 'duration' in k},'voxels':metrics[1].get('occupied_voxels'),'representatives':metrics[1].get('cluster_input_points')})
result={'platform':'local Mac, not Jetson','frames':len(outputs),'before_p50_s':float(np.median([r['old_s'] for r in outputs])),'after_p50_s':float(np.median([r['new_s'] for r in outputs])),'before_p95_s':float(np.percentile([r['old_s'] for r in outputs],95)),'after_p95_s':float(np.percentile([r['new_s'] for r in outputs],95)),'max_box_change_m':max(differences),'same_detection_counts':all(r['old_detections']==r['new_detections'] for r in outputs),'frames_detail':outputs}
(OUT/'profile_comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='frames_detail'},indent=2))
