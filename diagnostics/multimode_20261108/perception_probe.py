"""Actual KF association/velocity with synthetic detection rectangles.
This evaluates the tracker component, not raw-lidar segmentation or road edges.
"""
import sys,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa')]
from rda_dwa_real.kf_tracker import GlobalKFTracker
results=[]
for speed,size,label in [(.4,.45,'pedestrian'),(.5,.45,'pedestrian'),(.5,.8,'vehicle'),(1.,.8,'vehicle')]:
 rng=np.random.default_rng(41);tracker=GlobalKFTracker();t=0.;rows=[]
 for i in range(80):
  dt=float(rng.choice([.06,.08,.12]));t+=dt;x=4.-speed*t
  detections=[] if i%11==7 else [(x+rng.normal(0,.015),rng.normal(0,.015),label,size,size)]
  tracked=tracker.step(detections,dt=dt)
  if tracked:
   out=tracked[0];rows.append({'t':t,'true_x':x,'estimated_x':float(out[1]),'estimated_speed':float(out[3]),'type':out[5],'missed':not detections})
 errors=[abs(r['estimated_speed']-speed) for r in rows if r['t']>1.5]
 result={'true_speed':speed,'input_label':label,'mean_speed_error_mps':float(np.mean(errors)),'p95_speed_error_mps':float(np.percentile(errors,95)),'missed_detection_predictions':sum(r['missed'] for r in rows),'rows':rows,'scope':'actual GlobalKFTracker with synthetic noisy rectangles; no raw lidar'}
 results.append(result)
Path(__file__).with_name('perception_probe_results.json').write_text(json.dumps(results,indent=2));print([(r['true_speed'],round(r['mean_speed_error_mps'],3)) for r in results])
