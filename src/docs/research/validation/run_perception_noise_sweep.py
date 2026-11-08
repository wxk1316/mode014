"""Seed and source-frequency sensitivity, not sensor-noise calibration."""
from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_perception_comparison import models,probe,obs
rows=[]
for sigma,dt in ((.01,.1),(.04,.05),(.08,.05)):
 for window in (.1,.2,.3):
  counts=[];static=0
  for seed in range(20):
   rng=np.random.default_rng(seed)
   sequence=[[obs(2+rng.normal(0,sigma),2+rng.normal(0,sigma))] for _ in range(100)]
   summary,_=probe(lambda **kw:models['patched'](velocity_reconfirm_time=window,**kw),sequence,dt)
   counts.append(summary['unknown_frames']);static+=summary['last'][0][5]=='block'
  rows.append({'centroid_noise_sigma_m':sigma,'source_dt_seconds':dt,'window_seconds':window,
               'seeds':20,'frames_per_seed':100,'mean_unknown_frames':float(np.mean(counts)),
               'max_unknown_frames':max(counts),'final_static_trials':static})
out=ROOT/'docs/research/perception_noise_sweep.json';out.write_text(json.dumps({'scope':'Synthetic Gaussian centroid noise over 20 seeds; no annotation/sensor calibration; tests source-time window sensitivity.','rows':rows},indent=2));print(json.dumps(rows,indent=2))
