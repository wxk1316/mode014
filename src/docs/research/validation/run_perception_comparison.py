"""Same-input KF baseline, motion-guard ablation and motion-risk probes.

Pure Python, synthetic observations and oracle future positions. No ROS/UDP.
"""
from pathlib import Path
import sys,hashlib,json,zipfile,types,time,math
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from rda_dwa_real.kf_tracker import GlobalKFTracker

baseline_path='rda_dwa_real/rda_dwa_real/kf_tracker.py'
with zipfile.ZipFile(ROOT/'docs/research/baseline_sources.zip') as z:
 source=z.read(baseline_path)
old=types.ModuleType('perception_exact_baseline');exec(compile(source,baseline_path,'exec'),old.__dict__)
single=types.ModuleType('perception_single_step_ablation')
current_source=(ROOT/baseline_path).read_text().replace('raw_speed = float(np.linalg.norm(window_velocity))','window_velocity = raw_velocity.copy()\n        raw_speed = float(np.linalg.norm(window_velocity))')
exec(compile(current_source,'single_step_motion_guard','exec'),single.__dict__)
models={'baseline':old.GlobalKFTracker,'single_step_guard':single.GlobalKFTracker,'vectorized_guard_off':lambda **kw:GlobalKFTracker(motion_guard_enable=False,**kw),'patched':GlobalKFTracker}

def obs(x,y=2):return (float(x),float(y),'pedestrian',.3,.3)
def probe(factory,sequence,dt=.1):
 tracker=factory(dt=dt,lost_ttl_seconds=.5)
 times=[];records=[]
 for i,ds in enumerate(sequence):
  t=time.perf_counter_ns();out=tracker.step(ds,stamp=100+i*dt);times.append((time.perf_counter_ns()-t)*1e-6)
  records.append(out)
 return {'frames':len(sequence),'peak_tracks':max(map(len,records)),
         'known_nonzero_frames':sum(any(s[5]!='unknown' and s[3]>.05 for s in states) for states in records),
         'unknown_frames':sum(any(s[5]=='unknown' for s in states) for states in records),
         'max_known_speed_mps':max((s[3] for states in records for s in states if s[5]!='unknown'),default=0.0),
         'last':records[-1],'timing_ms':{str(q):float(np.percentile(times,q)) for q in (50,95,99,100)},
         'deadline_100ms_misses':sum(t>100 for t in times)},records

def main():
 rng=np.random.default_rng(20261106)
 scenarios={
  'constant_forward':([ [obs(2+i*.1)] for i in range(40)],.1),
  'constant_reverse':([ [obs(2-i*.1)] for i in range(40)],.1),
  'partial_surface_shift':([[obs(2)],[obs(2.25)]],.05),
  'large_spatial_misassociation':([[obs(2)],[obs(2.8)]],.05),
  'sudden_reversal':([[obs(2+i*.1)] for i in range(8)]+[[obs(2.6-i*.1)] for i in range(8)],.1),
  'stationary_centroid_noise_1cm_10hz':([[obs(2+rng.normal(0,.01),2+rng.normal(0,.01))] for _ in range(80)],.1),
  'stationary_centroid_noise_4cm_20hz':([[obs(2+rng.normal(0,.04),2+rng.normal(0,.04))] for _ in range(80)],.05),
  'occlusion':([[obs(2+i*.1)] for i in range(4)]+[[] for _ in range(8)],.1),
 }
 result={'scope':'Synthetic source-time detections + oracle future for a reversal; macOS offline KF only, excludes cloud decoding/filter/clustering/tiling/ROS/DDS. Config limits are operating assumptions, not calibration.','date':'2026-11-06','baseline_sha256':hashlib.sha256(source).hexdigest(),'scenario_input_sha256':hashlib.sha256(json.dumps(scenarios,sort_keys=True).encode()).hexdigest(),'scenarios':{}}
 for name,(sequence,dt) in scenarios.items():
  result['scenarios'][name]={}
  for label,factory in models.items():
   summary,records=probe(factory,sequence,dt)
   if name=='sudden_reversal':
    state=records[8][0];horizons=np.linspace(0,3,61);x0=2.6
    radius=math.hypot(state[6],state[7])/2
    checks=[]
    for t in horizons:
     predicted_x=state[1]+state[3]*math.cos(state[4])*t
     predicted_y=state[2]+state[3]*math.sin(state[4])*t
     true_x=x0-t;true_y=2
     rate=1.5 if state[5]=='unknown' else .05
     checks.append(math.hypot(predicted_x-true_x,predicted_y-true_y)<=radius+rate*t+1e-9)
    summary['oracle_constant_reverse_future_containment']={'covered_samples':int(sum(checks)),'total_samples':len(checks),'horizon_seconds':3,'kind':state[5],'speed':state[3],'note':'future oracle condition checks only: not vehicle collision/success validation'}
   result['scenarios'][name][label]=summary
 # Independent tracks spaced 3m apart; same moving input for every variant.
 result['tracking_performance']={}
 for count in (20,120):
  sequence=[[obs(3*j+.2*i*.05,2+3*(j%2)) for j in range(count)] for i in range(180)]
  result['tracking_performance'][str(count)]={}
  for label,factory in models.items():
   _,_=probe(factory,sequence[:10],.05) # isolated warm-up
   summary,_=probe(factory,sequence,.05)
   result['tracking_performance'][str(count)][label]=summary
 out=ROOT/'docs/research/perception_comparison.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps({'saved':str(out),'baseline_sha256':result['baseline_sha256'],'performance':{n:{k:v['timing_ms'] for k,v in rows.items()} for n,rows in result['tracking_performance'].items()}},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
