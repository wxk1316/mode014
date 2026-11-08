"""Held-out noisy inputs and irregular feedback; not a real lidar test."""
import json
from campaign import run,scenes,OUT
results=[]
for seed in (223,347,509):
 group=[]
 for case in scenes():
  r=run(case,90.,True,seed,worker_delay=.25);group.append(r);print(seed,case['name'],r['reached'],r['collision'],round(r['min_gap_m'],3),flush=True)
 results.append({'seed':seed,'results':group});(OUT/'robustness_final_results.json').write_text(json.dumps(results,indent=2))
