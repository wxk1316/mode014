"""Finite declared tuning set, no claim of global optimum. Keep failed runs."""
import json,time
from pathlib import Path
from campaign import run,scenes,OUT
names=['pedestrian_0.5','car_1.0','single_barrier','staggered_barriers','campus_planter','lateral_goal']
cases=[c for c in scenes() if c['name'] in names];results=[];configs=[]
for speed,rate in [(.3,30.),(.5,30.),(.7,30.),(.5,40.),(.7,40.),(.7,45.)]:
 label=f'delivery_tune_v{speed}_r{rate}';config={'crab_v_max':speed,'command_steer_rate_deg':rate,'experiment_label':label};group=[]
 for case in cases:
  result=run(case,90.,False,31,config,worker_delay=.25);group.append(result);print(label,case['name'],result['reached'],result['duration_s'],round(result['min_gap_m'],3),flush=True)
 results.append({'config':config,'results':group})
 collisions=sum(r['collision'] or r['min_gap_m']<.12 for r in group)
 failures=sum(not r['reached'] for r in group)
 mean_time=sum(r['duration_s'] for r in group)/len(group)
 switches=sum(len(r['switches']) for r in group)
 configs.append((collisions,failures,mean_time,switches,len(results)-1))
(OUT/'tuning_results.json').write_text(json.dumps(results,indent=2))
best=min(configs);selected=results[best[-1]]['config']
(OUT/'selected_config.json').write_text(json.dumps({'selected':selected,'ranking':configs,'criterion':['collision_or_margin_violation','failure','mean_time','switches'],'scope':'six finite configurations on declared tuning cases; independent campus/noise validation follows'},indent=2))
print('SELECTED',selected,flush=True)
