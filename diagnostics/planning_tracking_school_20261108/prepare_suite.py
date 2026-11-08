"""Build a self-contained audit harness from the existing multimode campaign.
Only diagnostics are written. All algorithm imports use current src/.
"""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
s = (ROOT/'diagnostics/multimode_20261108/campaign.py').read_text()
def change(old, new):
    global s
    assert old in s, old[:100]
    s = s.replace(old, new)
change("BASELINE='--baseline' in sys.argv", "BASELINE=False")
change('CONFIG=yaml.safe_load', 'from cloud_pipeline import CloudPipeline\nCONFIG=yaml.safe_load')
change("driver=None if BASELINE else DriverPolicy();", "cloud=CloudPipeline(CONFIG,case,noise,seed);perception=[];observations=collections.deque();next_cloud=0.;tracks=[];\n driver=None if BASELINE else DriverPolicy();")
change("  if tick%5==0:\n   age=.12 if noise else 0.", """  if t>=next_cloud-1e-9:
   dets,metrics,processing=cloud.process(t,(x,y,yaw))
   perception.append(dict(t=t,**metrics))
   if metrics['valid'] and not (noise and int(round(t*10))%37==11):
    observations.append((t+max(.1,processing),t,dets))
   next_cloud=round(next_cloud+.1,8)
  while observations and observations[0][0]<=t+1e-9:
   _,stamp,dets=observations.popleft()
   tracks=cloud.track(dets,stamp)
   packet=[(q[1],q[2],q[5],q[6],q[7],q[4],q[3],q[0],q[8] if len(q)>8 else q[4]) for q in tracks]
   core.set_obstacles(packet,stamp=100.+stamp,arrival=t,source_age=t-stamp)
   tracker.set_obstacles(core._last_received_obs,stamp,stamp=100.+stamp)
  if tick%5==0:
   age=.12 if noise else 0.""")
change("   if not(noise and tick%35==0):\n    tracker.set_obstacles(observed,t-age,stamp=100.+t-age);core._last_received_obs=observed;core._t_obs=t-age", "   # Obstacles come exclusively from the project clustering + KF pipeline.")
change("  if tick%5==0:rows.append", "  if True:rows.append")
change("planner=info.get('planner',{})))", "planner=info.get('planner',{}),recovery_goal=info.get('recovery_goal'),mode_epoch=info.get('mode_epoch'),spin_session=info.get('spin_session'),tracks=tracks))")
change("  if gap<0:collided=True;break", "  if gap<0:collided=True;break\n  if t>15. and info.get('reason') in ('recovery_exhausted','recovery_no_safe_heading','bounded_static_no_route') and abs(v)<.01 and abs(omega)<.01:break")
change("scope='preserved baseline", "scope='preserved baseline")
change("no ROS/UDP or real lidar')", "no ROS/UDP; synthetic per-frame lidar through production clustering and KF')")
change(" if driver is not None:driver.close()", """ result.update(perception_valid_frames=sum(q['valid'] for q in perception),perception_frames=len(perception),
  perception_p95_ms=float(np.percentile(cloud.durations,95)*1000),max_detection_count=max(q.get('detection_count',0) for q in perception),
  mode_edges=sum(rows[i]['mode']!=rows[i-1]['mode'] for i in range(1,len(rows))),
  trap_edges=sum(rows[i]['trap']!=rows[i-1]['trap'] for i in range(1,len(rows))),
  outcome=('collision' if collided else 'clearance_below_12cm' if min_gap<.12-1e-6 else 'reached' if info.get('reached') else 'safe_blocked' if case.get('blocked') and abs(v)<.02 else 'not_reached'))
 if driver is not None:driver.close()""")
change("(folder/(case['name']+'_result.json')).write_text", "(folder/(case['name']+'_perception.json')).write_text(json.dumps(perception,allow_nan=False))\n  (folder/(case['name']+'_result.json')).write_text")
change(" cases=scenes();", " from cloud_pipeline import school_scenes\n cases=school_scenes(scenes(),TrackedObstacle);")
change("(OUT/(('baseline' if BASELINE else 'candidate')+('_noise' if args.noise else '')+'_results.json'))", "(OUT/(('candidate')+('_noise' if args.noise else '')+f'_{args.seed}_results.json'))")
# Independent geometric checks at 100 Hz, with two actuator integration substeps.
start=s.index("  desired=last['speed'] if active")
end=s.index("  min_gap=min(min_gap,gap)",start)
block=s[start:end]
integration=block[:block.index('  body=corners')].replace('*.02','*.01').replace('yaw+w*.01','yaw+w*.005')
oracle=block[block.index('  body=corners'):].replace('(t+.02)','(t+sub*.01+.01)')
s=s[:start]+"  gap=math.inf\n  for sub in range(2):\n"+'\n'.join(' '+line for line in (integration+oracle).splitlines())+"\n   min_gap=min(min_gap,gap)\n   if gap<0:break\n"+s[end:]
change("x<case['exit_x']", "max(q[0] for q in body)<-1.22")
(OUT/'run_suite.py').write_text(s)
(OUT/'driver_bridge.cpp').write_text((ROOT/'diagnostics/multimode_20261108/driver_bridge.cpp').read_text())
