"""Actual multiprocessing spawn and mode-epoch handoff; no ROS or vehicle."""
import sys,time,json,math
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.multimode_planner import MultiModePlanner
from rda_dwa_real.async_planner import AsyncPlanner


def main():
 p=module_config('v15_latesc');p.velocity_horizon_targets=True;p.max_linear_velocity=.7;p.dt=.1
 base=MultiModePlanner(RealtimeAckermannPlanner(p,'ENHANCED_DWA',collision_model='rectangle',ack_params=AckermannParams()))
 planner=AsyncPlanner(base);r=SimpleNamespace(x=-.345,y=0.,theta=0.,v=0.,planning_stamp=0.)
 records=[]
 try:
  for mode,goal,beta in [(0,(5.,0.),0.),(1,(0.,1.2),math.pi/2)]:
   planner.current_mode=mode;planner.mode_epoch=mode;planner.current_steer=beta
   deadline=time.monotonic()+8.;ok=False
   while time.monotonic()<deadline:
    r.planning_stamp=time.monotonic();ok,v,w=planner.plan(r,goal,[])
    if ok:break
    time.sleep(.03)
   assert ok,planner.telemetry
   assert planner.telemetry.get('motion_mode')==('ACK' if mode==0 else 'CRAB'),planner.telemetry
   records.append({'mode':mode,'ok':ok,'telemetry':planner.telemetry})
 finally:planner.close()
 Path(__file__).with_name('real_worker_smoke_results.json').write_text(json.dumps(records,indent=2,allow_nan=False));print('actual spawned ACK/CRAB worker PASS')

if __name__=='__main__':main()
