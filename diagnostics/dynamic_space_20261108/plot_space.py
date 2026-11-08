import json,math,sys,types
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.ackermann_model import AckermannParams
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.motion_safety import TrackedObstacle,rollout_stop,to_centres,VehicleGeometry
frames=json.loads((OUT/'blocking_obstacles.json').read_text())['frames']
sample=next(r for r in frames if any(o['current_gap_m']>2 and o['best_candidate_gap_m']<.12 for o in r['unknowns']))
x,y,yaw=sample['robot'];centre=np.array([x+.345*math.cos(yaw),y+.345*math.sin(yaw)])
fwd=np.array([math.cos(yaw),math.sin(yaw)]);left=np.array([-fwd[1],fwd[0]])
def body(points):
 r=np.asarray(points)-centre;return np.column_stack((r@fwd,r@left))
pp=module_config('v15_latesc');pp.dt=.1;pp.sim_time=3.;pp.max_linear_velocity=1.;pp.planning_budget_s=.5;pp.velocity_horizon_targets=True;pp.steer_weight=.4;pp.steer_change_weight=.12
planner=RealtimeAckermannPlanner(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=AckermannParams(length=1.2,width=.8,wheelbase=.69))
obs=[]
for q in sample['obstacles']:
 hd=q['shape_heading'] if q['has_shape_heading'] else q['heading'];sp=q['speed'];direction=q['heading']
 obs.append(TrackedObstacle(q['x'],q['y'],q['length'],q['width'],hd,sp*math.cos(direction),sp*math.sin(direction),q['id'],q['type']=='unknown',1.5 if q['type']=='unknown' else 0.))
planner.obstacle_age=.13;robot=types.SimpleNamespace(x=x,y=y,theta=yaw,v=0.,planning_stamp=sample['t']);ok,v,w=planner.plan(robot,sample['goal'],obs)
stop_horizon=planner.telemetry.get('unknown_stop_horizon_s',1.) or 1.
fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax,h,title in zip(axes,(3.,stop_horizon),('Original 3 s unknown envelope','Action + checked braking envelope')):
 for o in obs:
  c,s=math.cos(o.heading),math.sin(o.heading);corners=np.array([[o.length/2,o.width/2],[-o.length/2,o.width/2],[-o.length/2,-o.width/2],[o.length/2,-o.width/2]])@np.array([[c,s],[-s,c]])+np.array([o.x,o.y])
  ax.add_patch(Polygon(body(corners),fill=False,color='orange' if o.uncertain else 'green',lw=.8))
  if o.uncertain:
   a=np.linspace(.0001,2*math.pi-.0001,160);radius=1.5*(h+.13)
   expanded=np.column_stack((o.length/2*np.sign(np.cos(a))+radius*np.cos(a),o.width/2*np.sign(np.sin(a))+radius*np.sin(a)))@np.array([[c,s],[-s,c]])+np.array([o.x,o.y])
   ax.add_patch(Polygon(body(expanded),color='red',alpha=.10,lw=.3))
   pos=body([[o.x,o.y]])[0];ax.text(*pos,f'{o.ident}',fontsize=7,color='darkorange')
 ax.add_patch(Polygon([[-.6,-.4],[.6,-.4],[.6,.4],[-.6,.4]],color='tab:blue'))
 ax.arrow(0.,0.,1.3,0.,width=.03,color='tab:blue');ax.set(xlim=(-9,9),ylim=(-9,9),xlabel='vehicle forward [m]',ylabel='vehicle left [m]',title=title);ax.set_aspect('equal');ax.grid(alpha=.3)
if planner.reference:
 path=body([[p[0]+.345*math.cos(p[2]),p[1]+.345*math.sin(p[2])] for p in planner.reference]);axes[1].plot(path[:,0],path[:,1],color='navy',lw=2,label='nominal reference')
 d=planner.telemetry['selected_delta'];rear,t=rollout_stop(x,y,yaw,0.,0.,[v],[d],VehicleGeometry(),3.,.05,reaction=.4,jerk=2.,steer_accel=math.radians(100));path=body(to_centres(rear,VehicleGeometry())[0,:,:2]);axes[1].plot(path[:,0],path[:,1],'--',color='black',label='braking backup');axes[1].legend(fontsize=8)
fig.suptitle(f'Recorded scene t={sample["t"]:.1f}s; orange=unknown boxes, green=confirmed boxes, red=reachability\nNew frozen-snapshot result: {planner.telemetry["reason"]}, target={v:.2f} m/s; illustrative envelope times, not hardware motion',fontsize=10)
fig.tight_layout();fig.savefig(OUT/'unknown_space_comparison.png',dpi=160)
(OUT/'illustrated_frame.json').write_text(json.dumps({'t':sample['t'],'planner':planner.telemetry,'critical_unknowns':sample['unknowns'],'pose':sample['robot']},indent=2))
print(json.dumps({'t':sample['t'],'result':planner.telemetry['reason'],'speed':v,'stop_horizon':stop_horizon},indent=2))
