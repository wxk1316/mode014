"""Single open-loop escape hypotheses; not DWA recursive safety or vehicle tests.

Includes .3s stop confirmation hypothesis plus >=3s wheel change and recovery.
All mode speeds/acceleration use the existing software bounds. CRAB negative
speed, lateral sign and ready IDs remain physically unverified. A safe scheduled
primitive does not guarantee a safe stop at every intermediate state or goal
arrival. This script constructs in-memory ROS doubles and sends no UDP.
"""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'docs/validation'),str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from check_ros_adapters import install_doubles
install_doubles()
from rda_dwa_real.bridge_node import RealVehicleBridgeNode
from rda_dwa.hybrid_primitives import evaluate_maneuver
from rda_dwa.robot_model import RobotState
from rda_dwa.obstacle import new_obstacle
cc=RealVehicleBridgeNode().planner.cc
rows=[]
actions=[('ACK_turn','ACK',.4,math.radians(30),5.,0.),('SPIN_only','SPIN',0.,0.,2.,0.),('CRAB_forward_45','CRAB',.4,0.,5.,math.pi/4),('CRAB_reverse_45','CRAB',-.3,0.,8.,math.pi/4),('TRANSLATE','TRANSLATE',.4,0.,5.,math.pi/2)]
for distance in (5.,7.):
 obs=[new_obstacle(id_=1,x=distance,y=0.,heading=math.pi,speed=.7,length=.5,width=.5,obstacle_type='pedestrian')]
 for name,mode,v,delta,T,alpha in actions:
  r=evaluate_maneuver(cc,RobotState(),obs,mode=mode,speed=v,steering=delta,motion_time=T,yaw_change=math.pi/2 if mode=='SPIN' else 0.,translation_angle=alpha,simulation=mode in ('CRAB','TRANSLATE'),ready_at_s=3.3,wait_min_s=3.3)
  rows.append({'initial_actor_range_m':distance,'action':name,'status':r.status,'scheduled_model_safe':r.safe,'minimum_gap_m':r.minimum_gap_m,'schedule_time_s':r.nominal_duration_s,'terminal_pose':r.trajectory[-1].tolist() if len(r.trajectory) else None,'protocol_verified':r.protocol_verified,'navigation_success':None,'recursive_safe_backup':None})
print(json.dumps({'date':'2026-11-06','scope':__doc__,'rows':rows},indent=2,allow_nan=False))
