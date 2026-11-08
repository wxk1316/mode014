"""Compare single maneuver capabilities with full waiting/footprint risk.

CRAB/TRANSLATE are explicit kinematic simulation hypotheses, disabled in the
production protocol. These are capability comparisons, not optimized equal-goal
navigation trials. No ROS, DDS, wheel commands or UDP are emitted.
"""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'docs/validation'),str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from check_ros_adapters import install_doubles
install_doubles()
from rda_dwa_real.bridge_node import RealVehicleBridgeNode
from rda_dwa.hybrid_primitives import evaluate_maneuver,DEFAULT_PROTOCOLS
from rda_dwa.robot_model import RobotState
from rda_dwa.obstacle import new_obstacle
cc=RealVehicleBridgeNode().planner.cc
scenes={'open':[],
 'narrow_corridor':[new_obstacle(id_=1,x=0,y=.85,length=5,width=.2,obstacle_type='block'),new_obstacle(id_=2,x=0,y=-.85,length=5,width=.2,obstacle_type='block')],
 'waiting_approach':[new_obstacle(id_=3,x=3,y=0,heading=math.pi,speed=.7,length=.5,width=.5,obstacle_type='pedestrian')]}
actions=[('ACK_FORWARD','ACK',.3,math.radians(30),0,3.,0),('ACK_REVERSE','ACK',-.2,math.radians(30),0,3.,0),('SPIN_ALIGN','SPIN',0.,0.,math.pi/2,2.,0),('CRAB','CRAB',.2,0.,0.,4.14,math.pi/4),('TRANSLATE','TRANSLATE',.2,0.,0.,4.14,math.pi/2)]
results=[]
for scene,obs in scenes.items():
 for name,mode,v,delta,yaw,T,alpha in actions:
  result=evaluate_maneuver(cc,RobotState(),obs,mode=mode,speed=v,steering=delta,yaw_change=yaw,motion_time=T,translation_angle=alpha,simulation=mode in ('CRAB','TRANSLATE'))
  points=result.trajectory;phases=result.phases
  end=points[-1].tolist() if len(points) else None
  path=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
  results.append({'scene':scene,'action':name,'safe_model':result.safe,'status':result.status,'terminal_pose_m_rad':end,'path_m':path,'duration_s':result.nominal_duration_s,'minimum_model_gap_m':result.minimum_gap_m,'waiting_s':sum(p in ('WAIT_READY','RECOVERY_WAIT') for p in phases)*.05,'mode_switches_nominal':0 if mode=='ACK' else 2,'protocol_verified':result.protocol_verified,'deployment_enabled':DEFAULT_PROTOCOLS[mode].enabled,'target_lateral_pose_error_m':None if end is None else math.hypot(end[0],end[1]-.8)})
reject={m:evaluate_maneuver(cc,RobotState(),[],mode=m,simulation=False).status for m in ('CRAB','TRANSLATE')}
print(json.dumps({'date':'2026-11-06','scope':__doc__,'production_unknown_protocol_rejection':reject,'results':results},indent=2,allow_nan=False))
