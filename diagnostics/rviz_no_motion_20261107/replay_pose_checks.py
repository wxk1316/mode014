"""Replay recorded TF poses through production duplicate/jump/age checks."""
import sys,json,pathlib,collections
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from analyze_recording import analyze
from rda_dwa_real.traj_tracker_core import TrajTrackerCore
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig
from rda_dwa.module_config import module_config
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.ackermann_model import AckermannParams,AckermannModel
report,poses=analyze(ROOT/'record_data/run_19700109_151544_181276302')
pp=module_config('v15_latesc');ack=AckermannParams();model=AckermannModel(ack)
planner=RealtimeAckermannPlanner(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=ack)
cfg=BridgeConfig();cfg.localization_timeout=.35
bridge=RealtimeVehicleBridgeCore(planner,pp,config=cfg,ack_model=model);tracker=TrajTrackerCore()
firstgoal=717395.927476285
counts=collections.Counter();last_stamp=None;accepted=0;jump_at=[];bridge_fault_at=[]
for now,stamp,x,y,yaw in poses:
    if not bridge._goal_received and now>=firstgoal:
        bridge.set_goal(10.313,.203,frame='map')
        tracker.set_safety('')  # first new plan replaces the pre-goal stop reason
    if now<firstgoal:
        tracker.set_safety('no_goal')  # same stop lifecycle as the ROS plan callback
    if now-stamp<=.35:
        bridge.set_localization(x,y,yaw,stamp,arrival=stamp)
        tracker.set_pose(x,y,yaw,stamp=stamp,arrival=stamp)
        if stamp!=last_stamp:accepted+=1;last_stamp=stamp
    counts['source_stale']+=now-stamp>.35
    counts['bridge_timeout']+=now-bridge._t_loc>.35
    counts['tracker_timeout']+=now-tracker._pose_at>.35
    if tracker._safety_reason=='localization_jump':jump_at.append(now)
    if bridge._pose_fault:bridge_fault_at.append(now)
result={'queries':len(poses),'new_source_samples':accepted,'stop_counts':dict(counts),'pre_goal_tracker_jump_ticks':sum(t<firstgoal for t in jump_at),'post_goal_tracker_jump_ticks':sum(t>=firstgoal for t in jump_at),'bridge_pose_fault_ticks':len(bridge_fault_at),'first_jump_ros_s':jump_at[0] if jump_at else None,'note':'Production core pose checks on recorded TF composition with a synthetic no-goal/new-goal plan lifecycle; no planning or vehicle simulation.'}
pathlib.Path(__file__).with_name('core_tf_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
