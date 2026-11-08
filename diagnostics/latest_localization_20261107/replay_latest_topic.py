"""Replay localization receipt order from the original recording, without ROS."""
import json,sys,types,math,collections
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real')]
from rda_dwa_real.localization_input import LocalizationInput
from rda_dwa_real.traj_tracker_core import TrajTrackerCore
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig
from rda_dwa.module_config import module_config
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.ackermann_model import AckermannParams,AckermannModel

class Node:
    def __init__(self):self.params={};self.ros=0.
    def declare_parameter(self,name,value):self.params[name]=value
    def get_parameter(self,name):return types.SimpleNamespace(value=self.params[name])
    def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=round(self.ros*1e9)))
    def get_logger(self):return types.SimpleNamespace(info=lambda *args:None)
node=Node();source=LocalizationInput(node)
pp=module_config('v15_latesc');ack=AckermannParams();planner=RealtimeAckermannPlanner(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=ack)
cfg=BridgeConfig();cfg.localization_timeout=2.
bridge=RealtimeVehicleBridgeCore(planner,pp,config=cfg,ack_model=AckermannModel(ack));tracker=TrajTrackerCore();tracker.p.localization_timeout=2.
c=collections.Counter();pose_ages=[];header_ages=[];bad_jump_after_goal=0
for path in sorted((ROOT/'record_data/run_19700109_151544_181276302').glob('messages_*.jsonl')):
    for line in path.open():
        r=json.loads(line);now=r['monotonic_ns']*1e-9;node.ros=r['ros_receive_ns']*1e-9;p=r['payload']
        if r['topic']=='/move_base_simple/goal':
            pos=p['pose']['position'];bridge.set_goal(pos['x'],pos['y'],frame='map');tracker.set_safety('')
        if r['topic']=='/localization':
            pose=p['pose']['pose'];hdr=p['header']
            pose=types.SimpleNamespace(position=types.SimpleNamespace(**pose['position']),orientation=types.SimpleNamespace(**pose['orientation']))
            hdr=types.SimpleNamespace(frame_id=hdr['frame_id'],stamp=types.SimpleNamespace(**hdr['stamp']))
            good=source.receive_topic(pose,hdr,received_at=now);c['localization_messages']+=1;c['accepted_messages']+=good
            if good:
                xyz,stamp,age=source.sample(2.,now=now)
                bridge.set_localization(*xyz,stamp,arrival=now)
                tracker.set_pose(*xyz,stamp=stamp,arrival=now)
                if bridge._goal_received and (bridge._pose_fault or tracker._safety_reason=='localization_jump'):bad_jump_after_goal+=1
                if not bridge._goal_received:tracker.set_safety('no_goal')
        elif r['topic'] in ('/rda/perception_diag','/rda/control_diag'):
            sample=source.sample(2.,now=now)
            c[r['topic']+'_queries']+=1;c[r['topic']+'_fresh']+=sample is not None
            if sample is not None:
                pose_ages.append(sample[2]);header_ages.append(source.age)
result={'counts':dict(c),'receipt_age_max_s':max(pose_ages),'source_header_age_max_s':max(header_ages),'post_goal_pose_fault_ticks':bad_jump_after_goal,'note':'Original receipt order through latest-topic cache and production pose checks. Does not replay planning, point clouds, or vehicle motion.'}
Path(__file__).with_name('latest_topic_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
