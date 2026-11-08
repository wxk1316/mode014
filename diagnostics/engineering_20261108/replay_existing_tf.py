"""Inspect the last 50 original clouds with the active perception/planner code.

Historical recording only; not a replay of the current vehicle run.
"""
import sys,types,json,sqlite3,bisect,math,time,collections,argparse
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/rda_dwa_real/test'),str(ROOT/'diagnostics/rviz_no_motion_20261107')]
from test_localization_input import LocalizationInputTests
from analyze_recording import CDR,latest_pose
from rda_dwa.motion_safety import TrackedObstacle
LocalizationInputTests.setUpClass();test=LocalizationInputTests()
lidar=test.node('lidar_track_node','LidarTrackNode',source='latest_available');bridge=test.node('bridge_node','RealVehicleBridgeNode')
ap=argparse.ArgumentParser();ap.add_argument("--limit",type=int,default=50);args=ap.parse_args()
loc=[]
for line in (ROOT/'record_data/run_19700109_151544_181276302/messages_000001.jsonl').open():
 r=json.loads(line)
 if r['topic']=='/localization':loc.append((r['ros_receive_ns'],r['payload']))
keys=[a for a,b in loc]
# Offline TF chain composition from transforms received before each cloud.
tf_events=[]
for bag_path in sorted((ROOT/'record_data/run_19700109_151544_181276302/raw_bag').glob('*.db3')):
 with sqlite3.connect('file:'+str(bag_path)+'?mode=ro',uri=True) as db:
  ident=db.execute('select id from topics where name=?',('/tf',)).fetchone()[0]
  tf_events.extend(db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ident,)).fetchall())
histories=collections.defaultdict(list);tf_poses=[]
for tf_received,tf_data in sorted(tf_events,key=lambda row:row[0]):
 for parent,child,source,p,q in CDR(tf_data).transforms():
  h=histories[(parent,child)]
  if not h or source>h[-1][0]:h.append((source,p,q))
  if len(h)>150:del h[:-150]
 result=latest_pose(histories)
 if result is not None:tf_poses.append((tf_received,result))
tf_keys=[r[0] for r in tf_poses]
rows=[]
path=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
with sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True) as db:
 ident=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
 rows=db.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit ?',(ident,args.limit)).fetchall()[::-1]
outputs=[];lidar._publish_obstacles=lambda tracks:outputs.append(tracks)
class Reader:
 def read_points(self,msg,**kwargs):
  dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[next(f.offset for f in msg.fields if f.name==n) for n in ('x','y','z')],'itemsize':msg.point_step})
  return np.frombuffer(msg.data,dtype=dtype)
sys.modules['rda_dwa_real.lidar_track_node']._pc2=Reader()
summary=[];reasons=collections.Counter()
for ts,data in rows:
 c=CDR(data);sec=c.number('i',4);ns=c.number('I',4);frame=c.string();height=c.number('I',4);width=c.number('I',4)
 fields=[]
 for _ in range(c.number('I',4)):
  name=c.string();offset=c.number('I',4);datatype=c.number('B',1);count=c.number('I',4)
  fields.append(types.SimpleNamespace(name=name,offset=offset,datatype=datatype,count=count))
 big=c.number('B',1);step=c.number('I',4);row_step=c.number('I',4);size=c.number('I',4)
 raw=data[c.offset:c.offset+size];c.offset+=size;dense=c.number('B',1);assert c.offset==len(data)
 cloud=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=ns),frame_id=frame),width=width,height=height,fields=fields,point_step=step,data=raw)
 p=loc[bisect.bisect_right(keys,ts)-1][1];hdr=p['header'];pose=p['pose']['pose']
 msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(**hdr['stamp']),frame_id=hdr['frame_id']),pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(**pose['position']),orientation=types.SimpleNamespace(**pose['orientation']))))
 lidar.get_clock=lambda ts=ts:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=ts))
 lidar._cb_odom(msg)
 tf_source,tx,ty,th=tf_poses[bisect.bisect_right(tf_keys,ts)-1][1]
 lidar._localization_input.buffer.transform=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=int(tf_source),nanosec=round((tf_source-int(tf_source))*1e9))),transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=tx,y=ty,z=0.),rotation=types.SimpleNamespace(x=0.,y=0.,z=math.sin(th/2),w=math.cos(th/2))))
 start=time.perf_counter();lidar._cb_cloud(cloud);duration=time.perf_counter()-start
 tracks=outputs[-1] if outputs else []
 objects=[TrackedObstacle(t[1],t[2],t[6],t[7],t[8] if len(t)>8 else t[4],t[3]*math.cos(t[4]),t[3]*math.sin(t[4]),t[0],t[5]=='unknown',1.5 if t[5]=='unknown' else 0.) for t in tracks]
 x,y,yaw=lidar._loc;r=bridge.core.robot;r.x=x-.345*math.cos(yaw);r.y=y-.345*math.sin(yaw);r.theta=yaw;r.v=0.
 bridge.planner.obstacle_age=.15;ok,v,w=bridge.planner.plan(r,(-7.56,-.89),objects)
 reason=bridge.planner.telemetry['reason'];reasons[reason]+=1
 summary.append({'source_stamp':sec+ns*1e-9,'perception_reason':lidar._perception_metrics['reason'],'types':dict(collections.Counter(t[5] for t in tracks)),'planner_reason':reason,'best_gap':bridge.planner.telemetry.get('best_gap'),'perception_duration_s':duration,'localization_source':lidar._localization_input.diagnostic(),'detection_count':lidar._perception_metrics.get('detection_count'),'weak_clusters_removed':lidar._perception_metrics.get('weak_clusters_removed'),'unknown_tracks':[t for t in tracks if t[5]=='unknown']})
front=(x+2.*math.cos(yaw),y+2.*math.sin(yaw));ok,v,w=bridge.planner.plan(r,front,objects)
result={'note':f'Historical last {args.limit} clouds, latest recorded localization, ROS doubles; not current run or vehicle motion. Optional scipy unavailable in this local replay.','planner_reasons':dict(reasons),'first_frames':summary[:3],'last_frames':summary[-3:],'final_forward_probe':{'goal':front,'reason':bridge.planner.telemetry['reason'],'v':v,'reference_points':len(bridge.planner.reference)}}
result['final_tracks']=tracks;result['final_detections']=lidar._perception_metrics.get('detections_world');result['final_perception']=lidar._perception_metrics
result['frames']=summary
Path(__file__).with_name(f'tf_cloud_replay_{args.limit}.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ('planner_reasons','final_forward_probe')},indent=2))
