"""20 Hz raw clouds through the actual spawned perception process, no ROS."""
import sys,time,json,sqlite3,statistics,types
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(ROOT/'diagnostics/engineering_20261108')]
from cloud_stats import decode_cloud
from rda_dwa_real.async_perception import AsyncPerception,cloud_packet
from rda_dwa_real.perception_core import PerceptionParams
from rda_dwa_real.kf_tracker import GlobalKFTracker


def run():
 path=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
 with sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True) as db:
  tid=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
  data=db.execute('select data from messages where topic_id=? order by timestamp desc limit 30',(tid,)).fetchall()[::-1]
 clouds=[cloud_packet(decode_cloud(row[0])) for row in data]
 worker=AsyncPerception(PerceptionParams(max_range=8.,y_min=-8.,y_max=8.),GlobalKFTracker(dt=.05))
 samples=[];submissions=[];seq=0;begin=time.monotonic();next_frame=begin
 try:
  while time.monotonic()-begin<3.5:
   now=time.monotonic()
   if now>=next_frame:
    seq+=1;stamp=777000.+(now-begin);cloud=dict(clouds[(seq-1)%len(clouds)]);cloud['sec']=int(stamp);cloud['nanosec']=round((stamp-int(stamp))*1e9)
    start=time.monotonic();worker.submit((seq,now,cloud,(.65,.1,-.022),{'source_stamp':stamp,'source_age_at_receive_s':.01}));submissions.append(time.monotonic()-start);next_frame=now+.05
   result=worker.poll()
   if result is not None:
    n,received,sec,ns,tracks,m=result;age=.01+time.monotonic()-received
    samples.append({'sequence':n,'source_age_s':age,'worker_duration_s':m['worker_duration_s'],'queue_delay_s':m['worker_queue_delay_s'],'reason':m['reason'],'track_reset':m.get('track_reset'),'unknown_count':m.get('unknown_count'),'published_eligible':tracks is not None and age<=.4})
   time.sleep(.01)
 finally:worker.close()
 durations=[s['worker_duration_s'] for s in samples if s['reason']=='published'];warm=[s for s in samples if s['reason']=='published' and s['sequence']>10]
 report={'platform':'local Mac, spawned worker with numeric threads=1; not Jetson/DDS','input_frames':seq,'completed_frames':len(samples),'post_start_eligible_fraction':sum(s['published_eligible'] for s in warm)/len(warm),'submit_p95_s':float(np.percentile(submissions,95)),'worker_duration_p50_s':float(np.median(durations)),'worker_duration_p95_s':float(np.percentile(durations,95)),'warm_source_age_p95_s':float(np.percentile([s['source_age_s'] for s in warm],95)),'track_resets':sum(bool(s['track_reset']) for s in samples),'last_unknown_count':samples[-1].get('unknown_count'),'samples':samples}
 (OUT/'worker_profile.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
if __name__=='__main__':run()
