"""Fresh source stamps, bounded latest inputs, nonblocking ROS, tracking gaps."""
from pathlib import Path
import sys,math,time,queue,types,unittest,json
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real/test')]
from rda_dwa_real.async_perception import AsyncPerception,PerceptionWorkerCore,replace_pending,cloud_packet
from rda_dwa_real.perception_core import PerceptionParams,cell_index,voxel_components
from rda_dwa_real.kf_tracker import GlobalKFTracker
from rda_dwa_real.obstacle_fusion import Clusterer
import test_localization_input as localization_tests


def cloud(stamp=100.):
    xyz=np.array([[x,y,z] for x in np.linspace(1.9,2.1,5) for y in np.linspace(-.15,.15,5) for z in (-.2,.1)],dtype='<f4')
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)),frame_id='rslidar'),height=1,width=len(xyz),point_step=12,row_step=len(xyz)*12,data=xyz.tobytes(),is_bigendian=False,fields=[types.SimpleNamespace(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])

def job(sequence=1,stamp=100.,age=.01):
    return sequence,time.monotonic(),cloud_packet(cloud(stamp)),(0.,0.,0.),dict(source_stamp=stamp,source_age_at_receive_s=age)


class AsyncPerceptionTests(unittest.TestCase):
    def test_waiting_slot_replaces_older_input_without_growing(self):
        channel=queue.Queue(1);replace_pending(channel,1);replace_pending(channel,2);replace_pending(channel,3)
        self.assertEqual(channel.qsize(),1);self.assertEqual(channel.get(),3)

    def test_155_second_interval_does_not_reset_tracks_every_frame(self):
        core=PerceptionWorkerCore(PerceptionParams(ground_enabled=False),GlobalKFTracker(static_hold_frames=3),reset_gap=2.)
        for i in range(5):result=core.process(job(i,100.+i*1.55))
        tracks=result[-2];self.assertEqual(len(tracks),1);self.assertEqual(tracks[0][0],1)
        self.assertFalse(result[-1]['track_reset']);self.assertEqual(tracks[0][5],'block')
        result=core.process(job(6,110.))
        self.assertTrue(result[-1]['track_reset'])

    def test_expired_waiting_observation_is_not_processed_as_empty_scene(self):
        core=PerceptionWorkerCore(PerceptionParams(ground_enabled=False),GlobalKFTracker())
        j=job();j=(j[0],j[1]-.6,*j[2:]);r=core.process(j)
        self.assertIsNone(r[-2]);self.assertEqual(r[-1]['reason'],'cloud_expired_in_queue')

    def test_failed_ground_does_not_publish_empty_obstacles(self):
        core=PerceptionWorkerCore(PerceptionParams(),GlobalKFTracker())
        r=core.process(job());self.assertIsNone(r[-2]);self.assertEqual(r[-1]['reason'],'ground_unavailable')

    def test_actual_spawn_keeps_source_timestamp_and_returns_supported_obstacle(self):
        worker=AsyncPerception(PerceptionParams(ground_enabled=False),GlobalKFTracker())
        try:
            deadline=time.monotonic()+5.;sequence=0;result=None;submit_durations=[]
            while time.monotonic()<deadline:
                sequence+=1;start=time.monotonic();worker.submit(job(sequence,100.+sequence*.05));submit_durations.append(time.monotonic()-start)
                result=worker.poll()
                if result is not None and result[-2] is not None:break
                time.sleep(.02)
            self.assertIsNotNone(result);self.assertTrue(result[-2]);self.assertEqual(result[-1]['reason'],'published')
            self.assertEqual(result[2]+result[3]*1e-9,result[-1]['source_stamp'])
            self.assertLess(float(np.percentile(submit_durations,95)),.05)
        finally:worker.close()

    def test_expired_finished_result_does_not_renew_obstacle_stamp(self):
        localization_tests.LocalizationInputTests.setUpClass();helper=localization_tests.LocalizationInputTests()
        node=helper.node('lidar_track_node','LidarTrackNode');output=[];node._publish_obstacles=output.append
        payload=(1,time.monotonic()-.7,999,990000000,[],{'source_age_at_receive_s':.01,'reason':'published'})
        node._async_perception=types.SimpleNamespace(poll=lambda:payload,submitted=1,skipped=0)
        node._poll_perception()
        self.assertEqual(output,[]);self.assertEqual(node._perception_metrics['reason'],'perception_result_stale')
        self.assertFalse(node._perception_metrics['obstacle_published'])

    def test_fresh_result_publishes_original_cloud_stamp(self):
        localization_tests.LocalizationInputTests.setUpClass();helper=localization_tests.LocalizationInputTests()
        node=helper.node('lidar_track_node','LidarTrackNode')
        payload=(1,time.monotonic(),999,980000000,[],{'source_age_at_receive_s':.02,'reason':'empty_cloud_predicted_tracks'})
        node._async_perception=types.SimpleNamespace(poll=lambda:payload,submitted=1,skipped=0)
        with patch('rda_dwa_real.lidar_track_node.MarkerArray',side_effect=lambda:types.SimpleNamespace(markers=[])):
            node._poll_perception()
        sent=node._pub_obs.messages[-1]
        self.assertEqual(sent.header.stamp.sec,999);self.assertEqual(sent.header.stamp.nanosec,980000000)
        self.assertTrue(node._perception_metrics['obstacle_published'])

    def test_large_cloud_callback_keeps_receiving_localization_without_processing(self):
        localization_tests.LocalizationInputTests.setUpClass();helper=localization_tests.LocalizationInputTests()
        node=helper.node('lidar_track_node','LidarTrackNode');node._async_enabled=True;submitted=[]
        node._async_perception=types.SimpleNamespace(submit=submitted.append)
        msg=cloud(999.98);msg.data=msg.data*400;msg.width*=400;msg.row_step=msg.width*msg.point_step
        durations=[]
        with patch.object(node._perception,'process',side_effect=AssertionError('heavy work ran in callback')):
            for i in range(10):
                node._cb_odom(helper.localization(stamp=997.,x=float(i)))
                msg.header.stamp.nanosec=980000000+i*1000000
                start=time.monotonic();node._cb_cloud(msg);durations.append(time.monotonic()-start)
        self.assertEqual(len(submitted),10);self.assertEqual(submitted[-1][3][0],9.)
        self.assertLess(max(durations),.05)

    def test_native_components_match_original_euclidean_extrema_graph(self):
        rng=np.random.default_rng(8)
        for trial in range(12):
            points=rng.uniform(-2.,2.,(180,2));keys=np.floor(points/.1).astype(np.int32)
            cells,ids,counts=cell_index(keys);starts=np.r_[0,np.cumsum(counts)[:-1]];ends=np.cumsum(counts)-1;indices=[]
            for axis in (0,1):
                order=np.lexsort((points[:,axis],ids));indices.extend((order[starts],order[ends]))
            representatives=np.stack([points[idx] for idx in indices],axis=1)
            labels,_=voxel_components(cells,representatives,.1,.3)
            selected=np.unique(np.concatenate(indices));old=Clusterer(.3,1,10.).cluster([tuple(p) for p in points[selected]])
            component={p:i for i,group in enumerate(old) for p in group}
            old_labels=np.array([component[tuple(p)] for p in points[selected]])
            new_labels=labels[ids[selected]]
            self.assertTrue(np.array_equal(old_labels[:,None]==old_labels[None,:],new_labels[:,None]==new_labels[None,:]))

if __name__=='__main__':unittest.main()
