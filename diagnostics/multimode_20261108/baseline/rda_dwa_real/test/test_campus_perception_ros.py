"""Source-time and invalid-result contracts for the campus surface grid.

Uses in-memory ROS doubles and one local spawned numerical worker; no DDS or
vehicle I/O. Grid evidence is published only with a valid, fresh observation.
"""
from dataclasses import asdict
from pathlib import Path
import sys
import time
import types
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa_real'),str(ROOT/'rda_dwa'),str(Path(__file__).parent)]
from test_campus_perception import road,returns,grid_at
import test_localization_input as localization_tests
from rda_dwa_real.async_perception import AsyncPerception,cloud_packet
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams
from rda_dwa_real.kf_tracker import GlobalKFTracker


def campus_cloud(stamp=999.98):
    xy=np.array([[2.+i*.03,1.+j*.03] for i in range(6) for j in range(6)])
    xyz=np.vstack((road(spacing=.1),returns(xy,.1))).astype('<f4')
    sec=int(stamp);ns=round((stamp-sec)*1e9)
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=ns),frame_id='rslidar'),
        height=1,width=len(xyz),point_step=12,row_step=len(xyz)*12,data=xyz.tobytes(),is_bigendian=False,
        fields=[types.SimpleNamespace(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])


class CampusPerceptionROSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        localization_tests.LocalizationInputTests.setUpClass()

    def node(self):
        helper=localization_tests.LocalizationInputTests();node=helper.node('lidar_track_node','LidarTrackNode')
        self.obstacles=[];node._publish_obstacles=self.obstacles.append
        node._cb_odom(helper.localization(stamp=997.,x=0.))
        return node

    def grid(self):
        _,metrics=LidarPerceptionCore().process(road(spacing=.1),100.)
        return metrics['drivable_grid']

    def test_sync_curb_grid_preserves_source_time_and_summary(self):
        node=self.node();cloud=campus_cloud();node._cb_cloud(cloud)
        self.assertTrue(self.obstacles)
        sent=node._pub_drivable.messages[-1]
        self.assertEqual(sent.header.frame_id,'base_link')
        self.assertEqual(sent.header.stamp.sec,cloud.header.stamp.sec)
        self.assertEqual(sent.header.stamp.nanosec,cloud.header.stamp.nanosec)
        self.assertEqual(sent.info.origin.orientation.w,1.)
        self.assertEqual(len(sent.data),sent.info.width*sent.info.height)
        self.assertTrue(node._perception_metrics['drivable_published'])
        self.assertNotIn('drivable_grid',node._perception_metrics)
        self.assertNotIn('data',node._perception_metrics['drivable_grid_summary'])

    def poll(self,node,age,tracks,valid=True):
        metrics=dict(source_age_at_receive_s=.02,reason='published',valid=valid,drivable_grid=self.grid())
        payload=(1,time.monotonic()-age,999,980000000,tracks,metrics)
        node._async_perception=types.SimpleNamespace(poll=lambda:payload,submitted=1,skipped=0)
        node._poll_perception()

    def test_fresh_async_grid_keeps_acquisition_stamp(self):
        node=self.node();self.poll(node,.01,[])
        sent=node._pub_drivable.messages[-1]
        self.assertEqual((sent.header.stamp.sec,sent.header.stamp.nanosec),(999,980000000))

    def test_expired_async_grid_is_not_published(self):
        node=self.node();self.poll(node,.7,[])
        self.assertFalse(node._pub_drivable.messages)
        self.assertFalse(self.obstacles)
        self.assertEqual(node._perception_metrics['reason'],'perception_result_stale')

    def test_invalid_partial_result_does_not_publish_grid(self):
        node=self.node();self.poll(node,.01,None,valid=False)
        self.assertFalse(node._pub_drivable.messages);self.assertFalse(self.obstacles)

    def test_all_lidar_yaml_parameters_are_declared_and_read(self):
        node=self.node()
        self.assertFalse(set(node.overrides)-set(node.declared))
        self.assertFalse(set(node.overrides)-node.read)
        expected=asdict(PerceptionParams())
        for name in ('ground_reference_z','ground_reference_tolerance','height_min','dropoff_height',
                     'boundary_min_points','max_distance_tests','cavity_tile_size','drivable_resolution'):
            self.assertEqual(getattr(node._perception.p,name),expected[name])

    def test_new_geometry_and_memory_limits_reject_invalid_configuration(self):
        for changed in ({'ground_reference_z':float('nan')},{'drivable_resolution':.00001},
                        {'max_distance_tests':0},{'sensor_yaw':float('inf')},{'x_min':10.,'x_max':1.}):
            with self.assertRaises(ValueError):LidarPerceptionCore(PerceptionParams(**changed))

    def test_spawned_worker_serializes_grid_and_retains_curb(self):
        worker=AsyncPerception(PerceptionParams(),GlobalKFTracker())
        try:
            deadline=time.monotonic()+5.;sequence=0;result=None
            while time.monotonic()<deadline:
                sequence+=1;cloud=campus_cloud(300.+sequence*.05)
                worker.submit((sequence,time.monotonic(),cloud_packet(cloud),(0.,0.,0.),dict(source_age_at_receive_s=.01)))
                result=worker.poll()
                if result is not None and result[-2] is not None:break
                time.sleep(.02)
            self.assertIsNotNone(result);self.assertTrue(result[-2])
            metrics=result[-1];self.assertTrue(metrics['valid'])
            self.assertEqual(grid_at(metrics['drivable_grid'],(2.04,1.04)),100)
            self.assertEqual(grid_at(metrics['drivable_grid'],(0.,0.)),-1)
        finally:worker.close()


if __name__=='__main__':unittest.main()
