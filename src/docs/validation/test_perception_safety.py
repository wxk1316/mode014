"""Offline perception contract regressions. No ROS imports or vehicle output."""
from pathlib import Path
import math
import random
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'rda_dwa'), str(ROOT/'rda_dwa_real')]

from rda_dwa_real.frame_utils import (SourcePoseBuffer, filter_lidar_points,
                                    oriented_box_aabb, body_to_world_2d,
                                    body_pose_to_base_link)
from rda_dwa_real.obstacle_fusion import (Clusterer, cluster_bounding_box, TrackManager,
    split_world_cluster,WorldObstacleRepresenter,RepresentationLimitError)
from rda_dwa_real.kf_tracker import GlobalKFTracker
from rda_dwa.predictor import ObstaclePredictor
from rda_dwa_real.diag_recorder_node import (
    nearest_obstacle_summary, command_mode_matches_feedback, build_snapshot_row)


class PerceptionSafetyTests(unittest.TestCase):
    def test_full_surround_and_low_object_kept_ground_removed(self):
        points = [(1,0,-.2),(-1,0,-.2),(0,1,-.2),(0,-1,-.2),
                  (1,0,-.45),(.1,.1,-.2),(7,0,-.2)]
        got = filter_lidar_points(points)
        self.assertCountEqual(got, [(1,0),(-1,0),(0,1),(0,-1)])

    def test_mode_wait_observation_ring_keeps_eleven_meters(self):
        points = [(11,0,-.2),(-11,0,-.2),(0,11,-.2),(0,-11,-.2),
                  (13,0,-.2),(-13,0,-.2),(0,13,-.2),(0,-13,-.2)]
        got = filter_lidar_points(points,max_range=12,x_min=-12,x_max=12,
                                  y_min=-12,y_max=12)
        self.assertCountEqual(got,[(11,0),(-11,0),(0,11),(0,-11)])

    def test_filter_toggle_and_extrinsics_are_used(self):
        got = filter_lidar_points([(1,0,-2)],use_z_filter=False,
            sensor_x=.2,sensor_yaw=math.pi/2)
        self.assertEqual(len(got),1)
        self.assertAlmostEqual(got[0][0],.2)
        self.assertAlmostEqual(got[0][1],1)

    def test_self_exclusion_uses_physical_body_center(self):
        pts = [(.8,0,-.2),(-.7,0,-.2)]
        got = filter_lidar_points(pts,half_length=.75,vehicle_center_x=.15)
        self.assertEqual(got,[(-.7,0)])

    def test_confirmed_vehicle_center_zero_and_length_one_point_two(self):
        got = filter_lidar_points([(.59,0,-.2),(-.59,0,-.2),(.61,0,-.2),(-.61,0,-.2)])
        self.assertCountEqual(got,[(.61,0),(-.61,0)])

    def test_side_obstacle_two_cm_outside_body_is_not_self_filtered(self):
        got = filter_lidar_points([(0,.42,-.2),(.59,.39,-.2),(.59,.42,-.2)])
        self.assertCountEqual(got,[(0,.42),(.59,.42)])

    def test_center_localization_has_no_second_planar_offset(self):
        for yaw in (0,math.pi/2,math.pi):
            self.assertEqual(body_pose_to_base_link(12,3,yaw,0.0),(12,3,yaw))

    def test_structured_pointcloud_records_are_accepted(self):
        cloud = np.array([(1.,0.,-.2)],dtype=[('x','f4'),('y','f4'),('z','f4')])
        self.assertEqual(filter_lidar_points(cloud),[(1.,0.)])

    def test_old_heartbeat_does_not_refresh_pose(self):
        poses = SourcePoseBuffer(max_age=.5)
        self.assertTrue(poses.add(100,(0,0,0),100))
        self.assertFalse(poses.add(100,(0,0,0),100.4))
        self.assertFalse(poses.fresh(100.6))
        self.assertFalse(poses.add(100,(1,1,1),100.6))

    def test_pose_interpolation_wraps_yaw_at_pi(self):
        poses = SourcePoseBuffer(sync_tolerance=.2,max_speed=20)
        poses.add(100,(0,0,math.radians(179)),100)
        poses.add(100.2,(2,0,math.radians(-179)),100.2)
        pose = poses.at(100.1)
        self.assertAlmostEqual(pose[0],1)
        self.assertAlmostEqual(abs(pose[2]),math.pi)
        self.assertIsNone(poses.at(99.5))

    def test_localization_reset_jump_is_latched_invalid(self):
        poses = SourcePoseBuffer()
        self.assertTrue(poses.add(100,(0,0,0),100))
        self.assertFalse(poses.add(100.1,(3,0,0),100.1))
        self.assertFalse(poses.fresh(100.1))
        self.assertFalse(poses.add(110,(3,0,0),110))
        self.assertEqual(poses.fault,'localization_jump')

    def test_normal_spin_at_ten_hz_keeps_pose_valid(self):
        poses = SourcePoseBuffer(max_speed=.4,max_yaw_rate=.8)
        for i in range(30):
            theta = i*.08
            # User confirmed compensated base_link is the physical center.
            pose = (0,0,theta)
            self.assertTrue(poses.add(100+i*.1,pose,100+i*.1))
        self.assertFalse(poses.fault)

    def test_reverse_ego_motion_does_not_make_static_obstacle_dynamic(self):
        poses = SourcePoseBuffer(max_speed=.4,max_yaw_rate=.8)
        tracker = GlobalKFTracker(dt=.1,static_hold_frames=5)
        for i in range(20):
            stamp,rx = 100+i*.1,-.3*i*.1
            self.assertTrue(poses.add(stamp,(rx,0,0),stamp))
            # Fixed map object appears to move forward in the reversing sensor.
            body_x = 2-rx
            wx,wy,_ = body_to_world_2d(body_x,0,0,rx,0,0)
            out = tracker.step([(wx,wy,'block',.3,.3)],stamp=stamp)
            self.assertAlmostEqual(out[0][1],2)
            self.assertAlmostEqual(out[0][3],0)
        self.assertEqual(out[0][5],'block')

    def test_negative_world_velocity_is_positive_speed_with_pi_heading(self):
        fusion = TrackManager(ObstaclePredictor(dt=.1),lost_ttl_seconds=.5,
                              uncertainty_growth=0)
        fusion.update_measurements([(2,0,'pedestrian',.3,.3,math.pi,1,9)],
                                   now=0,stamp=100)
        out = fusion.snapshot(.2)[0]
        self.assertAlmostEqual(out.x,1.8)
        self.assertAlmostEqual(out.speed,1)
        self.assertAlmostEqual(abs(out.velocity_heading),math.pi)

    def test_diagonal_static_box_map_aabb_contains_all_corners(self):
        length,width,yaw = 3,.2,math.radians(35)
        L,W = oriented_box_aabb(length,width,yaw)
        c,s = math.cos(yaw),math.sin(yaw)
        for x in (-length/2,length/2):
            for y in (-width/2,width/2):
                self.assertLessEqual(abs(c*x-s*y),L/2+1e-9)
                self.assertLessEqual(abs(s*x+c*y),W/2+1e-9)

    def test_connected_clustering_order_independent(self):
        points = [(i*.2,1) for i in range(12)] + [(i*.1,-2) for i in range(5)]
        clusterer = Clusterer(gate=.3,min_points=2)
        original = clusterer.cluster(points)
        random.Random(23).shuffle(points)
        shuffled = clusterer.cluster(points)
        self.assertEqual(len(original),2)
        a = sorted(cluster_bounding_box(c)[:4] for c in original)
        b = sorted(cluster_bounding_box(c)[:4] for c in shuffled)
        for av,bv in zip(a,b):
            for x,y in zip(av,bv):
                self.assertAlmostEqual(x,y)

    def test_voxel_compression_preserves_raw_point_coverage(self):
        points = [(1+.002*i,.8+.002*(i%23)) for i in range(100)]
        cluster = Clusterer(gate=.6,min_points=2).cluster(points)[0]
        x,y,L,W,yaw = cluster_bounding_box(cluster)
        c,s = math.cos(yaw),math.sin(yaw)
        self.assertLess(len(cluster),len(points))
        for px,py in points:
            self.assertLessEqual(abs((px-x)*c+(py-y)*s),L/2+1e-9)
            self.assertLessEqual(abs(-(px-x)*s+(py-y)*c),W/2+1e-9)

    def test_bbox_center_contains_nonuniform_samples(self):
        points = [(0,1)]*40+[(3,1),(0,2),(3,2)]
        x,y,L,W,yaw = cluster_bounding_box(points)
        c,s = math.cos(yaw),math.sin(yaw)
        for px,py in points:
            self.assertLessEqual(abs((px-x)*c+(py-y)*s),L/2+1e-9)
            self.assertLessEqual(abs(-(px-x)*s+(py-y)*c),W/2+1e-9)

    def test_kf_real_dt_estimates_speed_before_motion_confirmation(self):
        trackers = [GlobalKFTracker(dt=.05),GlobalKFTracker(dt=.05)]
        for tracker,elapsed in zip(trackers,[.05,.1]):
            for i in range(1,41):
                out = tracker.step([(i*elapsed,1,'pedestrian',.3,.3)],
                                   stamp=100+i*elapsed)
            self.assertEqual(out[0][5],'pedestrian')
            self.assertAlmostEqual(out[0][3],1,delta=.1)
        short = GlobalKFTracker(dt=.1,dyn_hold_frames=20)
        short.step([(0,0,'block',.3,.3)],stamp=100)
        out = short.step([(.1,0,'block',.3,.3)],stamp=100.1)
        self.assertEqual(out[0][5],'pedestrian')
        self.assertGreater(out[0][3],0)

    def test_association_uses_predicted_position(self):
        tracker = GlobalKFTracker(dt=.1,gate=.05)
        tracker.step([(0,0,'pedestrian',.3,.3)],stamp=100)
        tracker.tracks[1].x[2] = 1
        out = tracker.step([(.2,0,'pedestrian',.3,.3)],stamp=100.2)
        self.assertEqual([o[0] for o in out],[1])

    def test_kf_duplicate_source_does_not_erase_velocity(self):
        tracker = GlobalKFTracker(dt=.1)
        tracker.step([(0,0,'pedestrian',.3,.3)],stamp=100)
        tracker.step([(.1,0,'pedestrian',.3,.3)],stamp=100.1)
        before = tracker.tracks[1].x.copy()
        tracker.step([(.1,0,'pedestrian',.3,.3)],stamp=100.1)
        self.assertTrue((tracker.tracks[1].x==before).all())

    def test_kf_empty_frame_holds_until_time_ttl(self):
        tracker = GlobalKFTracker(dt=.1,lost_ttl_seconds=.5)
        tracker.step([(0,0,'pedestrian',.3,.3)],stamp=100)
        self.assertEqual(len(tracker.step([],stamp=100.2)),1)
        self.assertEqual(len(tracker.step([],stamp=100.6)),0)

    def test_fusion_extrapolation_does_not_repeat_measurement(self):
        fusion = TrackManager(ObstaclePredictor(dt=.1),lost_ttl_seconds=.5,
                              uncertainty_growth=0)
        fusion.update_measurements([(1,2,'pedestrian',.3,.3,math.pi/2,1,9)],
                                   now=0,stamp=100)
        at_two = fusion.snapshot(.2)[0]
        at_three = fusion.snapshot(.3)[0]
        self.assertAlmostEqual(at_two.y,2.2)
        self.assertAlmostEqual(at_three.y,2.3)
        self.assertAlmostEqual(fusion.tracks[9].y,2)
        self.assertAlmostEqual(at_three.speed,1)
        self.assertEqual(fusion.snapshot(.6),[])

    def test_fusion_occlusion_and_shape_can_grow(self):
        fusion = TrackManager(ObstaclePredictor(dt=.1),lost_ttl_seconds=.5)
        fusion.update_measurements([(1,1,'block',.3,.2,0,0,5)],now=0,stamp=100)
        self.assertEqual(len(fusion.update_measurements([],now=.1,stamp=100.1)),1)
        out = fusion.update_measurements([(1,1,'pedestrian',1,.4,.7,1,5)],
                                         now=.2,stamp=100.2)[0]
        self.assertEqual(out.obstacle_type,'pedestrian')
        self.assertGreaterEqual(out.length,math.hypot(1,.4))
        self.assertEqual(out.shape_heading,0)
        self.assertAlmostEqual(out.velocity_heading,.7)

    def test_first_stationary_pedestrian_late_frame_covers_unknown_motion(self):
        fusion = TrackManager(ObstaclePredictor(dt=.1),max_speed=1.5,
                              lost_ttl_seconds=.6,uncertainty_growth=0)
        fusion.update_measurements([(2,0,'pedestrian',.3,.3,0,0,9)],
                                   now=10,stamp=100)
        initial = fusion.tracks[9].radius
        out = fusion.snapshot(10.5)[0]
        self.assertTrue(out.velocity_unknown)
        self.assertEqual(out.observed_count,1)
        self.assertGreaterEqual(out.radius,initial+1.5*.5)

    def test_second_real_source_measurement_releases_velocity_unknown(self):
        tracker = GlobalKFTracker(dt=.1,max_speed=1.5)
        fusion = TrackManager(ObstaclePredictor(dt=.1),max_speed=1.5,
                              lost_ttl_seconds=.6,uncertainty_growth=0)
        def detection(track):
            tid,x,y,speed,heading,kind,L,W = track
            return x,y,kind,L,W,heading,speed,tid
        first = tracker.step([(2,0,'pedestrian',.3,.3)],stamp=100)[0]
        out = fusion.update_measurements([detection(first)],now=10,stamp=100)[0]
        self.assertTrue(out.velocity_unknown)
        self.assertEqual(first[5],'unknown')
        second = tracker.step([(1.9,0,'pedestrian',.3,.3)],stamp=100.1)[0]
        out = fusion.update_measurements([detection(second)],now=10.1,stamp=100.1)[0]
        self.assertFalse(out.velocity_unknown)
        self.assertEqual(out.observed_count,2)
        self.assertAlmostEqual(out.speed,1,delta=.1)
        self.assertLess(fusion.snapshot(10.2)[0].x,out.x)

    def test_first_observation_miss_does_not_reset_reachable_bound(self):
        tracker = GlobalKFTracker(dt=.1,max_speed=1.5,lost_ttl_seconds=.6)
        fusion = TrackManager(ObstaclePredictor(dt=.1),max_speed=1.5,
                              lost_ttl_seconds=.6,uncertainty_growth=0)
        first = tracker.step([(2,0,'pedestrian',.3,.3)],stamp=100)[0]
        def detection(track):
            tid,x,y,speed,heading,kind,L,W = track
            return x,y,kind,L,W,heading,speed,tid
        first_obs = fusion.update_measurements([detection(first)],now=10,stamp=100)[0]
        predicted = tracker.step([],stamp=100.2)[0]
        out = fusion.update_measurements([detection(predicted)],now=10.2,stamp=100.2)[0]
        self.assertTrue(out.velocity_unknown)
        self.assertEqual(out.observed_count,1)
        self.assertGreaterEqual(out.radius,first_obs.radius+1.5*.2)
        measured = tracker.step([(1.7,0,'pedestrian',.3,.3)],stamp=100.3)[0]
        known = fusion.update_measurements([detection(measured)],now=10.3,stamp=100.3)[0]
        self.assertFalse(known.velocity_unknown)
        self.assertLess(known.radius,out.radius)  # uncertainty is not permanent shape

    def test_confirmed_static_first_frame_has_no_velocity_unknown(self):
        fusion = TrackManager(ObstaclePredictor(dt=.1),max_speed=1.5,
                              lost_ttl_seconds=.6,uncertainty_growth=0)
        out = fusion.update_measurements([(2,0,'block',.3,.3,0,0,9)],
                                         now=10,stamp=100)[0]
        self.assertFalse(out.velocity_unknown)
        self.assertAlmostEqual(fusion.snapshot(10.5)[0].radius,out.radius)

    @staticmethod
    def raw_u():
        return ([(-1.8,-1.3+i*.05) for i in range(53)]
                + [(x,y) for y in (-1.3,1.3)
                   for x in [-1.8+i*.05 for i in range(97)]])

    def test_raw_u_tiles_keep_cavity_and_cover_every_wall_point(self):
        raw = self.raw_u()
        clusters = Clusterer().cluster(raw)
        self.assertEqual(len(clusters),1)
        parts = split_world_cluster(clusters[0])
        self.assertGreater(len(parts),1)
        for _,x,y,L,W in parts:
            self.assertFalse(abs(x)<=L/2 and abs(y)<=W/2)
            self.assertFalse(abs(x)<=.6+L/2 and abs(y)<=.4+W/2)
        for px,py in raw:
            self.assertTrue(any(abs(px-x)<=L/2+1e-8 and abs(py-y)<=W/2+1e-8
                                for _,x,y,L,W in parts))

    def test_u_tile_geometry_is_point_order_independent(self):
        raw = self.raw_u()
        a = split_world_cluster(Clusterer().cluster(raw)[0])
        random.Random(56).shuffle(raw)
        b = split_world_cluster(Clusterer().cluster(raw)[0])
        self.assertEqual(a,b)

    def test_world_tiles_are_stable_during_ego_rotation(self):
        raw = self.raw_u()
        representer = WorldObstacleRepresenter()
        initial_positions = None
        for i in range(15):
            yaw = i*.08
            c,s = math.cos(yaw),math.sin(yaw)
            body = [(c*x+s*y,-s*x+c*y) for x,y in raw]
            kept = filter_lidar_points([(x,y,-.2) for x,y in body],max_range=12,
                                      x_min=-12,x_max=12,y_min=-12,y_max=12)
            world = [(c*x-s*y,s*x+c*y) for x,y in kept]
            out = representer.step(Clusterer().cluster(world,origin=(0,0)),100+i*.1)
            positions = {item[0]:(item[1],item[2]) for item in out}
            if initial_positions is None:
                initial_positions = positions
            self.assertEqual(positions,initial_positions)
            self.assertTrue(all(abs(item[3])<1e-9 for item in out))
        self.assertEqual({item[5] for item in out},{'block'})

    def test_moving_long_cluster_tiles_inherit_parent_motion(self):
        representer = WorldObstacleRepresenter()
        raw = [(i*.05,2) for i in range(100)]
        first = representer.step(Clusterer().cluster(raw),100)
        self.assertGreater(len(first),1)
        self.assertEqual({item[5] for item in first},{'unknown'})
        out = representer.step(Clusterer().cluster([(x+.1,y) for x,y in raw]),100.1)
        self.assertEqual({item[5] for item in out},{'pedestrian'})
        self.assertTrue(all(abs(item[3]-1)<.1 for item in out))

    def test_small_person_and_dense_scene_keep_whole_boxes(self):
        person = Clusterer().cluster([(1+i*.05,1+j*.05) for i in range(4) for j in range(4)])[0]
        self.assertEqual(split_world_cluster(person)[0][0],None)
        dense = Clusterer().cluster([(-3+i*.1,-3+j*.1) for i in range(61) for j in range(61)])[0]
        self.assertEqual(len(split_world_cluster(dense)),1)
        self.assertEqual(split_world_cluster(dense)[0][0],None)

    def test_thick_high_occupancy_u_does_not_fill_protected_cavity(self):
        raw = [(-2+i*.1,-2+j*.1) for i in range(41) for j in range(41)
               if abs(-2+j*.1)>=1.1 or -2+i*.1 <= -1.1]
        cluster = Clusterer().cluster(raw)[0]
        parts = split_world_cluster(cluster,protected_point=(0,0))
        self.assertGreater(len(parts),1)
        self.assertTrue(all(not (abs(x)<=L/2 and abs(y)<=W/2) for _,x,y,L,W in parts))

    def test_narrow_thick_u_trigger_uses_rectangle_clearance(self):
        raw = [(-2+i*.1,-1.4+j*.1) for i in range(41) for j in range(29)
               if abs(-1.4+j*.1)>=.8-1e-9 or -2+i*.1 <= -1.4+1e-9]
        cluster = Clusterer().cluster(raw)[0]
        parts = split_world_cluster(cluster,protected_pose=(0,0,0),half_length=.6,half_width=.4)
        self.assertGreater(len(parts),1)
        self.assertTrue(all(not (abs(x)<=.6+L/2 and abs(y)<=.4+W/2) for _,x,y,L,W in parts))

    def test_tile_limit_stops_instead_of_filling_cavity(self):
        representer = WorldObstacleRepresenter(max_tiles=4)
        with self.assertRaises(RepresentationLimitError):
            representer.step(Clusterer().cluster(self.raw_u()),100)

    def test_explicit_map_aabb_uses_physical_size_for_all_types(self):
        for kind in ('unknown','pedestrian','static_large','block'):
            fusion = TrackManager(ObstaclePredictor(dt=.1))
            out = fusion.update_measurements([(2,0,kind,.15,.5,1.2,0,9,0)],
                                             now=10,stamp=100)[0]
            self.assertAlmostEqual(out.length,.15)
            self.assertAlmostEqual(out.width,.5)
            self.assertEqual(out.shape_heading,0)

    def test_diagnostics_map_distance_and_mode_mapping(self):
        objs = [{'x':100,'y':0,'id':1,'type':'block'},
                {'x':2,'y':0,'id':2,'type':'block'}]
        self.assertEqual(nearest_obstacle_summary(objs,{'x':99,'y':0})[:2],(1,1))
        self.assertTrue(command_mode_matches_feedback(4,4))
        self.assertTrue(command_mode_matches_feedback(0,0))
        self.assertFalse(command_mode_matches_feedback(4,2))
        # The current ROS source age is supplied by the recorder; monotonic t
        # must not be added to it, especially across simulation/ROS clock jumps.
        row = build_snapshot_row({'loc':{'source_age':.5,'received_at':0},
                'diag':{'arbit':'ACK','spin_attempts':2},
                'ctrl_status':{'fault':'spin_no_yaw_progress','physical_ready':4}},
                .4,'offline')
        self.assertAlmostEqual(row['loc_source_age'],.5)
        self.assertEqual(row['diag_spin_attempts'],2)
        self.assertEqual(row['tk_fault'],'spin_no_yaw_progress')


if __name__ == '__main__':
    unittest.main()
