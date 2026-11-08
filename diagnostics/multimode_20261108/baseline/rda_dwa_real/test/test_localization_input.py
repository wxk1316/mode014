"""Latest localization, TF compatibility, and disconnect-stop regressions.

Uses ROS doubles only; no DDS, UDP, or vehicle commands are sent.
"""
import importlib
from pathlib import Path
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'rda_dwa_real'), str(ROOT/'rda_dwa'),
               str(ROOT/'docs/validation')]
import check_ros_adapters as doubles


class FakeTransformException(Exception):
    pass


class FakeBuffer:
    def __init__(self, **kwargs):
        self.transform = None
        self.calls = []

    def lookup_transform(self, *args):
        self.calls.append(args)
        if self.transform is None:
            raise FakeTransformException('missing')
        return self.transform


def transform(stamp=999.98, x=0., y=0., yaw=0.):
    import math
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(
            sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y, z=0.),
            rotation=types.SimpleNamespace(x=0., y=0.,
                z=math.sin(yaw/2), w=math.cos(yaw/2))))


class LocalizationInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        doubles.install_doubles()
        class Twist(doubles.Message):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.linear = self.twist.linear
                self.angular = self.twist.angular
        sys.modules['geometry_msgs.msg'].Twist = Twist
        doubles.module('rclpy.time').Time = object
        tf = doubles.module('tf2_ros')
        tf.Buffer = FakeBuffer
        tf.TransformException = FakeTransformException
        tf.TransformListener = lambda buffer, node: object()
        from rda_dwa_real.localization_input import LocalizationInput
        cls.input_class = LocalizationInput

    def input(self, source='tf'):
        node = doubles.Node('test_localization')
        node.overrides = {'localization_source': source}
        self.clock = 1000.
        node.get_clock = lambda: types.SimpleNamespace(now=lambda:
            types.SimpleNamespace(nanoseconds=round(self.clock*1e9)))
        return self.input_class(node)

    def node(self, module, classname, source='latest_topic'):
        names = {'bridge_node': 'rda_dwa_real_bridge',
                 'traj_tracker_node': 'traj_tracker_node',
                 'lidar_track_node': 'lidar_track_node'}
        config = doubles.CONFIG[names[module]]['ros__parameters']
        bridge_config = doubles.CONFIG['rda_dwa_real_bridge']['ros__parameters']
        old_source, old_async = config['localization_source'], bridge_config['async_planning']
        old_perception_async=config.get('async_perception')
        if old_perception_async is not None:config['async_perception']=False
        config['localization_source'] = source
        bridge_config['async_planning'] = False
        try:
            return getattr(importlib.import_module(f'rda_dwa_real.{module}'), classname)()
        finally:
            config['localization_source'] = old_source
            bridge_config['async_planning'] = old_async
            if old_perception_async is not None:config['async_perception']=old_perception_async

    def localization(self, stamp=997., x=0., y=0., yaw=0., frame='map'):
        tr = transform(stamp, x, y, yaw)
        tr.header.frame_id = frame
        pose = types.SimpleNamespace(position=tr.transform.translation,
                                     orientation=tr.transform.rotation)
        return types.SimpleNamespace(header=tr.header,
                                     pose=types.SimpleNamespace(pose=pose))

    def test_latest_tf_uses_source_time_and_body_frame(self):
        source = self.input()
        source.buffer.transform = transform(x=2., yaw=.2)
        pose, stamp, age = source.sample(.35)
        self.assertEqual(pose[:2], (2., 0.))
        self.assertAlmostEqual(pose[2], .2)
        self.assertAlmostEqual(stamp, 999.98)
        self.assertAlmostEqual(age, .02)
        self.assertEqual(source.buffer.calls[0][:2], ('map', 'body'))
        self.assertEqual(len(source.buffer.calls[0]), 3)  # no blocking timeout

    def test_missing_stale_future_and_invalid_tf_rejected(self):
        source = self.input()
        self.assertIsNone(source.sample(.35))
        self.assertEqual(source.reason, 'tf_unavailable')
        for sample, reason in [(transform(999.), 'tf_stale'),
                               (transform(1000.1), 'tf_future'),
                               (transform(x=float('nan')), 'tf_invalid')]:
            source.buffer.transform = sample
            self.assertIsNone(source.sample(.35))
            self.assertEqual(source.reason, reason)
        source.buffer.transform = transform()
        source.buffer.transform.transform.rotation.w = 0.
        self.assertIsNone(source.sample(.35))

    def test_topic_mode_has_no_tf_dependency_or_lookup(self):
        source = self.input('topic')
        self.assertFalse(source.uses_tf)
        self.assertFalse(hasattr(source, 'buffer'))
        self.assertIsNone(source.sample(.35))

    def test_repeated_tf_does_not_refresh_tracker_source(self):
        from rda_dwa_real.traj_tracker_core import TrajTrackerCore
        source = self.input()
        source.buffer.transform = transform()
        core = TrajTrackerCore()
        for now in [1000., 1000.1, 1000.2]:
            self.clock = now
            pose, stamp, age = source.sample(.35)
            core.set_pose(*pose, stamp=stamp, arrival=now-age)
        self.assertAlmostEqual(core._pose_at, 999.98)
        self.clock = 1000.4
        self.assertIsNone(source.sample(.35))
        self.assertGreater(self.clock-core._pose_at, core.p.localization_timeout)

    def test_all_consumers_ignore_topic_and_accept_fresh_tf(self):
        for module, classname, callback in [
                ('bridge_node', 'RealVehicleBridgeNode', '_cb_odom'),
                ('traj_tracker_node', 'TrajTrackerNode', '_cb_loc'),
                ('lidar_track_node', 'LidarTrackNode', '_cb_odom')]:
            with self.subTest(node=module):
                node = self.node(module, classname, source='tf')
                getattr(node, callback)(object())  # topic data cannot overwrite TF
                node._localization_input.buffer.transform = transform(x=.3)
                if module == 'bridge_node':
                    node._control_cycle()
                    self.assertAlmostEqual(node.core.centre()[0], .3)
                    self.assertAlmostEqual(node.core._pose_source, 999.98)
                elif module == 'traj_tracker_node':
                    node._control_tick()
                    self.assertAlmostEqual(node.core._centre[0], .3)
                    self.assertAlmostEqual(node.core._pose_stamp, 999.98)
                    # Keep plan and obstacles current; TF alone is made stale.
                    node._localization_input.buffer.transform = transform(997.)
                    node.core._pose_at = node._now()-3.
                    node._last_cmd_ts = node._now()
                    node.core._obstacle_at = node._now()
                    node._control_tick()
                    self.assertEqual(node.core._safety_reason, 'localization_timeout')
                    self.assertEqual(node._pub_car.messages[-1].speed, 0.)
                else:
                    node._process_cloud = lambda msg: 'test_cloud'
                    cloud = types.SimpleNamespace(header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(sec=1000, nanosec=0),
                        frame_id='rslidar'), width=0, height=1, data=b'')
                    node._cb_cloud(cloud)
                    self.assertAlmostEqual(node._loc[0], .3)
                    self.assertAlmostEqual(node._loc_t, 999.98)

    def test_forward_goal_produces_motion_and_tf_loss_stops_it(self):
        import json
        import time
        bridge = self.node('bridge_node', 'RealVehicleBridgeNode', source='tf')
        tracker = self.node('traj_tracker_node', 'TrajTrackerNode', source='tf')
        bridge._localization_input.buffer.transform = transform()
        bridge.core.set_goal(4., 0., frame='map')
        bridge.core.set_obstacles([], stamp=1000., arrival=time.monotonic())
        bridge._control_cycle()
        packet = bridge._pub_plan.messages[-1]
        self.assertEqual(json.loads(packet.data)['reason'], 'motion')
        self.assertGreater(len(json.loads(packet.data)['reference']), 2)
        tracker._cb_plan(packet)
        origin = tracker._now()
        maximum_speed = 0.
        for tick in range(20):
            now = origin+tick*.02
            ros_now = 1000.+tick*.02
            tracker._now = lambda now=now: now
            tracker.get_clock = lambda ros_now=ros_now: types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(nanoseconds=round(ros_now*1e9)))
            tracker._localization_input.buffer.transform = transform(ros_now-.02)
            tracker.core.set_obstacles([], now, stamp=ros_now)
            tracker._control_tick()
            maximum_speed = max(maximum_speed, tracker._pub_car.messages[-1].speed)
        self.assertGreater(maximum_speed, 0.)
        tracker._localization_input.buffer.transform = None
        for tick in range(20, 140):
            now = origin+tick*.02
            tracker._now = lambda now=now: now
            tracker._last_cmd_ts = now
            tracker.core.set_obstacles([], now, stamp=1000.+tick*.02)
            tracker._control_tick()
        self.assertEqual(tracker.core._safety_reason, 'localization_timeout')
        self.assertEqual(tracker._pub_car.messages[-1].speed, 0.)

    def test_latest_topic_accepts_old_duplicate_and_irregular_headers(self):
        source = self.input('latest_topic')
        for receipt, stamp, x in [(10., 997., 0.), (10.1, 997., .02),
                                  (11.5, 996., .1), (11.6, 1005., .11)]:
            msg = self.localization(stamp=stamp, x=x)
            self.assertTrue(source.receive_topic(msg.pose.pose, msg.header, received_at=receipt))
            pose, effective, age = source.sample(2., now=receipt+.1)
            self.assertAlmostEqual(pose[0], x)
            self.assertEqual(effective, receipt)
            self.assertAlmostEqual(age, .1)
            self.assertAlmostEqual(source.stamp, stamp)
        self.assertFalse(hasattr(source, 'buffer'))
        self.assertEqual(source.diagnostic()['pose_stamp_clock'], 'monotonic_receive')

    def test_cache_reads_do_not_extend_disconnect_watchdog(self):
        source = self.input('latest_topic')
        msg = self.localization()
        source.receive_topic(msg.pose.pose, msg.header, received_at=10.)
        for now in (10.2, 10.9, 11.5, 12.):
            _, stamp, _ = source.sample(2., now=now)
            self.assertEqual(stamp, 10.)
        self.assertIsNone(source.sample(2., now=12.01))
        self.assertEqual(source.reason, 'latest_topic_disconnected')

    def test_invalid_pose_or_frame_cannot_refresh_latest_cache(self):
        source = self.input('latest_topic')
        msg = self.localization(x=.2)
        source.receive_topic(msg.pose.pose, msg.header, received_at=10.)
        for msg in (self.localization(x=float('nan')),
                    self.localization(frame='odom')):
            self.assertFalse(source.receive_topic(msg.pose.pose, msg.header, received_at=11.))
        msg = self.localization()
        msg.pose.pose.orientation.w = 0.
        self.assertFalse(source.receive_topic(msg.pose.pose, msg.header, received_at=11.))
        pose, _, _ = source.sample(2., now=11.9)
        self.assertAlmostEqual(pose[0], .2)
        self.assertIsNone(source.sample(2., now=12.1))

    def test_manual_goal_waits_for_localization_and_no_goal_is_not_invented(self):
        bridge = self.node('bridge_node', 'RealVehicleBridgeNode')
        bridge._control_cycle()
        self.assertEqual(bridge.core._last_info['reason'], 'no_goal')
        bridge.core.set_goal(4., 0., frame='map')
        bridge._control_cycle()
        self.assertEqual(bridge.core._last_info['reason'], 'localization_fault')
        self.assertEqual(bridge.core._last_info['v_cmd'], 0.)

    def test_three_consumers_use_latest_message_with_repeated_source_stamp(self):
        import time
        for module, classname, callback in [
                ('bridge_node', 'RealVehicleBridgeNode', '_cb_odom'),
                ('traj_tracker_node', 'TrajTrackerNode', '_cb_loc'),
                ('lidar_track_node', 'LidarTrackNode', '_cb_odom')]:
            with self.subTest(node=module):
                node = self.node(module, classname)
                now = time.monotonic()
                for receipt, x in [(now-.1, .1), (now-.05, .12)]:
                    from unittest.mock import patch
                    with patch('rda_dwa_real.localization_input.time.monotonic', return_value=receipt):
                        getattr(node, callback)(self.localization(stamp=997., x=x))
                if module == 'bridge_node':
                    node._control_cycle()
                    self.assertAlmostEqual(node.core.centre()[0], .12)
                    self.assertAlmostEqual(node.core._t_loc, now-.05)
                    self.assertEqual(node.core.cfg.localization_timeout, 2.)
                elif module == 'traj_tracker_node':
                    node._control_tick()
                    self.assertAlmostEqual(node.core._centre[0], .12)
                    self.assertAlmostEqual(node.core._pose_at, now-.05)
                    self.assertEqual(node.core.p.localization_timeout, 2.)
                else:
                    node._process_cloud = lambda msg: 'test_cloud'
                    node._cb_cloud(types.SimpleNamespace(header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(sec=1000, nanosec=0),
                        frame_id='rslidar'), width=0, height=1, data=b''))
                    self.assertAlmostEqual(node._loc[0], .12)
                    self.assertLess(node._perception_metrics['location_age_s'], .2)
                    self.assertEqual(node._perception_metrics['location_source_stamp'], 997.)
                    self.assertEqual(node._loc_timeout, 2.)

    def test_latest_localization_enables_motion_and_disconnect_stops_tracker(self):
        import json
        import time
        bridge = self.node('bridge_node', 'RealVehicleBridgeNode')
        tracker = self.node('traj_tracker_node', 'TrajTrackerNode')
        bridge._cb_odom(self.localization(stamp=997.))
        bridge.core.set_goal(4., 0., frame='map')
        bridge.core.set_obstacles([], stamp=1000., arrival=time.monotonic())
        bridge._control_cycle()
        packet = bridge._pub_plan.messages[-1]
        self.assertEqual(json.loads(packet.data)['reason'], 'motion')
        tracker._cb_plan(packet)
        origin = tracker._now()
        maximum_speed = 0.
        for tick in range(20):
            now = origin+tick*.02
            tracker._now = lambda now=now: now
            # Header remains old and identical throughout the run.
            tracker._cb_loc(self.localization(stamp=997.))
            tracker.core.set_obstacles([], now, stamp=1000.+tick*.02)
            tracker._control_tick()
            maximum_speed = max(maximum_speed, tracker._pub_car.messages[-1].speed)
        self.assertGreater(maximum_speed, 0.)
        for tick in range(20, 140):
            now = origin+tick*.02
            tracker._now = lambda now=now: now
            tracker._last_cmd_ts = now
            tracker.core.set_obstacles([], now, stamp=1000.+tick*.02)
            tracker._control_tick()
        self.assertEqual(tracker.core._safety_reason, 'localization_timeout')
        self.assertEqual(tracker._pub_car.messages[-1].speed, 0.)

    def test_lidar_processing_uses_receipt_clock_and_allows_irregular_updates(self):
        import struct
        import time
        from unittest.mock import patch
        node = self.node('lidar_track_node', 'LidarTrackNode')
        node._perception.p.ground_enabled = False  # isolate the receipt-clock gate
        origin = time.monotonic()
        msg = self.localization(stamp=997.)
        node._localization_input.receive_topic(msg.pose.pose, msg.header, received_at=origin-1.)
        outputs = []
        node._publish_obstacles = outputs.append
        cloud = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=999, nanosec=990000000), frame_id='rslidar'),
            width=1, height=1, fields=[types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
            point_step=12, data=struct.pack('<fff', 2., 0., 0.))
        with patch('rda_dwa_real.localization_input.time.monotonic', return_value=origin):
            node._cb_cloud(cloud)
        self.assertEqual(node._perception_metrics['reason'], 'empty_cloud_predicted_tracks')
        self.assertEqual(len(outputs), 1)
        cloud.header.stamp.nanosec += 1000000
        with patch('rda_dwa_real.localization_input.time.monotonic', return_value=origin+1.01):
            node._cb_cloud(cloud)
        self.assertEqual(node._perception_metrics['reason'], 'localization_stale')
        self.assertEqual(len(outputs), 1)

    def test_node_passes_configured_result_age_and_budget_to_worker(self):
        from unittest.mock import patch
        module = importlib.import_module('rda_dwa_real.bridge_node')
        config = doubles.CONFIG['rda_dwa_real_bridge']['ros__parameters']
        old_async = config['async_planning']
        config['async_planning'] = True
        captured = {}
        def factory(base, max_age):
            captured.update(budget=base.params.planning_budget_s, max_age=max_age)
            return base
        try:
            with patch.object(module, 'AsyncPlanner', side_effect=factory):
                module.RealVehicleBridgeNode()
        finally:
            config['async_planning'] = old_async
        self.assertEqual(captured, {'budget': .25, 'max_age': .8})


if __name__ == '__main__':
    unittest.main()
