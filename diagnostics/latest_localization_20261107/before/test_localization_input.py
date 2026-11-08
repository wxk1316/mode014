"""Fresh TF input, topic isolation, and stop-on-source-loss regressions.

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
                config = doubles.CONFIG['rda_dwa_real_bridge']['ros__parameters']
                old_async = config['async_planning']
                config['async_planning'] = False
                try:
                    node = getattr(importlib.import_module(
                        f'rda_dwa_real.{module}'), classname)()
                finally:
                    config['async_planning'] = old_async
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
                    node._localization_input.buffer.transform = transform(999.)
                    node.core._pose_at = node._now()-1.
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
        config = doubles.CONFIG['rda_dwa_real_bridge']['ros__parameters']
        old_async = config['async_planning']
        config['async_planning'] = False
        try:
            bridge = importlib.import_module('rda_dwa_real.bridge_node').RealVehicleBridgeNode()
        finally:
            config['async_planning'] = old_async
        tracker = importlib.import_module('rda_dwa_real.traj_tracker_node').TrajTrackerNode()
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
        for tick in range(20, 40):
            now = origin+tick*.02
            tracker._now = lambda now=now: now
            tracker._last_cmd_ts = now
            tracker.core.set_obstacles([], now, stamp=1000.+tick*.02)
            tracker._control_tick()
        self.assertEqual(tracker.core._safety_reason, 'localization_timeout')
        self.assertEqual(tracker._pub_car.messages[-1].speed, 0.)


if __name__ == '__main__':
    unittest.main()
