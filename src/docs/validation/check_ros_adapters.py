"""Instantiate active ROS adapters with an in-memory ROS test double.

This checks constructor/configuration contracts only. It does not start ROS,
send DDS messages, open UDP sockets, or prove ROS middleware compatibility.
Run with Python + numpy + PyYAML; source packages are added from this workspace.
"""
from __future__ import annotations

import importlib
import json
import sys
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'rda_dwa'), str(ROOT / 'rda_dwa_real')]
CONFIG = yaml.safe_load((ROOT / 'rda_dwa_real/config/real_vehicle.yaml').read_text())


def module(name):
    result = types.ModuleType(name)
    sys.modules[name] = result
    return result


class Message:
    def __init__(self, **kwargs):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1000, nanosec=0), frame_id='map')
        self.data = None
        self.poses = []
        self.obstacles = []
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0., y=0., z=0.),
            orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))
        self.twist = types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0., y=0., z=0.),
            angular=types.SimpleNamespace(x=0., y=0., z=0.))
        self.info=types.SimpleNamespace(map_load_time=None,resolution=0.,width=0,height=0,
            origin=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
                orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        self.__dict__.update(kwargs)


class Clock:
    def now(self):
        return types.SimpleNamespace(
            nanoseconds=1000_000_000_000,
            to_msg=lambda: types.SimpleNamespace(sec=1000, nanosec=0))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class Node:
    def __init__(self, name):
        self.name = name
        self.overrides = CONFIG.get(name, {}).get('ros__parameters', {})
        self.declared = {}
        self.read = set()
        self.subscriptions = []
        self.publishers = {}

    def declare_parameter(self, name, default=None):
        value = self.overrides.get(name, default)
        if default is not None and type(value) is not type(default):
            raise TypeError(f'{self.name}.{name}: YAML {type(value).__name__}, '
                            f'declaration {type(default).__name__}')
        self.declared[name] = value
        return types.SimpleNamespace(value=value)

    def get_parameter(self, name):
        self.read.add(name)
        return types.SimpleNamespace(value=self.declared[name])

    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        self.subscriptions.append(topic)
        return types.SimpleNamespace(callback=callback)

    def create_publisher(self, msg_type, topic, qos, **kwargs):
        result = Publisher()
        self.publishers[topic] = result
        return result

    def create_timer(self, *args, **kwargs):
        return None

    def get_clock(self):
        return Clock()

    def get_logger(self):
        return types.SimpleNamespace(**{key: lambda *args, **kwargs: None
                                       for key in ('info', 'warning', 'warn', 'error', 'debug')})


def install_doubles():
    ros = module('rclpy')
    ros.init = ros.shutdown = ros.spin = lambda *args, **kwargs: None
    module('rclpy.node').Node = Node
    qos = module('rclpy.qos')
    qos.QoSProfile = lambda *args, **kwargs: types.SimpleNamespace(**kwargs)
    for name in ('DurabilityPolicy', 'ReliabilityPolicy', 'HistoryPolicy'):
        setattr(qos, name, types.SimpleNamespace(
            RELIABLE=1, BEST_EFFORT=2, VOLATILE=1, TRANSIENT_LOCAL=2, KEEP_LAST=1))
    qos.qos_profile_sensor_data = types.SimpleNamespace()
    callback = module('rclpy.callback_groups')
    callback.MutuallyExclusiveCallbackGroup = callback.ReentrantCallbackGroup = object
    module('rclpy.executors').MultiThreadedExecutor = object
    groups = {
        'std_msgs': ('Float32MultiArray', 'String', 'Float32', 'Int32', 'Bool', 'Header'),
        'sensor_msgs': ('LaserScan', 'NavSatFix', 'PointCloud2'),
        'geometry_msgs': ('Twist', 'TwistStamped', 'PoseStamped', 'PoseWithCovarianceStamped'),
        'nav_msgs': ('Odometry', 'Path', 'OccupancyGrid'),
        'control_msgs': ('CarState', 'CarCtr'),
        'rda_dwa_real_msgs': ('ObstacleArray', 'Obstacle'),
        'visualization_msgs': ('Marker', 'MarkerArray'),
    }
    for package, names in groups.items():
        parent = module(package)
        child = module(f'{package}.msg')
        parent.msg = child
        for name in names:
            cls = type(name, (Message,), {'ADD': 0, 'DELETEALL': 3, 'CUBE': 1, 'TEXT_VIEW_FACING': 9})
            setattr(child, name, cls)
    sensor = module('sensor_msgs_py')
    sensor.point_cloud2 = module('sensor_msgs_py.point_cloud2')
    sensor.point_cloud2.read_points = lambda *args, **kwargs: []


def main():
    install_doubles()
    result = {}
    failures = []
    nodes = [
        ('bridge_node', 'RealVehicleBridgeNode'),
        ('traj_tracker_node', 'TrajTrackerNode'),
        ('lidar_track_node', 'LidarTrackNode'),
    ]
    for filename, classname in nodes:
        try:
            imported = importlib.import_module(f'rda_dwa_real.{filename}')
            node = getattr(imported, classname)()
            undeclared = sorted(set(node.overrides) - set(node.declared))
            unread = sorted(set(node.overrides) - node.read)
            result[node.name] = {
                'configured_count': len(node.overrides),
                'undeclared_yaml_parameters': undeclared,
                'unread_yaml_parameters': unread,
                'subscriptions': node.subscriptions,
                'publications': sorted(node.publishers),
            }
            if undeclared or unread:
                failures.append(f'{node.name}: undeclared={undeclared}, unread={unread}')
            if filename == 'bridge_node':
                result[node.name]['effective'] = {
                    'planner_v_max': node.planner.params.max_linear_velocity,
                    'planner_v_min': node.planner.params.min_linear_velocity,
                    'horizon': node.planner.params.sim_time,
                    'ack_wheelbase': node.core.ack_model.p.wheelbase,
                    'ack_length': node.core.ack_model.p.length,
                    'ack_width': node.core.ack_model.p.width,
                    'body_to_base_link_x': node._body_to_base_x,
                    'footprint_center_offset': node.core.ack_model.p.footprint_center_offset,
                    'reference_rear_offset': node.core.ack_model.p.reference_rear_offset,
                    'spin_pivot_offset': node.core.cfg.spin_pivot_offset,
                    'reaction_time': node.planner.params.reaction_time,
                }
            elif filename == 'traj_tracker_node':
                result[node.name]['effective'] = vars(node.core.p)
            elif filename == 'lidar_track_node':
                result[node.name]['effective'] = {
                    'body_to_base_link_x': node._body_to_base_x,
                    'max_obstacle_range': node._max_range,
                    'lidar_filter': node._lidar_filter_cfg,
                }
        except Exception as exc:
            failures.append(f'{filename}: {type(exc).__name__}: {exc}')
            result[filename] = {'constructor_error': repr(exc)}
    result['failures'] = failures
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
