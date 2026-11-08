import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    mqtt_host = LaunchConfiguration('mqtt_host')
    mqtt_port = LaunchConfiguration('mqtt_port')
    pub_mqtt_topic = LaunchConfiguration('pub_mqtt_topic')
    sub_mqtt_topic = LaunchConfiguration('sub_mqtt_topic')
    local_vehicle_topic = LaunchConfiguration('local_vehicle_topic')
    remote_vehicle_topic = LaunchConfiguration('remote_vehicle_topic')
    rate = LaunchConfiguration('rate')

    odom_topic = LaunchConfiguration('odom_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    car_state_topic = LaunchConfiguration('car_state_topic')
    car_cmd_topic = LaunchConfiguration('car_cmd_topic')
    goal_topic = LaunchConfiguration('goal_topic')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_4'),
        DeclareLaunchArgument('mqtt_host', default_value='192.168.8.199'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('pub_mqtt_topic', default_value='__auto__'),
        DeclareLaunchArgument('sub_mqtt_topic', default_value='vehicles/+'),
        DeclareLaunchArgument('local_vehicle_topic', default_value='/mqtt/local_vehicle_state'),
        DeclareLaunchArgument('remote_vehicle_topic', default_value='/mqtt/remote_vehicle_state'),
        DeclareLaunchArgument('rate', default_value='10.0'),
        DeclareLaunchArgument('odom_topic', default_value='/localization'),
        DeclareLaunchArgument('pose_topic', default_value='/current_pose'),
        DeclareLaunchArgument('car_state_topic', default_value='/car_state'),
        DeclareLaunchArgument('car_cmd_topic', default_value='/car_cmd'),
        DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal'),

        Node(
            package='mqtt_comm',
            executable='mqtt_vehicle_pub',
            name='mqtt_vehicle_pub',
            output='screen',
            respawn=True,
            parameters=[{
                'robot_id': robot_id,
                'mqtt_host': mqtt_host,
                'mqtt_port': mqtt_port,
                'mqtt_topic': pub_mqtt_topic,
                'client_id': '__auto__',
                'local_vehicle_topic': local_vehicle_topic,
                'rate': rate,
                'odom_topic': odom_topic,
                'pose_topic': pose_topic,
                'car_state_topic': car_state_topic,
                'car_cmd_topic': car_cmd_topic,
                'goal_topic': goal_topic,
            }]
        ),

        Node(
            package='mqtt_comm',
            executable='mqtt_vehicle_sub',
            name='mqtt_vehicle_sub',
            output='screen',
            respawn=True,
            parameters=[{
                'robot_id': robot_id,
                'mqtt_host': mqtt_host,
                'mqtt_port': mqtt_port,
                'mqtt_topic': sub_mqtt_topic,
                'client_id': '__auto__',
                'remote_vehicle_topic': remote_vehicle_topic,
                'ignore_own_id': True,
            }]
        ),
    ])
