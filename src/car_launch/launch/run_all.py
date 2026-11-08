import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('car_launch'), 'launch')
    mqtt_share = get_package_share_directory('mqtt_comm')

    # ---- 多车/云端 MQTT 参数（默认：本车 robot_4，broker = 主站 192.168.8.199）----
    mqtt_robot_id = LaunchConfiguration('mqtt_robot_id')
    mqtt_host = LaunchConfiguration('mqtt_host')
    mqtt_start_sub = LaunchConfiguration('mqtt_start_sub')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_sensor.launch.py'))
    )

    fastlio_launch = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_fastlio.launch.py'))
        )]
    )

    perception_launch = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_perception.launch.py'))
        )]
    )

    # 新 service/action 路网任务链路，替代原 run_planning + run_control。
    action_navigation_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_action_navigation.launch.py'))
        )]
    )

    # MQTT 多车/云端链路：连接主站 broker（192.168.8.199），接收云端下发的目标点/路径、
    # 上报本车状态，从而把新车纳入云端系统的控制。
    mqtt_launch = TimerAction(
        period=9.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mqtt_share, 'launch', 'run.launch.py')),
            launch_arguments={
                'robot_id': mqtt_robot_id,
                'mqtt_host': mqtt_host,
                'start_sub': mqtt_start_sub,
            }.items(),
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument('mqtt_robot_id', default_value='robot_4'),
        DeclareLaunchArgument('mqtt_host', default_value='192.168.8.199'),
        DeclareLaunchArgument('mqtt_start_sub', default_value='true'),
        sensor_launch,
        fastlio_launch,
        # perception_launch,
        action_navigation_launch,
        mqtt_launch,
    ])
