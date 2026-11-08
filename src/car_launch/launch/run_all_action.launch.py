import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('car_launch'), 'launch')

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


    return LaunchDescription([
        sensor_launch,
        fastlio_launch,
        perception_launch,
        action_navigation_launch,
    ])
