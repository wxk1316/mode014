import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    # 1. 获取包的路径
    launch_dir = os.path.join(get_package_share_directory('car_launch'), 'launch')

    # 2. 定义各个 Launch 文件
    # (1) Sensor - 立即启动
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_sensor.launch.py'))
    )

    # # (2) Bitcar 底盘 - 立即启动 (遥控器+电池+泵站+底盘驱动)
    # bitcar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_bitcar.launch.py'))
    # )

    # (3) FastLIO - 延时 6 秒启动 (等待雷达数据稳定)
    fastlio_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_fastlio.launch.py'))
            )
        ]
    )

    # (4) Perception - 延时 6 秒启动 (等待定位和点云)
    perception_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_perception.launch.py'))
            )
        ]
    )

    # (5) Planning - 延时 6 秒启动
    planning_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_planning.launch.py'))
            )
        ]
    )

    # (6) Control (pathtrack + speedlimit) - 延时 6 秒启动
    control_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_control.launch.py'))
            )
        ]
    )


    # (7) Foxglove Bridge - 立即启动
    foxglove_bridge_launch = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    # 3. 返回 LaunchDescription
    return LaunchDescription([
        # foxglove_bridge_launch,
        sensor_launch,
        # bitcar_launch,
        fastlio_launch,
        # perception_launch,
        planning_launch,
        control_launch,
    ])