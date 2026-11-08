import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    obs_check_share = get_package_share_directory('obs_check')

    # 节点: cloud_seg
    cloud_seg_node = Node(
        package='obs_check',
        executable='cloud_seg',
        name='cloud_seg',
        output='screen'
    )

    # 节点: obs_check_inlane
    obs_check_inlane_node = Node(
        package='obs_check',
        executable='obs_check_inlane',
        name='obs_check_inlane',
        output='screen'
    )

    return LaunchDescription([
        cloud_seg_node,
        obs_check_inlane_node
    ])