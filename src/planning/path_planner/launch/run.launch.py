import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('path_planner')
    road_planner_share = get_package_share_directory('road_planner')
    config_file = os.path.join(pkg_share, 'config', 'path_planner.yaml')
    road_graph_path = os.path.join(road_planner_share, 'data', 'road_graph.pkl')

    return LaunchDescription([
        Node(
            package='path_planner',
            executable='path_planner_node.py',
            name='path_planner',
            output='screen',
            parameters=[config_file, {'roadmap_path': road_graph_path}],
        ),
    ])
