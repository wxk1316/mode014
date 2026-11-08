import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 road_planner 包的系统安装路径
    road_planner_share_dir = get_package_share_directory('road_planner')

    # 拼接绝对路径 (等同于 ROS 1 中的 $(find road_planner)/...)
    pcd_file_path = os.path.join(road_planner_share_dir, 'data', 'indoor.pcd')
    # rviz_config_path = os.path.join(road_planner_share_dir, 'data', 'display.rviz')

    # 1. pcd_to_pointcloud 点云发布节点
    pcd_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',  # 注意：ROS 2 中通常带有 _node 后缀
        name='pcd_publisher',
        output='screen',
        parameters=[{
            # 'frame_id': 'map',
            'tf_frame': 'map',  # ROS 2 中改为 tf_frame
            'latch': False,
            'file_name': pcd_file_path,
            'interval': 2.0
        }],
        remappings=[('cloud_pcd', '/map')]
    )

    # 2. road_planner Python 算法节点
    road_planner_node = Node(
        package='road_planner',
        executable='road_planner',  # 必须确保在 setup.py 或 CMakeLists.txt 中已注册
        name='road_planner',
        output='screen'
    )

    # 3. RViz2 可视化节点
    rviz_node = Node(
        package='rviz2',           # ROS 2 中包名改成了 rviz2
        executable='rviz2',        # 可执行文件也叫 rviz2
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        # pcd_publisher_node,
        road_planner_node,
        # rviz_node
    ])