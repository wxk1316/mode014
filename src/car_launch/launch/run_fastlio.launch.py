import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 注意：必须使用软链接编码的方式，否则在不同系统上可能会出现路径问题 colcon build --symlink-install
    pkg_name = 'fast_lio' 
    try:
        pkg_share = get_package_share_directory(pkg_name)
    except KeyError:
        print(f"Error: 找不到包 {pkg_name}")
        return LaunchDescription([])
    
    # ================= 路径定义 ================= 
    # Lidar 配置
    config_file = os.path.join(pkg_share, 'config', 'velodyne.yaml')
    fast_lio_src_dir = '/home/nvidia/Desktop/car_0513_wjh/src/localization/FAST_LIO'
    map_file_s = os.path.join(fast_lio_src_dir, 'PCD', 'global_ss.pcd')
    map_file_global = os.path.join(fast_lio_src_dir, 'PCD', 'global1226.pcd')
    global_loc_config = os.path.join(pkg_share, 'config', 'initial_pose.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'global.rviz')
    
    # ================= 节点定义 =================
    # TF 静态变换节点 ---
    # 1. body -> base_link
    tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body2base_link',
        arguments=[
            '--x', '-0.15', 
            '--y', '0', 
            '--z', '0', 
            '--yaw', '0', 
            '--pitch', '0', 
            '--roll', '0', 
            '--frame-id', 'body', 
            '--child-frame-id', 'base_link'
        ],
        output='screen'
    )

    # 2. base_link -> rslidar
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_base_link',
        arguments=[
            '--x', '0', 
            '--y', '0', 
            '--z', '0.45', 
            '--yaw', '0', 
            '--pitch', '0', 
            '--roll', '0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'rslidar'
        ],
        output='screen'
    )

    # 3. FastLIO 里程计
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )

    # 4. 地图发布 (PCL ROS)
    pcd_publisher_node = Node(
        package='pcl_ros', 
        executable='pcd_to_pointcloud', 
        name='map_publisher',
        output='screen',
        parameters=[{
            'file_name': map_file_s,
            'tf_frame': 'map',
            'publishing_period_ms': 2000 
        }],
        remappings=[('cloud_pcd', '/map')]
    )

    # 5. 全局定位 (Global Localization)
    global_loc_node = Node(
        package='fast_lio',
        executable='global_localization.py', 
        name='global_localization',
        output='screen',
        parameters=[{
            'filepath': global_loc_config,  
            'map_file': map_file_global,
            'load_init_pose_mode': 2, 
            'save_interval': 3.0  # s    
        }]
    )

    # 6. 坐标转换融合 (Transform Fusion)
    transform_fusion_node = Node(
        package='fast_lio',
        executable='transform_fusion.py',
        name='transform_fusion',
        output='screen'
    )

    # 7. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], 
        output='screen'
    )

    return LaunchDescription([
        tf_body_to_base,        # 新增
        tf_base_to_lidar,       # 新增
        fastlio_node,
        pcd_publisher_node,
        global_loc_node,
        transform_fusion_node,
        rviz_node  
    ])
