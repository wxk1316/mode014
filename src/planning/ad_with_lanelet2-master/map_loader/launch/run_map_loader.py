# launch/run_proper_sequence.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bl_hg',
        arguments=['5', '-9', '0', '0', '0', '0', '1', 'map', 'base_link']
    )

    # 参数定义
    file_name_arg = DeclareLaunchArgument(
        'file_name',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_loader'), 'data', 'indoor_airy(2).osm'
        ])
    )
    
    # 1. 先启动map系统
    loader_node = Node(
        package='map_loader',
        executable='lanelet2_map_loader',
        name='lanelet2_map_loader',
        arguments=[LaunchConfiguration('file_name')],
        remappings=[('output/lanelet2_map', 'vector_map')],
        output='screen'
    )
    
    visualization_node = Node(
        package='map_loader',
        executable='lanelet2_map_visualization',
        name='lanelet2_map_visualization',
        remappings=[
            ('input/lanelet2_map', 'vector_map'),
            ('output/lanelet2_map_marker', 'vector_map_marker')
        ],
        output='screen'
    )

    # 1. RViz2节点（立即启动）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('map_loader'), 'loader.rviz'
        ])],
        output='screen'
    )  

    delayed_loader = TimerAction(
        period=3.0,
        actions=[            
            loader_node,
            LogInfo(msg="Starting visualization_node system...")
        ]
    )      
    

    
    return LaunchDescription([
        # static_transform_publisher,
        file_name_arg,
        # rviz_node,    
        LogInfo(msg="Waiting 3 seconds before starting loader_node..."),                         
        delayed_loader,             
        visualization_node
    ])