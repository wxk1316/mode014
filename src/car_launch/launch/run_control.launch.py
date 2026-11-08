import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # pathtrack 主节点
    pathtrack_node = Node(
        package='pathtrack',
        executable='pathtrack', 
        name='pathtrack',
        output='screen',
        parameters=[{
            'ref_speed': 1.0,
            'angle_offset': 0.0, # 右偏为负, 左偏为正
            'run_enable': True,
            'use_sim_time': False,
            'check/node_rate': 10.0,
            'check/track_rate': 10.0,
            'check/msg': ''
        }]
    )

    # speedlimit 限速节点
    speedlimit_node = Node(
        package='pathtrack',
        executable='speedlimit',
        name='speedlimit',
        output='screen',
        parameters=[{
            'ref_speed': 1.5,
            'obstacle_enable': True,
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        pathtrack_node,
        speedlimit_node,
    ])

