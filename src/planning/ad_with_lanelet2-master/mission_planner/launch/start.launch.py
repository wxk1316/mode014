from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定义启动参数
    goal_topic_name = DeclareLaunchArgument(
        'goal_topic_name',
        default_value='/move_base_simple/goal'
    )
    
    checkpoint_topic_name = DeclareLaunchArgument(
        'checkpoint_topic_name',
        default_value='/move_base_simple/goal'
    )
    
    rout_topic_name = DeclareLaunchArgument(
        'rout_topic_name',
        default_value='/planning/mission_planning/route'
    )
    
    map_topic_name = DeclareLaunchArgument(
        'map_topic_name',
        default_value='/vector_map'
    )
    
    visualization_topic_name = DeclareLaunchArgument(
        'visualization_topic_name',
        default_value='/planning/mission_planning/route_marker'
    )
    
    # 创建节点
    mission_planner_node = Node(
        package='mission_planner',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[
            {'map_frame': 'map'},
            {'base_link_frame': 'base_link'}
        ],
        remappings=[
            ('input/vector_map', LaunchConfiguration('map_topic_name')),
            ('input/goal_pose', LaunchConfiguration('goal_topic_name')),
            ('input/checkpoint', LaunchConfiguration('checkpoint_topic_name')),
            ('output/route', LaunchConfiguration('rout_topic_name')),
            ('debug/route_marker', LaunchConfiguration('visualization_topic_name'))
        ]
    )
    
    return LaunchDescription([
        goal_topic_name,
        checkpoint_topic_name,
        rout_topic_name,
        map_topic_name,
        visualization_topic_name,
        mission_planner_node
    ])