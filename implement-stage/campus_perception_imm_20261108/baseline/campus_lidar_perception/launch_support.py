"""New launch composition only. Existing car_launch files stay untouched."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC,PathJoinSubstitution
from launch_ros.actions import Node,SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name,default,description=''):
    return DeclareLaunchArgument(name,default_value=str(default),description=description)


def _parameters():
    share=Path(get_package_share_directory('campus_lidar_perception'))
    return [_arg('profile','centerpoint_harpnext','centerpoint_geometry vs centerpoint_harpnext; harp is the same model alias'),
            _arg('perception_config',share/'config/perception.yaml'),
            _arg('model_dir',share/'models'),
            _arg('engine_dir',Path.home()/'.cache/campus_lidar_perception/engines'),
            _arg('native_library',''),_arg('points_topic','/rslidar_points'),
            _arg('odom_topic','/localization'),_arg('obstacles_topic','/rda/obstacles'),
            _arg('localization_source','latest_available'),_arg('body_to_base_link_x','0.0'),
            _arg('sensor_x','0.0'),_arg('sensor_y','0.0'),_arg('sensor_z','0.45'),
            _arg('sensor_roll','0.0'),_arg('sensor_pitch','0.0'),_arg('sensor_yaw','0.0'),_arg('use_sim_time','false'),
            _arg('with_perception_record','true'),_arg('perception_record_dir',Path.home()/'record_data/campus_perception')]


def _perception_node():
    typed_bool=lambda name:ParameterValue(LC(name),value_type=bool)
    typed_float=lambda name:ParameterValue(LC(name),value_type=float)
    return Node(package='campus_lidar_perception',executable='campus_perception_node',
                name='campus_lidar_perception',output='screen',
                parameters=[LC('perception_config'),
                            {k:LC(k) for k in ('profile','model_dir','engine_dir','native_library','points_topic','odom_topic','obstacles_topic','localization_source')},
                            dict(use_sim_time=typed_bool('use_sim_time'),
                                 **{k:typed_float(k) for k in ('body_to_base_link_x','sensor_x','sensor_y','sensor_z','sensor_roll','sensor_pitch','sensor_yaw')})])


def _sensors():
    share=Path(get_package_share_directory('car_launch'))
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/run_sensor.launch.py')),
                                    condition=IfCondition(LC('with_sensors')))


def _record_node():
    return Node(package='campus_lidar_perception',executable='campus_perception_record',
                name='campus_perception_recorder',output='screen',
                parameters=[dict(out_dir=LC('perception_record_dir'),profile=LC('profile'),
                                 use_sim_time=ParameterValue(LC('use_sim_time'),value_type=bool))],
                condition=IfCondition(LC('with_perception_record')))


def detection_launch():
    return LaunchDescription(_parameters()+[_arg('with_sensors','false'),
                                            SetParameter(name='use_sim_time',value=LC('use_sim_time')),
                                            _sensors(),_perception_node(),_record_node()])


def _localization_args():
    share=Path(get_package_share_directory('fast_lio'))
    maps=Path(os.environ.get('FAST_LIO_MAP_DIR',str(share/'PCD')))
    return [_arg('fastlio_config',share/'config/velodyne.yaml'),
            _arg('fastlio_map_dir',maps,'Directory containing existing global_ss.pcd/global1226.pcd'),
            DeclareLaunchArgument('map_cloud_file',default_value=PathJoinSubstitution([LC('fastlio_map_dir'),'global_ss.pcd'])),
            DeclareLaunchArgument('localization_map_file',default_value=PathJoinSubstitution([LC('fastlio_map_dir'),'global1226.pcd'])),
            _arg('initial_pose_file',share/'config/initial_pose.yaml'),
            _arg('with_rviz','false'),
            _arg('rviz_config',Path(get_package_share_directory('campus_lidar_perception'))/'config/campus_perception.rviz')]


def _localization_nodes():
    sim={'use_sim_time':ParameterValue(LC('use_sim_time'),value_type=bool)}
    # These are the same executables as the old FAST_LIO launch. The reference
    # offset follows current real_vehicle.yaml, rather than its obsolete -0.15 TF.
    return [Node(package='tf2_ros',executable='static_transform_publisher',name='campus_body_to_base',
                 arguments=['--x',LC('body_to_base_link_x'),'--y','0','--z','0',
                            '--yaw','0','--pitch','0','--roll','0','--frame-id','body','--child-frame-id','base_link'],parameters=[sim]),
            Node(package='tf2_ros',executable='static_transform_publisher',name='campus_base_to_lidar',
                 arguments=['--x',LC('sensor_x'),'--y',LC('sensor_y'),'--z',LC('sensor_z'),
                            '--yaw',LC('sensor_yaw'),'--pitch',LC('sensor_pitch'),'--roll',LC('sensor_roll'),
                            '--frame-id','base_link','--child-frame-id','rslidar'],parameters=[sim]),
            Node(package='fast_lio',executable='fastlio_mapping',name='laserMapping',output='screen',parameters=[LC('fastlio_config'),sim]),
            Node(package='pcl_ros',executable='pcd_to_pointcloud',name='map_publisher',output='screen',
                 parameters=[dict(file_name=LC('map_cloud_file'),tf_frame='map',publishing_period_ms=2000),sim],remappings=[('cloud_pcd','/map')]),
            Node(package='fast_lio',executable='global_localization.py',name='global_localization',output='screen',
                 parameters=[dict(filepath=LC('initial_pose_file'),map_file=LC('localization_map_file'),load_init_pose_mode=2,save_interval=3.),sim]),
            Node(package='fast_lio',executable='transform_fusion.py',name='transform_fusion',output='screen',parameters=[sim],
                 remappings=[('/localization',LC('odom_topic'))]),
            Node(package='rviz2',executable='rviz2',name='rviz2',arguments=['-d',LC('rviz_config')],
                 condition=IfCondition(LC('with_rviz')),parameters=[sim])]


def localization_launch():
    common=[_arg('use_sim_time','false'),_arg('body_to_base_link_x','0.0'),
            _arg('sensor_x','0.0'),_arg('sensor_y','0.0'),_arg('sensor_z','0.45'),
            _arg('sensor_roll','0.0'),_arg('sensor_pitch','0.0'),_arg('sensor_yaw','0.0'),
            _arg('with_sensors','true'),_arg('odom_topic','/localization')]
    return LaunchDescription(common+_localization_args()+[SetParameter(name='use_sim_time',value=LC('use_sim_time')),_sensors()]+_localization_nodes())


def combined_launch(full_stack=False):
    args=_parameters()+_localization_args()+[_arg('with_sensors','true')]
    nodes=[SetParameter(name='use_sim_time',value=LC('use_sim_time')),_sensors()]+_localization_nodes()+[_perception_node(),_record_node()]
    if full_stack:
        rda=Path(get_package_share_directory('rda_dwa_real'))
        args += [_arg('planner_config',rda/'config/real_vehicle.yaml'),_arg('with_car_ctr','true'),
                 _arg('remote_ip','192.168.15.15'),_arg('with_diag','true'),
                 _arg('record_bag','true'),_arg('diag_out_dir',Path.home()/'record_data/campus_perception'),
                 _arg('bag_max_gb','8.0'),_arg('experiment_label','campus_perception')]
        shared=[LC('planner_config'),{'use_sim_time':ParameterValue(LC('use_sim_time'),value_type=bool),
                                    'body_to_base_link_x':ParameterValue(LC('body_to_base_link_x'),value_type=float)}]
        nodes += [Node(package='rda_dwa_real',executable='bridge_node',name='rda_dwa_real_bridge',output='screen',
                       parameters=shared,remappings=[('/rda/obstacles',LC('obstacles_topic')),('/localization',LC('odom_topic'))]),
                  Node(package='rda_dwa_real',executable='traj_tracker_node',name='traj_tracker_node',output='screen',
                       parameters=shared,remappings=[('/rda/obstacles',LC('obstacles_topic')),('/localization',LC('odom_topic'))]),
                  Node(package='car_ctr',executable='car_ctr',name='car_ctr',output='screen',
                       parameters=[dict(remote_ip=LC('remote_ip'),run_enable=True,mode_change_time=3.)],
                       condition=IfCondition(LC('with_car_ctr'))),
                  IncludeLaunchDescription(PythonLaunchDescriptionSource(str(rda/'launch/diag_recorder.launch.py')),
                    condition=IfCondition(LC('with_diag')),
                    launch_arguments={'rec_hz':'20.0','config_file':LC('planner_config'),
                                      'record_bag':LC('record_bag'),'bag_max_gb':LC('bag_max_gb'),
                                      'experiment_label':LC('experiment_label'),'out_dir':LC('diag_out_dir'),
                                      'start_grace':'15.0'}.items())]
    return LaunchDescription(args+nodes)
