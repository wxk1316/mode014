"""run_rda_dwa.launch.py — 一键启动 RL-DWA 实车完整链路（替代 run_all.launch.py）。

链路：
  run_sensor.launch.py      → 雷达/IMU/rs_to_velodyne（立即启动）
  run_fastlio.launch.py     → FAST_LIO 定位，输出 /localization（延时 6s）
  rl_dwa_car.launch.py      → lidar_track_node + bridge_node + traj_tracker_node + car_ctr
                              （延时 12s，等定位稳定）

与 run_all.launch.py 的区别：
  - 不启动 perception（cloud_seg）：lidar_track_node 直接吃 /rslidar_points，
    内部做地面过滤 + 车体自剔除，不再依赖 cloud_seg 的 /cloud_arround。
  - 不启动 planning（path_planner/globalpathplan）和 control（pathtrack/speedlimit）：
    RL-DWA 接管局部规划与控制，/car_cmd 只有 traj_tracker_node 一个发布者。
  - car_ctr 已内置在 rl_dwa_car.launch.py 里（with_car_ctr:=true 默认），
    remote_ip=192.168.15.15, run_enable=True。

用法：
  ros2 launch car_launch run_rda_dwa.launch.py
"""
import os
from launch.actions import DeclareLaunchArgument          # 顶部 import 处新增
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    car_launch_dir = os.path.join(get_package_share_directory('car_launch'), 'launch')
    rda_dwa_real_dir = get_package_share_directory('rda_dwa_real')


    
    # 1) 传感器（雷达 + IMU + rs_to_velodyne）——立即启动
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_launch_dir, 'run_sensor.launch.py')
        )
    )

    # 2) FAST_LIO 定位——延时 6s（等雷达数据稳定）
    fastlio_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(car_launch_dir, 'run_fastlio.launch.py')
                )
            )
        ]
    )

    # 3) RL-DWA 实车算法栈——延时 12s（等 /localization 稳定输出）
    #    lidar_track_node 直接订阅 /rslidar_points（不再依赖 cloud_seg）
    #    bridge_node 吃 /localization + /rda/obstacles + /move_base_simple/goal → /cmd_vel
    #    traj_tracker_node 把 /cmd_vel → /car_cmd（阿克曼+自转+超时急停）
    #    car_ctr 由 rl_dwa_car.launch.py 内置启动（with_car_ctr:=true 默认）
    rda_dwa_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rda_dwa_real_dir, 'launch', 'rl_dwa_car.launch.py')
                ),
                launch_arguments={
                    'with_car_ctr': 'true',
                    'config':LaunchConfiguration('config'),
                }.items(),
            )
        ]
    )
    diag_recorder_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rda_dwa_real_dir, 'launch', 'diag_recorder.launch.py')
            ),
            launch_arguments={
                # ★2026-11-06：逐消息流保留完整到达，快照20Hz只作概览；
                # 若 SD 卡写入跟不上（snapshots.jsonl 每行 ~1KB → 50Hz≈50KB/s），可降低快照频率，逐消息流仍记录。
                'rec_hz': '20.0',
                'config_file':LaunchConfiguration('config'),
                'record_bag':LaunchConfiguration('record_bag'),
                'bag_max_gb':LaunchConfiguration('bag_max_gb'),
                'experiment_label':LaunchConfiguration('experiment_label'),
                'out_dir':LaunchConfiguration('diag_out_dir'),
                'start_grace': '15.0',   # 算法栈延 12s 才起，空窗内不报 topic_drop
            }.items(),
            condition=IfCondition(LaunchConfiguration('diag')),
        )
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=os.path.join(rda_dwa_real_dir,'config','real_vehicle.yaml')),
        DeclareLaunchArgument('record_bag',default_value='true'),
        DeclareLaunchArgument('diag_out_dir',default_value='/home/nvidia/Desktop/Wxk/car/record_data'),
        DeclareLaunchArgument('bag_max_gb',default_value='8.0'),
        DeclareLaunchArgument('experiment_label',default_value=''),
        DeclareLaunchArgument('diag', default_value='true',
                              description='是否启动诊断记录节点'),
        sensor_launch,
        fastlio_launch,
        rda_dwa_launch,
        diag_recorder_launch,
    ])
