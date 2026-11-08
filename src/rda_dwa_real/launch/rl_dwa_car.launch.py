"""Real-vehicle launch: lidar tracking -> asynchronous Ackermann DWA supervisor
-> geometric tracker/session executor -> car_ctr UDP bridge. Physical footprint
1.2 x 0.8 m, wheelbase 0.69 m; mode protocol ACK=0 / SPIN=4. The authoritative
plan/session topics are /rda/plan and /rda/session_state. Existing localization
and diagnostic topics remain. Runtime parameters are in real_vehicle.yaml.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 算法包源码目录（可选）：仅在未 colcon build rda_dwa 时需要；通常留空走 install 环境。
    rda_src = os.environ.get('RDA_DWA_SRC', '')
    pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = rda_src if not pythonpath else (rda_src + os.pathsep + pythonpath)
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'real_vehicle.yaml')
    config_file = os.path.abspath(config_file)

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file,
                              description='参数文件路径'),
        DeclareLaunchArgument('with_car_ctr', default_value='true',
                              description='是否在本 launch 内启动 car_ctr（run_enable=True）；'
                                          '若已自行 ros2 launch control run_car_ctr.launch.py 则置 false'),
        SetEnvironmentVariable('PYTHONPATH', new_pythonpath),

        # 1) 激光聚类 + map 系 KF/匈牙利跟踪 → /rda/obstacles（带稳定 id；位姿来自 yaml odom_topic=/localization）
        Node(
            package='rda_dwa_real',
            executable='lidar_track_node',
            name='lidar_track_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        # 2) 规划主节点：话题全部硬编码（无 remap）——订阅 /localization + /rda/obstacles + /move_base_simple/goal + /amcl_pose → 发布 /cmd_vel
        Node(
            package='rda_dwa_real',
            executable='bridge_node',
            name='rda_dwa_real_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        # 3) 跟踪控制：/rda/traj + /rda/traj_clear + /cmd_vel + /localization
        #    + /rda/spin_target + /rda/traj_mode → /car_cmd
        #    （固定 50Hz：完整参考替换 + 几何投影 LQR + 限速/加速度/jerk，只下发期望转角/期望速度；
        #    自转模式只按目标角闭环。替代 control_bridge_node 的 5~7Hz 直发转角，后者保留可回滚）
        Node(
            package='rda_dwa_real',
            executable='traj_tracker_node',
            name='traj_tracker_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        # 4) 底盘桥（可选）：/car_cmd → UDP → 底盘；run_enable=True 否则 car_ctr 直接 return 不下发
        Node(
            package='car_ctr',
            executable='car_ctr',
            name='car_ctr',
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_car_ctr')),
            parameters=[{
                'remote_ip': '192.168.15.15',
                'run_enable': True,
                'mode_change_time': 3.0,
                'spin_command_scale': 0.429,
                'spin_sign': -1.0,
                'wheel_feedback_scale': 1.0,
            }],
        ),
    ])
