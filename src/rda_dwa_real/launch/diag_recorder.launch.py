"""Record arrival-indexed JSONL and optional bounded raw rosbag, without commands.
Snapshots are asynchronous caches; messages_*.jsonl retain each callback message.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    defaults={
        'rec_hz':('20.0',float,'快照频率；逐消息记录不受此频率限制'),
        'out_dir':('',str,'输出根目录，空串用 ~/ros2_diag'),
        'config_file':(os.path.join(get_package_share_directory('rda_dwa_real'),'config','real_vehicle.yaml'),str,'与算法使用相同的配置文件副本'),
        'record_gps':('false',bool,'是否额外记录 /fix'),
        'record_bag':('true',bool,'记录原始点云/IMU/TF及关键消息 rosbag'),
        'bag_extra_topics':('',str,'额外原始话题，逗号或空格分隔'),
        'bag_max_gb':('8.0',float,'原始 bag 近似总容量上限 GiB，达到后仅停止 bag'),
        'bag_min_free_gb':('2.0',float,'原始 bag 保留的磁盘空闲 GiB'),
        'drop_timeout':('3.0',float,'连续话题无消息提示时间 s'),
        'start_grace':('15.0',float,'启动阶段不报告缺失话题 s'),
        'rate_window':('2.0',float,'接收频率统计窗口 s'),
        'writer_queue_records':('4096',int,'JSON 写入待处理记录数上限'),
        'writer_queue_mb':('32.0',float,'JSON 待写队列字节上限 MiB'),
        'message_segment_mb':('64.0',float,'逐消息 JSONL 分卷大小 MiB'),
        'experiment_label':('',str,'本轮实验名称或说明'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v[0],description=v[2]) for k,v in defaults.items()],
        Node(package='rda_dwa_real',executable='diag_recorder_node',name='diag_recorder_node',output='screen',parameters=[{k:ParameterValue(LaunchConfiguration(k),value_type=v[1]) for k,v in defaults.items()}]),
    ])
