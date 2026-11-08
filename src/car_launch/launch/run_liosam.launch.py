import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

def generate_launch_description():
    
    lio_sam_pkg_dir = get_package_share_directory('lio_sam')

    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lio_sam_pkg_dir, 'launch', 'run.launch.py')
        )
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        lio_sam_launch,
    ])