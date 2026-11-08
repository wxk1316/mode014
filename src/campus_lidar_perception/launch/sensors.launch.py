"""Optional existing vehicle drivers; evaluated only when sensors are requested."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _drivers(context):
    def include(prefix):
        package=LaunchConfiguration(prefix+'_package').perform(context)
        file=LaunchConfiguration(prefix+'_launch').perform(context)
        path=os.path.join(get_package_share_directory(package),'launch',file)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
    return [include('lidar'),TimerAction(period=2.,actions=[include('imu')]),
            TimerAction(period=3.,actions=[include('converter')])]


def generate_launch_description():
    defaults=dict(lidar_package='lidar',lidar_launch='start.launch.py',imu_package='imu',
                  imu_launch='ahrs_driver.launch.py',converter_package='rs_to_velodyne',
                  converter_launch='rs_to_velodyne.launch.py')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
                             +[OpaqueFunction(function=_drivers)])
