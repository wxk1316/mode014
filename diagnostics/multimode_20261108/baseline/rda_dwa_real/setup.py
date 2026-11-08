from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rda_dwa_real'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RDA-DWA',
    maintainer_email='dev@rda-dwa.local',
    description='RL-DWA real-vehicle adapter (non-model): external perception/localization/goal in, cmd_vel out.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = rda_dwa_real.bridge_node:main',
            'traj_tracker_node = rda_dwa_real.traj_tracker_node:main',
            'lidar_track_node = rda_dwa_real.lidar_track_node:main',
            'diag_recorder_node = rda_dwa_real.diag_recorder_node:main',
        ],
    },
)
