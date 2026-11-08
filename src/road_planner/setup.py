import os
from glob import glob
from setuptools import setup

package_name = 'road_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include data and launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bit',
    maintainer_email='bit@todo.todo',
    description='The road_planner package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'road_planner = road_planner.road_planner:main',
        ],
    },
)
