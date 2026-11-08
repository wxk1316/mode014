from pathlib import Path
from setuptools import setup,find_packages

NAME='campus_lidar_perception'
data=[('share/ament_index/resource_index/packages',['resource/'+NAME]),
      ('share/'+NAME,['package.xml','README.md','THIRD_PARTY.md','LICENSE','requirements-orin.txt'])]
for directory in ('config','launch','models','scripts','native'):
    for parent in sorted({p.parent for p in Path(directory).rglob('*') if p.is_file()
                          and 'build' not in p.parts and '__pycache__' not in p.parts}):
        files=[str(p) for p in parent.iterdir() if p.is_file() and not p.name.endswith('.part')]
        if files:data.append(('share/'+NAME+'/'+str(parent),files))
setup(name=NAME,version='0.1.0',packages=find_packages(exclude=['tests']),
      data_files=data,package_data={NAME:['lib/*.so'],NAME+'.vendor.harpnext':['LICENSE']},
      install_requires=['setuptools','numpy>=1.24','scipy>=1.8','PyYAML>=6'],zip_safe=False,
      maintainer='car_gpt',maintainer_email='dev@car-gpt.local',
      description='Pretrained LiDAR perception with planner-compatible tracked objects',license='Apache-2.0',
      entry_points={'console_scripts':['campus_perception_node = campus_lidar_perception.node:main',
                                       'campus_perception_replay = campus_lidar_perception.replay:main',
                                       'campus_perception_record = campus_lidar_perception.record_node:main',
                                       'campus_perception_compare = campus_lidar_perception.comparison:main']})
