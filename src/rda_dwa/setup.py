from setuptools import setup, find_packages

package_name = 'rda_dwa'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RDA-DWA',
    maintainer_email='dev@rda-dwa.local',
    description='RDA-DWA local path planning algorithm (non-model, fixed horizon EnhancedDWA).',
    license='MIT',
)
