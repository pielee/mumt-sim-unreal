from glob import glob
from setuptools import find_packages, setup

package_name = 'mumt_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ad11',
    maintainer_email='ad11@example.com',
    description='UDP to ROS2 bridge for MUMT_Sim',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = mumt_ros_bridge.bridge_node:main',
        ],
    },
)
