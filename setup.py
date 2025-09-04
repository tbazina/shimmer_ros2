import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'shimmer_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomislav',
    maintainer_email='tbazina@gmail.com',
    description='ROS2 interface to Shimmer3 EMG sensors.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = shimmer_ros2.nodes.simple_publisher:main',
            'shimmer_publisher = shimmer_ros2.nodes.shimmer_publisher:main',
            'emg_processor = shimmer_ros2.nodes.emg_processor:main',
        ],
    },
    scripts=['scripts/bind_shimmer_to_rfcomm.sh'],
)
