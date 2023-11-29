from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotiq_3f_gripper_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Hovaldt',
    maintainer_email='andreas.hovaldt@gmail.com',
    description='ROS2 Package for controlling the Robotiq 3 finger gripper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control_listener_node = robotiq_3f_gripper_ros2_control.control_listener_node:main',
            'gripper_control_action_server = robotiq_3f_gripper_ros2_control.control_action_server:main',
        ],
    },
)
