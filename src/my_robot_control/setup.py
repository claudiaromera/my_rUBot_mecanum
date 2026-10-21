from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='manel.puig@ub.edu',
    description='TODO: Package description',
    license='Apache License 2.0', # improved license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_lidar_test_exec = my_robot_control.my_robot_lidar_test:main',
            'my_robot_control_exec = my_robot_control.my_robot_control:main',
            'my_robot_selfcontrol_exec = my_robot_control.my_robot_selfcontrol:main',
            'my_robot_wallfollower_differential_exec = my_robot_control.my_robot_wallfollower_differential:main',
            'my_robot_wallfollower_holonomic_exec = my_robot_control.my_robot_wallfollower_holonomic:main',
            'my_robot_wallfollower_holonomic_improved_exec = my_robot_control.my_robot_wallfollower_holonomic_improved:main',
            'my_robot_wallfollower_holonomic_obstacle_exec = my_robot_control.my_robot_wallfollower_holonomic_obstacle:main',
            'my_robot_go2pose_exec = my_robot_control.my_robot_go2pose:main',
            'my_robot_set_pose_gazebo_exec = my_robot_control.my_robot_set_pose_gazebo:main',
        ],
    },
)
