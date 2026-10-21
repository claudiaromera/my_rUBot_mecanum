from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        'follow_distance': LaunchConfiguration('follow_distance'),
        'max_front_length': LaunchConfiguration('max_front_length'),
        'forward_speed': LaunchConfiguration('forward_speed'),
        'lateral_speed': LaunchConfiguration('lateral_speed'),
        'distance_gain': LaunchConfiguration('distance_gain'),
        'angle_gain': LaunchConfiguration('angle_gain'),
        'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        'detection_limit': LaunchConfiguration('detection_limit'),
        'back_right_limit_factor': LaunchConfiguration(
            'back_right_limit_factor'
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='0.35'),
        DeclareLaunchArgument('max_front_length', default_value='0.50'),
        DeclareLaunchArgument('forward_speed', default_value='0.30'),
        DeclareLaunchArgument('lateral_speed', default_value='0.40'),
        DeclareLaunchArgument('distance_gain', default_value='0.80'),
        DeclareLaunchArgument('angle_gain', default_value='1.20'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.00'),
        DeclareLaunchArgument('detection_limit', default_value='1.20'),
        DeclareLaunchArgument(
            'back_right_limit_factor', default_value='1.50'
        ),
        Node(
            package='my_robot_control',
            executable='my_robot_wallfollower_holonomic_improved_exec',
            name='improved_holonomic_wall_follower',
            output='screen',
            parameters=[parameters],
        ),
    ])
