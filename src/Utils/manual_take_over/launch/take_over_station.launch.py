from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='joy', executable='joy_node', namespace=LaunchConfiguration('namespace')),
        Node(package='manual_take_over', executable='manual_take_over_station', namespace=LaunchConfiguration('namespace'), output='screen'),
    ])
