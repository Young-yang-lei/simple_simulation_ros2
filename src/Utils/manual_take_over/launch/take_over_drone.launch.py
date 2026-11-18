from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('cmd_topic', default_value='position_cmd'),
        Node(package='manual_take_over', executable='manual_take_over', namespace=LaunchConfiguration('namespace'),
             remappings=[('position_cmd', LaunchConfiguration('cmd_topic'))], output='screen'),
    ])
