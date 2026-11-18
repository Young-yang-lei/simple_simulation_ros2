"""Standalone simulator with interchangeable ideal and SO3 dynamics backends."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def setup(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('fake', 'so3'):
        raise ValueError('backend must be fake or so3')
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    body = (namespace + '/' if namespace else '') + 'base_link'
    depth = LaunchConfiguration('depth').perform(context).lower() == 'true'
    params = {'init_x': 0.0, 'init_y': 0.0, 'init_z': 1.0}
    nodes = [
        Node(package='map_generator', executable='random_forest', namespace=namespace,
             parameters=[{'map.x_size': 35.0, 'map.y_size': 35.0, 'map.z_size': 3.0,
                          'map.obs_num': 20, 'map.moving_obs_num': 10, 'map.circle_num': 0,
                          'seed': 3728542744, 'pub_rate': 10.0}], output='screen'),
        Node(package='local_sensing_node', executable='pcl_render_node', namespace=namespace,
             parameters=[{'sensing_horizon': 8.0, 'sensing_rate': 30.0, 'publish_depth': depth}],
             remappings=[('global_map', 'map_generator/static_obs_cloud'),
                         ('local_map', 'map_generator/obj_cloud')], output='screen'),
        Node(package='odom_visualization', executable='odom_visualization', namespace=namespace,
             parameters=[{'robot_scale': 0.3, 'drone_id': 0}],
             remappings=[('odom', 'odometry'), ('cmd', 'position_cmd')], output='screen'),
        Node(package='uav_utils', executable='tf_assist.py', namespace=namespace,
             parameters=[{'body_frame': body}], remappings=[('odom', 'odometry'), ('path', 'tf_path')]),
    ]
    if backend == 'fake':
        nodes.append(Node(package='poscmd_2_odom', executable='poscmd_2_odom', namespace=namespace,
                          parameters=[params, {'body_frame': body}],
                          remappings=[('command', 'position_cmd')], output='screen'))
    else:
        nodes.extend([
            Node(package='so3_quadrotor_simulator', executable='quadrotor_simulator_so3', namespace=namespace,
                 parameters=[{'simulator.init_state_x': 0.0, 'simulator.init_state_y': 0.0,
                              'simulator.init_state_z': 1.0, 'mass': 0.9, 'quadrotor_name': body}],
                 remappings=[('odom', 'odometry'), ('cmd', 'so3_cmd')], output='screen'),
            Node(package='so3_control', executable='so3_control_node', namespace=namespace,
                 parameters=[{'so3_control.init_state_x': 0.0, 'so3_control.init_state_y': 0.0,
                              'so3_control.init_state_z': 1.0, 'mass': 0.9, 'use_external_yaw': False,
                              'quadrotor_name': body}], remappings=[('odom', 'odometry')], output='screen'),
        ])
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        config = os.path.join(get_package_share_directory('so3_quadrotor_simulator'), 'config', 'rviz.rviz')
        nodes.append(Node(package='rviz2', executable='rviz2', namespace=namespace, arguments=['-d', config]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='fake', description='fake or so3'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('depth', default_value='false'),
        OpaqueFunction(function=setup),
    ])
