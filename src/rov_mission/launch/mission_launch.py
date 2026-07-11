import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('rov_mission'), 'config', 'mission_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Path to the YAML file with mission parameters (goal, cruise depth, frames).')

    return LaunchDescription([
        params_file_arg,
        Node(
            package='rov_mission',
            executable='mission',
            name='mission_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
