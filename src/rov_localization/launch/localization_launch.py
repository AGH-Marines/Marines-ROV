import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config', default_value='ekf.yaml',
        description='Params file name in rov_localization/config, e.g. '
                     'ekf.yaml (simulation) or ekf_jetson.yaml (real ROV + ZED)')

    ekf_config_path = PathJoinSubstitution([
        get_package_share_directory('rov_localization'),
        'config',
        LaunchConfiguration('ekf_config'),
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    odometry_converter = Node(
        package='rov_bridge',
        executable='odometry_converter',
        name='odometry_converter_node'
    )

    return LaunchDescription([
        ekf_config_arg, ekf_node, odometry_converter
    ])
