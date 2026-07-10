import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config_path = os.path.join(
        get_package_share_directory('rov_localization'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # Konwertuje /marines_rov/odometry (natywna, "zejściowa" konwencja Stonefisha,
    # Z rośnie w dół) na /marines_rov/odometry_enu (Z rośnie w górę, zgodnie z resztą
    # stosu - patrz default_depth=-2.0 w pid_controller_node). Patrz komentarz w
    # odometry_converter_node.py po szczegóły.
    odometry_converter = Node(
        package='rov_bridge',
        executable='odometry_converter',
        name='odometry_converter_node'
    )

    return LaunchDescription([
        ekf_node, odometry_converter
    ])
