import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Ścieżki do plików pakietu
    scenario_file = os.path.join(
        get_package_share_directory('rov_simulation'),
        'scenarios',
        'underwater_world.scn'
    )

    simulation_data = os.path.join(
        get_package_share_directory('rov_simulation'),
        'data'
    )

    # Węzeł symulatora Stonefish
    stonefish_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[
            simulation_data,
            scenario_file,
            '300.0',
            '1280',
            '720',
            'low'
        ],
        output='screen',
    )

    return LaunchDescription([
        stonefish_node
    ])
