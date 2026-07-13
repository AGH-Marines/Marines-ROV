import os
import random
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# Available map/scenario variants, mapped to their .scn file names
WORLD_SCENARIOS = {
    'underwater': 'underwater_world.scn',
    'labirynt': 'labirynt.scn',
    'window_wall': 'window_wall.scn.template',
}

# Bounds for the randomized window position in window_wall (corridor is 10m wide, 4m tall)
WINDOW_Y_RANGE = (-3.0, 3.0)
WINDOW_Z_RANGE = (1.0, 19.0)


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)

    if world not in WORLD_SCENARIOS:
        raise ValueError(
            f"Unknown world '{world}'. Available options: {list(WORLD_SCENARIOS.keys())}"
        )

    scenario_file = os.path.join(
        get_package_share_directory('rov_simulation'),
        'scenarios',
        WORLD_SCENARIOS[world]
    )

    if world == 'window_wall':
        with open(scenario_file, 'r') as f:
            template = f.read()

        content = template.format(
            window_center_y=random.uniform(*WINDOW_Y_RANGE),
            window_center_z=random.uniform(*WINDOW_Z_RANGE),
        )

        generated = tempfile.NamedTemporaryFile(
            mode='w', suffix='.scn', prefix='window_wall_', delete=False
        )
        generated.write(content)
        generated.close()
        scenario_file = generated.name

    simulation_data = os.path.join(
        get_package_share_directory('rov_simulation'),
        'data'
    )

    # Stonefish simulator node
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

    return [stonefish_node]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='underwater',
        choices=list(WORLD_SCENARIOS.keys()),
        description='Wariant mapy symulacji do wczytania'
    )

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])
