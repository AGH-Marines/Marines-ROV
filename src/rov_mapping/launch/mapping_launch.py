from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='odom')
    map_frame = LaunchConfiguration('map_frame')

    # Stonefish nigdy nie publikuje TF dla ram sensorów (tylko world_ned -> ROV/base_link),
    # więc octomap_server nie ma jak zrzutować /marines_rov/point_cloud (frame_id=ROV/multibeam_3d,
    # sensor jest prefiksowany nazwą robota "ROV") do ramy mapy bez tej statycznej transformacji.
    # Offset skopiowany z underwater_world.scn.
    multibeam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='multibeam_static_tf',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'ROV/multibeam_3d',
        ],
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.2,
            'frame_id': map_frame,
            'base_frame_id': 'base_link',
            'sensor_model.max_range': 15.0,
            'filter_ground': False,
            'occupancy_min_z': 0.0,
            'occupancy_max_z': 4.5,
            'latch': False,
        }],
        remappings=[('cloud_in', '/marines_rov/point_cloud')],
    )

    return LaunchDescription([
        map_frame_arg,
        multibeam_static_tf,
        octomap_server_node,
    ])
