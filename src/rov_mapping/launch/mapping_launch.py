from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='odom')
    map_frame = LaunchConfiguration('map_frame')

    cloud_in_arg = DeclareLaunchArgument(
        'cloud_in', default_value='/marines_rov/point_cloud',
        description='PointCloud2 source for octomap_server, e.g. '
                     '/marines_rov/point_cloud (simulated multibeam) or '
                     '/zed/zed_node/point_cloud/cloud_registered (real ZED)')
    cloud_in = LaunchConfiguration('cloud_in')

    multibeam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='multibeam_static_tf',
        arguments=[
            '--x', '0.35', '--y', '0.0', '--z', '0.0',
            '--roll', '1.5707963267948966', '--pitch', '0.0', '--yaw', '1.5707963267948966',
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
            'occupancy_min_z': -50.0,
            'occupancy_max_z': 1.0,
            'latch': False,
        }],
        remappings=[('cloud_in', cloud_in)],
    )

    return LaunchDescription([
        map_frame_arg, cloud_in_arg,
        multibeam_static_tf,
        octomap_server_node,
    ])
