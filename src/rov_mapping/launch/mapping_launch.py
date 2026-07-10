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
    # Translacja skopiowana z underwater_world.scn (sensor współdzieli miejsce montażu z zed_camera).
    # Rotacja -90/0/-90 to standardowa konwersja "ramka fizyczna -> ramka optyczna": Multibeam2
    # w ROS2Interface::PublishMultibeam2 pakuje punkty z lokalnym Z jako kierunkiem patrzenia
    # (X/Y to tylko małe odchylenia kątowe), czyli dokładnie tak jak dane z kamery/RealSense/ZED,
    # a nie w standardowej konwencji ROS (X = przód). Bez tej korekty octomap wstawiał punkty
    # wzdłuż osi Z base_linka (w pionie) zamiast wzdłuż X (do przodu).
    multibeam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='multibeam_static_tf',
        arguments=[
            '--x', '0.35', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963267948966', '--pitch', '0.0', '--yaw', '-1.5707963267948966',
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
            # ROV operuje pod powierzchnią wody, czyli w standardowej (Z w górę) konwencji
            # ROS-a cała mapa ma ujemne Z (powierzchnia ~= 0). Domyślne 0.0/4.5 to wartości
            # dla robota lądowego - obcinały mapowanie tuż pod startową głębokością.
            'occupancy_min_z': -50.0,
            'occupancy_max_z': 1.0,
            'latch': False,
        }],
        remappings=[('cloud_in', '/marines_rov/point_cloud')],
    )

    return LaunchDescription([
        map_frame_arg,
        multibeam_static_tf,
        octomap_server_node,
    ])
