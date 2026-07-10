import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')
    use_sim = LaunchConfiguration('use_sim')

    def include(pkg, launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
            **kwargs,
        )

    sim_launch = include('rov_simulation', 'sim_launch.py', condition=IfCondition(use_sim))
    localization_launch = include('rov_localization', 'localization_launch.py')
    mapping_launch = include('rov_mapping', 'mapping_launch.py')
    navigation_launch = include('rov_navigation', 'navigation_launch.py')
    control_launch = include('rov_control', 'control_launch.py')
    actuation_launch = include('rov_actuation', 'actuation_launch.py', launch_arguments={'use_sim': use_sim}.items())
    mission_launch = include('rov_mission', 'mission_launch.py')

    rviz_config = os.path.join(get_package_share_directory('rov_bringup'), 'rviz', 'default_view.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])

    urdf_path = os.path.join(get_package_share_directory('rov_bringup'), 'urdf', 'rov_visual.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    dvl_converter = Node(
        package='rov_bridge',
        executable='dvl_converter',
        name='dvl_converter_node'
    )

    imu_filter = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ROV/imu_filter']
    )

    # Stonefish nie publikuje TF dla żadnej z ramek sensorów (patrz komentarz w
    # rov_mapping/mapping_launch.py) - kamera potrzebuje własnej statycznej
    # transformacji, inaczej wyświetlacze RViz zależne od TF (np. Camera) będą
    # w nieskończoność czekać na transformację i zapychać kolejkę Message Filter.
    # Rotacja -90/0/-90 to konwersja "ramka fizyczna -> ramka optyczna" (Z=przód,
    # X=prawo, Y=dół - konwencja obrazów/kamer w ROS), taka sama jak dla multibeamu
    # w rov_mapping/mapping_launch.py.
    zed_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_camera_static_tf',
        arguments=[
            '--x', '0.35', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963267948966', '--pitch', '0.0', '--yaw', '-1.5707963267948966',
            '--frame-id', 'base_link',
            '--child-frame-id', 'ROV/zed_camera',
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        sim_launch, localization_launch, mapping_launch, navigation_launch,
        control_launch, actuation_launch, mission_launch, dvl_converter, imu_filter,
        zed_camera_tf, robot_state_publisher, rviz_node
    ])
