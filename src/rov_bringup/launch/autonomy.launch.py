import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _include(pkg, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        **kwargs,
    )


def _setup_navigation(context, *args, **kwargs):
    hybrid = LaunchConfiguration('nav_mode').perform(context) == 'hybrid'

    control_launch = _include(
        'rov_control', 'control_launch.py',
        launch_arguments={
            'path_topic': '/marines_rov/local_path' if hybrid else '/marines_rov/global_path',
            'publish_path_complete': 'false' if hybrid else 'true',
            'reset_pid_on_new_path': 'false' if hybrid else 'true',
        }.items(),
    )

    nodes = [control_launch]
    if hybrid:
        nodes.append(Node(
            package='rov_navigation',
            executable='local_planner',
            name='local_planner_node',
            output='screen',
        ))
    return nodes


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')
    use_sim = LaunchConfiguration('use_sim')
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode', default_value='hybrid',
        description="'direct' = RRT* only (as before); 'hybrid' = RRT* + reactive APF layer")

    include = _include

    sim_launch = include('rov_simulation', 'sim_launch.py', condition=IfCondition(use_sim))
    localization_launch = include('rov_localization', 'localization_launch.py')
    mapping_launch = include('rov_mapping', 'mapping_launch.py')
    navigation_launch = include('rov_navigation', 'navigation_launch.py')
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
        use_sim_arg, nav_mode_arg,
        sim_launch, localization_launch, mapping_launch, navigation_launch,
        actuation_launch, mission_launch, dvl_converter, imu_filter,
        zed_camera_tf, robot_state_publisher, rviz_node,
        OpaqueFunction(function=_setup_navigation),
    ])
