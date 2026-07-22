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
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode', default_value='hybrid',
        description="'direct' = RRT* only (as before); 'hybrid' = RRT* + reactive APF layer")
    include = _include
    localization_launch = include(
        'rov_localization', 'localization_launch.py',
        launch_arguments={'ekf_config': 'ekf_jetson.yaml'}.items(),
    )
    mapping_launch = include(
        'rov_mapping', 'mapping_launch.py',
        launch_arguments={'cloud_in': '/zed/zed_node/point_cloud/cloud_registered'}.items(),
    )
    navigation_launch = include('rov_navigation', 'navigation_launch.py')
    actuation_launch = include(
        'rov_actuation', 'actuation_launch.py',
        launch_arguments={'use_sim': 'false'}.items(),
    )
    mission_launch = include('rov_mission', 'mission_launch.py')
    zed_launch = include('zed_wrapper', 'zed_camera.launch.py', launch_arguments={
        'camera_model': 'zed2i',
        'publish_tf': 'true',  # Włączone, aby ZED sam zbudował drzewo do base_link
        'base_frame_id': 'base_link',  # Główna rama Twojego ROV-a
        'map_frame_id': 'world_ned',  # Układ globalny zgodny z Twoim PID
        'odom_frame_id': 'odom',
        'publish_odometry_tf': 'true',
        'set_as_static': 'false',
        'publish_imu_tf': 'true'
    }.items())

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

    # zed_wrapper hardcodes its own base frame to "<camera_name>_camera_link"
    # (zed_camera_component.cpp: mBaseFrameId = mCameraName + "_camera_link") -
    # there is no launch argument to rename it to "base_link". So it publishes
    # odom -> zed_camera_link on its own, disconnected from our base_link tree
    # (imu_static_tf/zed_camera_static_tf/multibeam_static_tf, all rooted at
    # base_link) unless we bridge the two with the inverse of the same mounting
    # offset used above for zed_camera_tf (translation/rotation inverted, see
    # scratchpad calc: R^T and -R^T*t).
    zed_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_base_link_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '-0.35',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'zed_camera_link',
            '--child-frame-id', 'base_link',
        ],
    )

    return LaunchDescription([
        nav_mode_arg,
        localization_launch, mapping_launch, navigation_launch,
        actuation_launch, mission_launch, dvl_converter, imu_filter,
        zed_camera_tf, zed_base_link_tf, zed_launch, robot_state_publisher,
        OpaqueFunction(function=_setup_navigation),
    ])
