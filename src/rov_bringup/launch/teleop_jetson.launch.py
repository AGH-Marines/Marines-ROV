import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        **kwargs,
    )


def generate_launch_description():
    include = _include

    # Real-hardware wiring (ZED odometry/depth feeding the EKF and octomap)
    # plus the PID controller - everything that needs to run ON THE JETSON
    # for stabilized teleop. Bundled into one file (instead of a separate
    # controller.launch.py) since there's no reason to launch two files on
    # the same machine. The joystick is plugged into the topside/operator
    # computer, not the ROV, so joy_node/rov_teleop live in
    # topside.launch.py instead and reach this machine over the ROS2/DDS
    # network, same for RViz/rqt/camera view.
    path_topic_arg = DeclareLaunchArgument('path_topic', default_value='/marines_rov/global_path')
    publish_path_complete_arg = DeclareLaunchArgument('publish_path_complete', default_value='true')
    reset_pid_on_new_path_arg = DeclareLaunchArgument('reset_pid_on_new_path', default_value='true')

    control_launch = include(
        'rov_control', 'control_launch.py',
        launch_arguments={
            'path_topic': LaunchConfiguration('path_topic'),
            'publish_path_complete': LaunchConfiguration('publish_path_complete'),
            'reset_pid_on_new_path': LaunchConfiguration('reset_pid_on_new_path'),
        }.items(),
    )

    localization_launch = include(
        'rov_localization', 'localization_launch.py',
        launch_arguments={'ekf_config': 'ekf_jetson.yaml'}.items(),
    )
    mapping_launch = include(
        'rov_mapping', 'mapping_launch.py',
        launch_arguments={'cloud_in': '/zed/zed_node/point_cloud/cloud_registered'}.items(),
    )
    actuation_launch = include(
        'rov_actuation', 'actuation_launch.py',
        launch_arguments={'use_sim': 'false'}.items(),
    )
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
    # (imu_static_tf/zed_camera_static_tf, both rooted at base_link) unless we
    # bridge the two. zed_camera_link already uses the same X-fwd/Y-left/Z-up
    # convention as base_link (ROS_COORDINATE_SYSTEM = RIGHT_HANDED_Z_UP_X_FWD
    # in zed_camera_component.cpp), and the camera is mounted facing straight
    # forward (see rov_visual.urdf: camera box at xyz="0.35 0 0" rpy="0 0 0"),
    # so this bridge is a pure translation - no rotation needed.
    zed_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_base_link_tf',
        arguments=[
            '--x', '-0.35', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'zed_camera_link',
            '--child-frame-id', 'base_link',
        ],
    )

    return LaunchDescription([
        path_topic_arg, publish_path_complete_arg, reset_pid_on_new_path_arg,
        control_launch,
        localization_launch, mapping_launch, actuation_launch,
        dvl_converter, imu_filter,
        zed_camera_tf, zed_base_link_tf, zed_launch, robot_state_publisher,
    ])
