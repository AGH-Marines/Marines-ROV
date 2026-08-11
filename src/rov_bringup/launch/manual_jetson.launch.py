import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Fallback path for when the localization stack (ZED odometry/EKF) is
    # unavailable or misbehaving: manual_wrench_node -> thruster_allocator,
    # with nothing touching TF, odometry, or 6-DOF stabilization, so it keeps
    # working even if that whole stack is down. Runs ON THE JETSON, same as
    # teleop_jetson.launch.py. The joystick is on the topside/operator
    # computer, not the ROV - run topside.launch.py there (joy_node +
    # rov_teleop, plus GUIs) to feed /marines_rov/cmd_vel over the ROS2/DDS
    # network.
    actuation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rov_actuation'), 'launch', 'actuation_launch.py')),
        launch_arguments={'use_sim': 'false'}.items(),
    )

    manual_wrench_node = Node(
        package='rov_control',
        executable='manual_wrench',
        name='manual_wrench_node',
        output='screen',
    )

    return LaunchDescription([
        actuation_launch,
        manual_wrench_node,
    ])
