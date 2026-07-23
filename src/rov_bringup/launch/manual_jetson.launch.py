import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Fallback teleop path for when the localization stack (ZED odometry/EKF)
    # is unavailable or misbehaving: joy -> rov_teleop -> manual_wrench_node
    # -> thruster_allocator. No localization_launch, no mapping_launch, no
    # pid_controller_node - nothing here touches TF, odometry, or 6-DOF
    # stabilization, so it keeps working even if that whole stack is down.
    # Run alongside controller.launch.py if you also want RViz/rqt/camera view.
    actuation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rov_actuation'), 'launch', 'actuation_launch.py')),
        launch_arguments={'use_sim': 'false'}.items(),
    )

    joy_node = Node(package='joy', executable='joy_node', name='joy_node', output='screen')
    teleop_node = Node(
        package='rov_control',
        executable='rov_teleop',
        name='rov_teleop_node',
        output='screen',
    )
    manual_wrench_node = Node(
        package='rov_control',
        executable='manual_wrench',
        name='manual_wrench_node',
        output='screen',
    )

    return LaunchDescription([
        actuation_launch,
        joy_node, teleop_node, manual_wrench_node,
    ])
