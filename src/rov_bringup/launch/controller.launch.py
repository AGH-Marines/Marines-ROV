import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Mirrors rov_control/control_launch.py's own arguments so callers of this
    # file can still pick direct vs. hybrid path following without reaching into
    # the included launch file.
    path_topic_arg = DeclareLaunchArgument('path_topic', default_value='/marines_rov/global_path')
    publish_path_complete_arg = DeclareLaunchArgument('publish_path_complete', default_value='true')
    reset_pid_on_new_path_arg = DeclareLaunchArgument('reset_pid_on_new_path', default_value='true')

    # Split out of teleop_jetson.launch.py / autonomy_jetson.launch.py so the PID
    # controller and the GUIs can be started/stopped independently of the
    # sensor/teleop stack (e.g. bring up teleop_jetson once, then attach/detach
    # the controller and RViz/rqt as needed without restarting the ZED/EKF/joy nodes).
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rov_control'), 'launch', 'control_launch.py')),
        launch_arguments={
            'path_topic': LaunchConfiguration('path_topic'),
            'publish_path_complete': LaunchConfiguration('publish_path_complete'),
            'reset_pid_on_new_path': LaunchConfiguration('reset_pid_on_new_path'),
        }.items(),
    )

    rviz_config = os.path.join(get_package_share_directory('rov_bringup'), 'rviz', 'default_view.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])

    rqt_node = Node(package='rqt_gui', executable='rqt_gui', name='rqt_gui')

    return LaunchDescription([
        path_topic_arg, publish_path_complete_arg, reset_pid_on_new_path_arg,
        control_launch, rviz_node, rqt_node,
    ])
