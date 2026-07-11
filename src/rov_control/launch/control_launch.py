from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    path_topic = DeclareLaunchArgument('path_topic', default_value='/marines_rov/global_path')
    publish_path_complete = DeclareLaunchArgument('publish_path_complete', default_value='true')
    reset_pid_on_new_path = DeclareLaunchArgument('reset_pid_on_new_path', default_value='true')

    return LaunchDescription([
        path_topic, publish_path_complete, reset_pid_on_new_path,
        Node(
            package='rov_control',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen',
            parameters=[{
                'path_topic': ParameterValue(LaunchConfiguration('path_topic'), value_type=str),
                'publish_path_complete': ParameterValue(
                    LaunchConfiguration('publish_path_complete'), value_type=bool),
                'reset_pid_on_new_path': ParameterValue(
                    LaunchConfiguration('reset_pid_on_new_path'), value_type=bool),
            }],
        ),
    ])
