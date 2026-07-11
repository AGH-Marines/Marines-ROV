from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic_arg = DeclareLaunchArgument('topic', default_value='/thrusters/STONEFISH')
    thrust_value_arg = DeclareLaunchArgument('thrust_value', default_value='0.3')
    duration_arg = DeclareLaunchArgument('duration', default_value='3.0')
    robot_model_path_arg = DeclareLaunchArgument('robot_model_path', default_value='')

    return LaunchDescription([
        topic_arg, thrust_value_arg, duration_arg, robot_model_path_arg,
        Node(
            package='rov_actuation',
            executable='thruster_test',
            name='thruster_test_node',
            output='screen',
            parameters=[{
                'topic': ParameterValue(LaunchConfiguration('topic'), value_type=str),
                'thrust_value': ParameterValue(LaunchConfiguration('thrust_value'), value_type=float),
                'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
                'robot_model_path': ParameterValue(LaunchConfiguration('robot_model_path'), value_type=str),
            }],
        ),
    ])
