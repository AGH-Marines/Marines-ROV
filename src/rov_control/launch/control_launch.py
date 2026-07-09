from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov_control',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen'
        ),
    ])
