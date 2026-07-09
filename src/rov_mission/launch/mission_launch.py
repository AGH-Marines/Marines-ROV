from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rov_mission', executable='mission', name='mission_node', output='screen'),
    ])
