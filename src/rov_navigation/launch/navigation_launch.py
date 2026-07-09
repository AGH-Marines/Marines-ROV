from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rov_navigation', executable='planner', name='planner_node', output='screen'),
    ])
