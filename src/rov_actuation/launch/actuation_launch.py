from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='true: publish to /thrusters/STONEFISH; false: publish to thrusters/PWM for rov_bridge')
    use_sim = LaunchConfiguration('use_sim')

    allocator_sim = Node(
        package='rov_actuation', executable='thruster_allocator', name='thruster_allocator_node',
        output='screen', condition=IfCondition(use_sim),
    )
    allocator_hw = Node(
        package='rov_actuation', executable='thruster_allocator', name='thruster_allocator_node',
        output='screen', condition=UnlessCondition(use_sim),
        remappings=[('/thrusters/STONEFISH', 'thrusters/PWM')],
    )

    return LaunchDescription([use_sim_arg, allocator_sim, allocator_hw])
