import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')
    enable_mapping_arg = DeclareLaunchArgument('enable_mapping', default_value='true')
    use_sim = LaunchConfiguration('use_sim')
    enable_mapping = LaunchConfiguration('enable_mapping')

    def include(pkg, launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
            **kwargs,
        )

    sim_launch = include('rov_simulation', 'sim_launch.py', condition=IfCondition(use_sim))
    localization_launch = include('rov_localization', 'localization_launch.py')
    mapping_launch = include('rov_mapping', 'mapping_launch.py', condition=IfCondition(enable_mapping))
    control_launch = include('rov_control', 'control_launch.py')
    actuation_launch = include('rov_actuation', 'actuation_launch.py', launch_arguments={'use_sim': use_sim}.items())

    joy_node = Node(package='joy', executable='joy_node', name='joy_node', output='screen')
    teleop_node = Node(
        package='rov_control',
        executable='rov_teleop',
        name='rov_teleop_node',  # Zmieniona nazwa dla porządku
        output='screen'
    )

    rviz_config = os.path.join(get_package_share_directory('rov_bringup'), 'rviz', 'default_view.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])

    imu_filter = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ROV/imu_filter']
    )

    return LaunchDescription([
        use_sim_arg, enable_mapping_arg,
        sim_launch, localization_launch, mapping_launch,
        control_launch, actuation_launch,
        joy_node, teleop_node,
        rviz_node, imu_filter,
    ])
