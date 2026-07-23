import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Runs ON THE TOPSIDE/OPERATOR COMPUTER, not the Jetson - the joystick is
    # plugged in here and these are GUIs for the human operator (the Jetson
    # is normally headless anyway). Everything here just talks over the
    # ROS2/DDS network to whatever is running onboard - teleop_jetson.launch.py
    # (sensors + PID, stabilized) or manual_jetson.launch.py (raw, no
    # stabilization). Both machines need the same ROS_DOMAIN_ID/network.
    joy_node = Node(package='joy', executable='joy_node', name='joy_node', output='screen')
    teleop_node = Node(
        package='rov_control',
        executable='rov_teleop',
        name='rov_teleop_node',
        output='screen',
    )

    rviz_config = os.path.join(get_package_share_directory('rov_bringup'), 'rviz', 'default_view.rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])

    rqt_node = Node(package='rqt_gui', executable='rqt_gui', name='rqt_gui')

    zed_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/zed/zed_node/rgb/image_rect_color'],
    )

    return LaunchDescription([joy_node, teleop_node, rviz_node, rqt_node, zed_image_view])
