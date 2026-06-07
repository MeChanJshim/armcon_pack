from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('Y2_Joystick'),
        'config',
        'y2_joystick.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='Y2_Joystick',
            executable='y2_joystick_node',
            name='y2_joystick_node',
            output='screen',
            parameters=[config_file],
        )
    ])
