from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Y2JoyGUI',
            executable='y2_joy_gui_node',
            name='y2_joy_gui_node',
            output='screen',
            parameters=[{
                'robot_name': 'ur10skku',
                'host': '0.0.0.0',
                'port': 8080,
                'joy_axis_count': 8,
            }],
        )
    ])
