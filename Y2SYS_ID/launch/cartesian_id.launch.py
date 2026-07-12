from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("Y2SYS_ID")) / "config" / "cartesian_id.yaml")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Y2SYS_ID parameter file.",
        ),
        Node(
            package="Y2SYS_ID",
            executable="response_recorder_node",
            name="response_recorder",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="Y2SYS_ID",
            executable="online_metrics_node",
            name="online_metrics",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="Y2SYS_ID",
            executable="system_id_node",
            name="system_id",
            output="screen",
            parameters=[config_file],
            # This node does not control the robot. It only analyzes the measured
            # input/output response and exits when the experiment duration is over.
            # Its exit is used as the trigger to stop the whole Y2SYS_ID launch.
            on_exit=Shutdown(reason="Y2SYS_ID experiment finished"),
        ),
        Node(
            package="Y2SYS_ID",
            executable="cartesian_excitation_node",
            name="cartesian_excitation",
            output="screen",
            parameters=[config_file],
        ),
    ])
