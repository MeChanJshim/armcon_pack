from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    joint_command_topic = LaunchConfiguration('joint_command_topic')
    force_data_topic = LaunchConfiguration('force_data_topic')

    singleArm_node = Node(
        package='Y2RobMotion',
        executable='singleArm_motion',
        name='singleArm_motion',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/joint_states', joint_state_topic),
            ('/forward_position_controller/commands', joint_command_topic),
            ('/ur10skku/ftdata', force_data_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/armsimul/joint_states',
            description='MuJoCo JointState topic used by singleArm_motion.',
        ),
        DeclareLaunchArgument(
            'joint_command_topic',
            default_value='/armsimul/joint_position_cmd_array',
            description='MuJoCo Float64MultiArray joint position command topic.',
        ),
        DeclareLaunchArgument(
            'force_data_topic',
            default_value='/armsimul/ee_wrench',
            description='MuJoCo WrenchStamped end-effector force topic.',
        ),
        singleArm_node,
    ])
