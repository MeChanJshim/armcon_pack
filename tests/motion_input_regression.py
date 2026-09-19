"""Installed-node regression. Source workspace; use ROS_DOMAIN_ID=217."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import WrenchStamped
import math

assert os.environ.get('ROS_DOMAIN_ID') == '217'
rclpy.init()
node = rclpy.create_node('motion_input_regression')
pub = node.create_publisher(JointState, '/audit/joints', 10)
mode = node.create_publisher(String, '/ur10skku/cmdMode', 10)
motion = node.create_publisher(Float64MultiArray, '/ur10skku/cmdMotion', 10)
force = node.create_publisher(WrenchStamped, '/ur10skku/ftdata', 10)
commands = []
sub = node.create_subscription(Float64MultiArray, '/audit/commands', lambda m: commands.append(list(m.data)), 10)
binary = Path(get_package_prefix('Y2RobMotion'))/'lib/Y2RobMotion/singleArm_motion'
with open('/tmp/motion_input_regression.log', 'w') as log:
    proc = subprocess.Popen([str(binary), '--ros-args', '-r', '/joint_states:=/audit/joints',
        '-r', '/forward_position_controller/commands:=/audit/commands'], stdout=log, stderr=log)
    def pump(message, seconds=.4):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
    try:
        end=time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.02)
        pump(JointState())
        assert not commands, 'Empty joint state started command output'
        names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        pump(JointState(name=['wrong']*6, position=[0.]*6))
        pump(JointState(name=names, position=[float('nan')]*6))
        assert not commands, 'Invalid joints started command output'
        values=[.1,-1.,-1.,-1.,1.,.2]
        pump(JointState(name=names, position=values))
        assert commands and commands[-1]==values, commands[-1:]
        pump(JointState(name=names[::-1], position=values[::-1]))
        assert commands[-1]==values, 'Reordered joints were misinterpreted'
        end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.01)
        count=len(commands)
        end=time.monotonic()+.15
        while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.01)
        assert len(commands)==count, 'Commands continue after joint timeout, Force mode and force timeout'
        # Resume with valid feedback and exercise the installed force controller.
        state=JointState(name=names, position=values)
        pump(state)
        mode.publish(String(data='Force'))
        wrench=WrenchStamped();wrench.wrench.force.z=8.
        end=time.monotonic()+.5
        while time.monotonic()<end:
            pub.publish(state);force.publish(wrench)
            motion.publish(Float64MultiArray(data=[-500.,-300.,300.,0.,0.,0.,0.,0.,8.]))
            rclpy.spin_once(node,timeout_sec=.01)
        assert commands and all(math.isfinite(v) for v in commands[-1])
        assert commands[-1] != values, 'Force mode did not execute'
        # Keep joints fresh but stop the force sensor: hold measured joints.
        pump(state, .5)
        assert commands[-1]==values, 'Expired force feedback was still driving motion'
        assert proc.poll() is None
        print('PASS: invalid startup, finite valid commands, reordered joints, joint timeout, Force mode and force timeout')
    finally:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
