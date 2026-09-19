"""Headless installed-node graph check. Run with ROS_DOMAIN_ID=214."""
import os
import signal
import subprocess
import time
import math
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

assert os.environ.get('ROS_DOMAIN_ID') == '214'
rclpy.init()
node = rclpy.create_node('contact_graph_smoke')
seen = {}
def record(name):
    def cb(msg): seen[name] = msg
    return cb
subs = [node.create_subscription(t, topic, record(name), 10) for t, topic, name in [
    (JointState, '/armsimul/joint_states', 'joint'),
    (WrenchStamped, '/armsimul/ft_sensor_wrench', 'wrench'),
    (PointStamped, '/contactsensing/cp', 'cp'),
    (Float64MultiArray, '/ur10skku/currentP', 'pose')]]
def exe(pkg, name): return str(Path(get_package_prefix(pkg))/'lib'/pkg/name)
mesh = Path(__file__).resolve().parents[2]/'NRS_intrinsic_contact_sensing/Extracted_surface/NRS_spindle_surface.stl'
commands = [
    [exe('mujoco_simulpack', 'ur10_contact_sim'), '--ros-args', '-p', 'use_viewer:=false',
     '-p', 'joint_state_topic:=/armsimul/joint_states'],
    [exe('Y2RobMotion', 'singleArm_motion'), '--ros-args', '-r', '/joint_states:=/armsimul/joint_states',
     '-r', '/forward_position_controller/commands:=/armsimul/joint_position_cmd_array',
     '-r', '/ur10skku/ftdata:=/armsimul/ee_wrench'],
    [exe('nrs_ifs', 'main_ifs'), '--ros-args', '-p', 'mesh_directory:='+str(mesh),
     '-p', 'mesh_transformation:=[1.0,0.0,0.0,0.06,0.0,1.0,0.0,0.06,0.0,0.0,1.0,0.0]',
     '-p', 'contact_sensor_topic:=/armsimul/ft_sensor_wrench']]
procs = []
with open('/tmp/contact_graph_smoke.log', 'w') as log:
    try:
        for cmd in commands: procs.append(subprocess.Popen(cmd, stdout=log, stderr=log))
        end = time.monotonic()+6
        while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.01)
        assert all(p.poll() is None for p in procs), 'Node exited'
        assert set(seen)=={'joint', 'wrench', 'cp', 'pose'}, set(seen)
        assert all(math.isfinite(v) for v in seen['joint'].position+seen['pose'].data)
        assert seen['cp'].header.frame_id==seen['wrench'].header.frame_id
        print('PASS: installed simulator/controller/NRS graph, finite joint/pose feedback, matching sensor frames')
    finally:
        for p in procs:
            if p.poll() is None: p.send_signal(signal.SIGINT)
        for p in procs:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill(); p.wait()
        node.destroy_node()
        rclpy.shutdown()
