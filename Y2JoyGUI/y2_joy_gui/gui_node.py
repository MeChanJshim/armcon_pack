import json
import math
import os
import re
import signal
import shlex
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs
from urllib.parse import urlparse

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def _rotation_vector_to_quaternion(vector):
    angle = math.sqrt(sum(value * value for value in vector))
    if angle < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    scale = math.sin(angle * 0.5) / angle
    return [math.cos(angle * 0.5), *(value * scale for value in vector)]


def _quaternion_to_rotation_vector(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError("invalid zero quaternion in trajectory")
    w, x, y, z = (value / norm for value in quaternion)
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    angle = 2.0 * math.acos(min(1.0, abs(w)))
    sin_half = math.sin(angle * 0.5)
    if angle < 1e-6 or abs(sin_half) < 1e-10:
        return [0.0, 0.0, 0.0]
    return [component * angle / sin_half for component in (x, y, z)]


def _slerp(start, end, amount):
    dot = sum(a * b for a, b in zip(start, end))
    if dot < 0:
        end = [-value for value in end]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = [(1.0 - amount) * a + amount * b for a, b in zip(start, end)]
    else:
        omega = math.acos(abs(dot))
        sine = math.sin(omega)
        left = math.sin((1.0 - amount) * omega) / sine
        right = math.sin(amount * omega) / sine
        result = [left * a + right * b for a, b in zip(start, end)]
    norm = math.sqrt(sum(value * value for value in result))
    return [value / norm for value in result]


def _profile_column(values, period, starting_time, resting_time, acceleration_time):
    target_velocity = [0.0] * len(values)
    for index in range(1, len(values)):
        target_velocity[index] = (values[index] - values[index - 1]) / period

    total_time = starting_time + period * len(values) + resting_time
    sample_count = int(math.floor(total_time / period + 1e-8)) + 1
    profiled_velocity = []
    last_flag = 0
    for index in range(sample_count):
        sample_time = index * period
        if sample_time <= starting_time:
            profiled_velocity.append(0.0)
            last_flag += 1
        elif sample_time <= starting_time + period * len(values):
            source_index = index - last_flag
            profiled_velocity.append(
                target_velocity[source_index] if source_index < len(target_velocity) else 0.0
            )
        else:
            profiled_velocity.append(0.0)

    window = acceleration_time / period
    positions = [values[0]]
    final_velocity = 0.0
    for index in range(1, sample_count):
        if index <= window:
            final_velocity += (profiled_velocity[index] - profiled_velocity[0]) / index
        else:
            lag = max(1, int(window))
            final_velocity += (profiled_velocity[index] - profiled_velocity[index - lag]) / window
        positions.append(positions[-1] + final_velocity * period)
    return positions


def _blend_motion(waypoints, linear_velocity, angular_velocity, holding_time,
                  period, travel_time=5.0, starting_time=2.0,
                  resting_time=2.0, acceleration_time=1.0,
                  angular_velocity_limit=math.radians(5.0)):
    if len(waypoints) < 2:
        raise ValueError("motion path needs at least two poses")
    if not math.isfinite(period) or period <= 0:
        raise ValueError("Control period must be finite and greater than zero")
    # Bound intermediate lists before allocating trajectory samples.
    max_samples = 200_000
    is_force = len(waypoints[0]) == 9
    interpolated = []
    for index in range(1, len(waypoints)):
        start = waypoints[index - 1]
        end = waypoints[index]
        start_quat = _rotation_vector_to_quaternion(start[3:6])
        end_quat = _rotation_vector_to_quaternion(end[3:6])
        if index == 1:
            first = list(start[:3]) + start_quat
            if is_force:
                first += list(end[6:9])
            interpolated.append(first)

        distance = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
        dot = abs(sum(a * b for a, b in zip(start_quat, end_quat)))
        angle = 2.0 * math.acos(min(1.0, dot))
        lin_time = distance / linear_velocity[index] if linear_velocity[index] else 0.0
        ang_time = angle / angular_velocity[index] if angular_velocity[index] else 0.0
        segment_time = max(lin_time, ang_time)
        if segment_time:
            segment_time = max(segment_time, angle / angular_velocity_limit)
        elif angle:
            segment_time = angle / angular_velocity_limit
        if holding_time[index]:
            start = end
            start_quat = end_quat
            segment_time = holding_time[index]
        if not linear_velocity[index] and not angular_velocity[index] and not holding_time[index]:
            segment_time = travel_time

        sample_estimate = (segment_time + starting_time + resting_time) / period
        if not math.isfinite(sample_estimate) or sample_estimate + len(interpolated) > max_samples:
            raise ValueError(
                f"Trajectory too long (maximum {max_samples * period:.0f} s including padding). "
                "Check target distance and velocity in mm/s; use a shorter move or a higher velocity."
            )
        steps = int(segment_time / period)
        for sample in range(steps):
            amount = sample / (steps - 1) if steps > 1 else 0.0
            xyz = [start[i] + (end[i] - start[i]) * amount for i in range(3)]
            quat = start_quat if steps == 1 else _slerp(start_quat, end_quat, amount)
            row = xyz + quat
            if is_force:
                row += list(end[6:9])
            interpolated.append(row)

    if not interpolated:
        raise ValueError("motion path generated no samples")
    profiled = [
        _profile_column([row[column] for row in interpolated], period,
                        starting_time, resting_time, acceleration_time)
        for column in range(len(interpolated[0]))
    ]
    output = []
    for index in range(len(profiled[0])):
        quat = [profiled[column][index] for column in range(3, 7)]
        norm = math.sqrt(sum(value * value for value in quat))
        quat = [value / norm for value in quat]
        row = [profiled[column][index] for column in range(3)]
        row += _quaternion_to_rotation_vector(quat)
        if is_force:
            row += [profiled[column][index] for column in range(7, 10)]
        output.append(row)
    return output


class Y2JoyGuiNode(Node):
    def __init__(self):
        super().__init__("y2_joy_gui_node")

        self.robot_name = self.declare_parameter("robot_name", "ur10skku").value
        self.host = self.declare_parameter("host", "0.0.0.0").value
        self.port = int(self.declare_parameter("port", 8080).value)
        self.joy_axis_count = int(self.declare_parameter("joy_axis_count", 8).value)
        self.control_period = float(self.declare_parameter("control_period", 0.0).value)
        self.trajectory_mode = int(self.declare_parameter("trajectory_mode", -1).value)

        base_topic = "/" + self.robot_name.strip("/")
        self.mode_pub = self.create_publisher(String, base_topic + "/cmdMode", 10)
        self.cmd_motion_pub = self.create_publisher(
            Float64MultiArray, base_topic + "/cmdMotion", 10)
        self.joy_move_pub = self.create_publisher(
            Float64MultiArray, base_topic + "/joy_move", 10)
        self.record_stop_pub = self.create_publisher(
            Bool, base_topic + "/record_boolean_L", 10)
        self.record_start_pub = self.create_publisher(
            Bool, base_topic + "/record_boolean_R", 10)
        self.current_mode = "Unknown"
        self.current_mode_sub = self.create_subscription(
            String, base_topic + "/cmdMode", self.on_current_mode, 10)
        self.current_position = None
        self.monitor_lock = threading.Lock()
        self.monitor_data = {
            "currentP": [None] * 6,
            "targetP": [None] * 6,
            "currentF": [None] * 6,
            "targetF": [None] * 6,
            "baseF": [None] * 6,
            "base_source": "waiting",
        }
        self.pose_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.motion_busy = False
        self.motion_stop_event = None
        self.motion_pause_event = None
        self.robot_command_state = "Waiting for current position"
        self.current_pose_sub = self.create_subscription(
            Float64MultiArray, base_topic + "/currentP", self.on_current_pose, 10)
        self.create_subscription(
            Float64MultiArray, base_topic + "/targetP", self.on_target_pose, 10)
        self.create_subscription(
            Float64MultiArray, base_topic + "/currentF", self.on_current_force, 10)
        self.create_subscription(
            Float64MultiArray, base_topic + "/targetF", self.on_target_force, 10)
        self.create_subscription(
            WrenchStamped, base_topic + "/ftdata", self.on_base_wrench, 10)
        self.create_subscription(
            WrenchStamped, "/armsimul/ee_wrench", self.on_sim_wrench, 10)

        self.allowed_modes = {"Idling", "Guiding", "Joystick", "Joystick_force"}
        self.web_dir = Path(get_package_share_directory("Y2JoyGUI")) / "web"
        self.workspace_setup = Path(get_package_prefix("Y2JoyGUI")).parent / "setup.bash"
        self.path_files = self.find_path_files()
        self.measured_files = self.find_measured_files()
        self.server = None
        self.server_thread = None
        self.process_lock = threading.Lock()
        self.managed_processes = self.build_process_specs()

    def start_server(self):
        node = self
        web_dir = self.web_dir

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(web_dir), **kwargs)

            def log_message(self, format_text, *args):
                node.get_logger().debug(format_text % args)

            def end_headers(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                super().end_headers()

            def do_OPTIONS(self):
                self.send_response(204)
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)

                if parsed.path == "/api/status":
                    self.write_json({
                        "ok": True,
                        "robot_name": node.robot_name,
                        "topics": {
                            "mode": f"/{node.robot_name}/cmdMode",
                            "current_mode": node.current_mode,
                            "motion": f"/{node.robot_name}/cmdMotion",
                            "command_state": node.robot_command_state,
                            "joy_move": f"/{node.robot_name}/joy_move",
                            "record_start": f"/{node.robot_name}/record_boolean_R",
                            "record_stop": f"/{node.robot_name}/record_boolean_L",
                        },
                    })
                    return

                if parsed.path == "/api/monitor":
                    with node.monitor_lock:
                        self.write_json({"ok": True, **node.monitor_data})
                    return

                if parsed.path == "/api/processes":
                    self.write_json({
                        "ok": True,
                        "processes": node.process_status(),
                    })
                    return

                if parsed.path == "/api/path/read":
                    query = parse_qs(parsed.query)
                    path_type = query.get("type", [""])[0]
                    self.write_json({
                        "ok": True,
                        "path": node.read_path_file(path_type),
                    })
                    return

                if parsed.path == "/api/measured/list":
                    self.write_json({
                        "ok": True,
                        "files": node.measured_file_status(),
                    })
                    return

                if parsed.path == "/api/measured/read":
                    query = parse_qs(parsed.query)
                    file_id = query.get("file", [""])[0]
                    self.write_json({
                        "ok": True,
                        "measured": node.read_measured_file(file_id),
                    })
                    return

                if parsed.path == "/":
                    self.path = "/index.html"
                super().do_GET()

            def do_POST(self):
                try:
                    payload = self.read_json()
                    if self.path == "/api/mode":
                        mode = str(payload.get("mode", ""))
                        node.publish_mode(mode)
                        self.write_json({"ok": True, "mode": mode})
                        return

                    if self.path == "/api/record":
                        action = str(payload.get("action", ""))
                        node.publish_record(action)
                        self.write_json({"ok": True, "action": action})
                        return

                    if self.path == "/api/joy_move":
                        axes = payload.get("axes", [])
                        node.publish_joy_move(axes)
                        self.write_json({"ok": True, "axes": axes})
                        return

                    if self.path == "/api/ptp":
                        target = payload.get("target", [])
                        velocity = payload.get("velocity", 0.5)
                        node.start_ptp(target, velocity)
                        self.write_json({"ok": True, "target": target, "velocity": velocity})
                        return

                    if self.path == "/api/txt_load":
                        node.start_txt_load()
                        self.write_json({"ok": True, "action": "txt_load"})
                        return

                    if self.path == "/api/motion/stop":
                        node.stop_motion()
                        self.write_json({"ok": True, "action": "stop"})
                        return

                    if self.path == "/api/motion/pause":
                        node.pause_motion()
                        self.write_json({"ok": True, "action": "pause"})
                        return

                    if self.path == "/api/motion/resume":
                        node.resume_motion()
                        self.write_json({"ok": True, "action": "resume"})
                        return

                    if self.path == "/api/process/start":
                        process_id = str(payload.get("id", ""))
                        options = payload.get("options", {})
                        self.write_json({
                            "ok": True,
                            "process": node.start_process(process_id, options),
                        })
                        return

                    if self.path == "/api/process/stop":
                        process_id = str(payload.get("id", ""))
                        self.write_json({
                            "ok": True,
                            "process": node.stop_process(process_id),
                        })
                        return

                    if self.path == "/api/path/write":
                        path_type = str(payload.get("type", ""))
                        rows = payload.get("rows", [])
                        self.write_json({
                            "ok": True,
                            "path": node.write_path_file(path_type, rows),
                        })
                        return

                    self.write_json({"ok": False, "error": "unknown endpoint"}, 404)
                except ValueError as exc:
                    self.write_json({"ok": False, "error": str(exc)}, 400)
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().error(f"GUI request failed: {exc}")
                    self.write_json({"ok": False, "error": "internal server error"}, 500)

            def read_json(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b"{}"
                return json.loads(raw.decode("utf-8"))

            def write_json(self, payload, status=200):
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self.server = ReusableThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            if exc.errno == 98:
                self.get_logger().error(
                    f"Port {self.port} is already in use. Y2JoyGUI may already be running.")
                self.get_logger().error(
                    f"Command page: http://127.0.0.1:{self.port}")
                self.get_logger().error(
                    f"Node runner: http://127.0.0.1:{self.port}/nodes.html")
                return False
            raise

        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            name="y2_joy_gui_http",
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            f"Y2JoyGUI command page: http://127.0.0.1:{self.port} "
            f"(serving {self.web_dir})")
        self.get_logger().info(
            f"Y2JoyGUI node runner: http://127.0.0.1:{self.port}/nodes.html")
        self.get_logger().info(
            f"Y2JoyGUI path editor: http://127.0.0.1:{self.port}/path.html")
        self.get_logger().info(
            f"Y2JoyGUI measured viewer: http://127.0.0.1:{self.port}/measured.html")
        return True

    def publish_mode(self, mode):
        if mode not in self.allowed_modes:
            raise ValueError(f"unsupported mode: {mode}")
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.current_mode = mode
        self.get_logger().info(f"Published mode: {mode}")

    def on_current_mode(self, msg):
        mode = str(msg.data).strip()
        if mode:
            self.current_mode = mode

    def publish_record(self, action):
        msg = Bool()
        msg.data = True
        if action == "start":
            self.record_start_pub.publish(msg)
            self.get_logger().info("Published record start")
            return
        if action == "stop":
            self.record_stop_pub.publish(msg)
            self.get_logger().info("Published record stop")
            return
        raise ValueError(f"unsupported record action: {action}")

    def publish_joy_move(self, axes):
        if not isinstance(axes, list):
            raise ValueError("axes must be a list")

        values = []
        for index in range(self.joy_axis_count):
            value = axes[index] if index < len(axes) else 0.0
            values.append(float(value))

        msg = Float64MultiArray()
        msg.data = values
        self.joy_move_pub.publish(msg)

    def on_current_pose(self, msg):
        if len(msg.data) < 6 or not all(math.isfinite(value) for value in msg.data[:6]):
            return
        with self.pose_lock:
            self.current_position = list(msg.data[:6])
        with self.monitor_lock:
            self.monitor_data["currentP"] = list(msg.data[:6])
        if self.robot_command_state == "Waiting for current position":
            self.robot_command_state = "Robot pose received"

    def on_target_pose(self, msg):
        if len(msg.data) >= 6:
            with self.monitor_lock:
                self.monitor_data["targetP"] = list(msg.data[:6])

    def on_current_force(self, msg):
        if len(msg.data) >= 6:
            with self.monitor_lock:
                self.monitor_data["currentF"] = list(msg.data[:6])

    def on_target_force(self, msg):
        if len(msg.data) >= 6:
            with self.monitor_lock:
                self.monitor_data["targetF"] = list(msg.data[:6])

    def on_base_wrench(self, msg):
        wrench = msg.wrench
        values = [wrench.force.x, wrench.force.y, wrench.force.z,
                  wrench.torque.x, wrench.torque.y, wrench.torque.z]
        with self.monitor_lock:
            self.monitor_data["baseF"] = values
            self.monitor_data["base_source"] = f"{msg.header.frame_id or 'ur10skku/ftdata'}"

    def on_sim_wrench(self, msg):
        with self.monitor_lock:
            if self.monitor_data["baseF"] == [None] * 6:
                wrench = msg.wrench
                self.monitor_data["baseF"] = [
                    wrench.force.x, wrench.force.y, wrench.force.z,
                    wrench.torque.x, wrench.torque.y, wrench.torque.z,
                ]
                self.monitor_data["base_source"] = f"{msg.header.frame_id or 'armsimul/ee_wrench'} (fallback)"

    def _begin_motion(self, label):
        if self.cmd_motion_pub.get_subscription_count() == 0:
            raise ValueError("No robot motion node is subscribed to cmdMotion")
        with self.pose_lock:
            current = None if self.current_position is None else list(self.current_position)
        if current is None:
            raise ValueError("Waiting for currentP from the robot motion node")
        with self.motion_lock:
            if self.motion_busy:
                raise ValueError("A trajectory is already running")
            self.motion_busy = True
            self.motion_stop_event = threading.Event()
            self.motion_pause_event = threading.Event()
            self.robot_command_state = f"{label} preparing"
            stop_event = self.motion_stop_event
            pause_event = self.motion_pause_event
        return current, stop_event, pause_event

    def _start_worker(self, label, builder):
        current, stop_event, pause_event = self._begin_motion(label)

        def wait_while_paused():
            while pause_event.is_set() and not stop_event.is_set():
                stop_event.wait(0.1)
            return stop_event.is_set()

        def run():
            mode_active = False
            try:
                segments = builder(current)
                for mode, rows in segments:
                    if wait_while_paused():
                        break
                    self.robot_command_state = f"{label}: {mode} mode"
                    mode_msg = String()
                    mode_msg.data = mode
                    self.mode_pub.publish(mode_msg)
                    mode_active = True
                    if stop_event.wait(0.05) or wait_while_paused():
                        break
                    next_publish = time.monotonic()
                    for row in rows:
                        if wait_while_paused():
                            break
                        motion_msg = Float64MultiArray()
                        motion_msg.data = row
                        self.cmd_motion_pub.publish(motion_msg)
                        next_publish += self.control_period
                        if stop_event.wait(max(0.0, next_publish - time.monotonic())):
                            break
                    if stop_event.is_set():
                        break
                with self.motion_lock:
                    stopped = stop_event.is_set()
                    if not stopped:
                        self.motion_busy = False
                        if self.motion_stop_event is stop_event:
                            self.motion_stop_event = None
                if stopped:
                    self._publish_idle()
                    self.robot_command_state = f"{label} stopped, Idling"
                    self.get_logger().info(f"{label} stopped; switched to Idling")
                else:
                    self.robot_command_state = f"{label} complete"
                    self.get_logger().info(f"{label} trajectory completed")
            except Exception as exc:  # noqa: BLE001
                with self.motion_lock:
                    stopped = stop_event.is_set()
                    if not stopped:
                        self.motion_busy = False
                        if self.motion_stop_event is stop_event:
                            self.motion_stop_event = None
                if mode_active or stopped:
                    self._publish_idle()
                if stopped:
                    self.robot_command_state = f"{label} stopped, Idling"
                    self.get_logger().info(f"{label} stopped; switched to Idling")
                else:
                    self.robot_command_state = f"{label} failed: {exc}"
                    self.get_logger().error(self.robot_command_state)
            finally:
                with self.motion_lock:
                    self.motion_busy = False
                    if self.motion_stop_event is stop_event:
                        self.motion_stop_event = None

        threading.Thread(target=run, name="y2_gui_motion", daemon=True).start()

    def _publish_idle(self):
        idle_msg = String()
        idle_msg.data = "Idling"
        self.mode_pub.publish(idle_msg)

    def stop_motion(self):
        with self.motion_lock:
            if not self.motion_busy or self.motion_stop_event is None:
                raise ValueError("No PTP or TXTLoad motion is running")
            self.motion_stop_event.set()
            self.robot_command_state = "Stopping; switching to Idling"

    def pause_motion(self):
        with self.motion_lock:
            if not self.motion_busy or self.motion_pause_event is None:
                raise ValueError("No PTP or TXTLoad motion is running")
            if self.motion_stop_event is not None and self.motion_stop_event.is_set():
                raise ValueError("Motion is stopping")
            self.motion_pause_event.set()
            self.robot_command_state = "Paused"

    def resume_motion(self):
        with self.motion_lock:
            if not self.motion_busy or self.motion_pause_event is None:
                raise ValueError("No paused PTP or TXTLoad motion is running")
            self.motion_pause_event.clear()
            self.robot_command_state = "Resuming"

    def start_ptp(self, target, velocity):
        if not isinstance(target, list) or len(target) != 6:
            raise ValueError("PTP target must contain x, y, z, wx, wy, wz")
        try:
            values = [float(value) for value in target]
            speed = float(velocity)
        except (TypeError, ValueError) as exc:
            raise ValueError("PTP values must be numeric") from exc
        if not all(math.isfinite(value) for value in values + [speed]):
            raise ValueError("PTP values must be finite")
        if speed <= 0:
            raise ValueError("PTP velocity must be greater than zero")
        self._start_worker("PTP", lambda current: [(
            "Position",
            _blend_motion(
                [current, values[:3] + [math.radians(value) for value in values[3:]]],
                [0.0, speed], [0.0, 0.0], [0.0, 0.0], self.control_period,
                angular_velocity_limit=math.radians(5.0),
            ),
        )])

    def _load_numeric_path(self, path_type, expected_columns):
        file_path = self.path_files[path_type]["file"]
        if not file_path.exists():
            raise ValueError(f"path file does not exist: {file_path}")
        rows = []
        for line_number, line in enumerate(file_path.read_text(encoding="utf-8").splitlines(), start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            try:
                row = [float(value) for value in stripped.split()]
            except ValueError as exc:
                raise ValueError(f"{file_path.name}:{line_number} contains a non-numeric value") from exc
            if len(row) != expected_columns or not all(math.isfinite(value) for value in row):
                raise ValueError(
                    f"{file_path.name}:{line_number} must contain {expected_columns} finite values"
                )
            rows.append(row)
        if not rows:
            raise ValueError(f"path file is empty: {file_path}")
        return rows

    def start_txt_load(self):
        path_type = {0: "6D", 1: "9D", 2: "CONTINUE9D"}.get(self.trajectory_mode)
        if path_type is None:
            raise ValueError(f"unsupported trajectory_mode: {self.trajectory_mode}")
        self._start_worker("TXTLoad", lambda current: self._build_txt_motion(path_type, current))

    def _build_txt_motion(self, path_type, current):
        if path_type == "CONTINUE9D":
            continue_rows = self._load_numeric_path("CONTINUE9D", 9)
            first = continue_rows[0]
            translation_gap = math.sqrt(sum((first[i] - current[i]) ** 2 for i in range(3)))
            rotation_gap = math.degrees(math.sqrt(sum((first[i] - current[i]) ** 2 for i in range(3, 6))))
            if translation_gap > 200.0 or rotation_gap > 30.0:
                raise ValueError(
                    f"start gap exceeds safety limits ({translation_gap:.1f} mm, {rotation_gap:.1f} deg)"
                )
            connect_rows = _blend_motion(
                [current, first[:6]], [0.0, 15.0], [0.0, 0.0], [0.0, 0.0],
                self.control_period, angular_velocity_limit=math.radians(5.0),
            )
            return [("Position", connect_rows), ("Force", continue_rows)]

        is_force = path_type == "9D"
        source = self._load_numeric_path(path_type, 12 if is_force else 9)
        points = []
        for row in source:
            pose = row[:6]
            pose[3:] = [math.radians(value) for value in pose[3:]]
            if is_force:
                pose += row[6:9]
            points.append(pose)
        if is_force:
            points = [[*current, 0.0, 0.0, 0.0], *points, [*current, 0.0, 0.0, 0.0]]
            velocity = [0.0] + [row[9] for row in source] + [0.0]
            angular = [0.0] + [row[10] for row in source] + [0.0]
            holding = [0.0] + [row[11] for row in source] + [0.0]
            mode = "Force"
        else:
            points = [current, *points, current]
            velocity = [0.0] + [row[6] for row in source] + [0.0]
            angular = [0.0] + [row[7] for row in source] + [0.0]
            holding = [0.0] + [row[8] for row in source] + [0.0]
            mode = "Position"
        velocity[1] = 15.0
        velocity[-1] = 15.0
        angular[1] = 0.0
        angular[-1] = 0.0
        holding[1] = 0.0
        holding[-1] = 0.0
        blended = _blend_motion(
            points, velocity, angular, holding, self.control_period,
            angular_velocity_limit=math.radians(5.0),
        )
        if is_force:
            output_path = self.path_files["CONTINUE9D"]["file"]
            output_path.write_text(
                "".join(" ".join(f"{value:.12g}" for value in row) + "\n" for row in blended),
                encoding="utf-8",
            )
        return [(mode, blended)]

    def find_path_files(self):
        txtcmd_dir = None
        config_path = None
        try:
            config_path = (
                Path(get_package_share_directory("Y2RobMotion"))
                / "config"
                / "y2_rob_motion.yaml"
            )
            package_bundle_dir = self.read_package_bundle_dir(config_path)
            if package_bundle_dir:
                candidate = Path(package_bundle_dir) / "Y2RobMotion" / "txtcmd"
                if candidate.exists():
                    txtcmd_dir = candidate
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Could not resolve Y2RobMotion txtcmd from config: {exc}")

        if txtcmd_dir is None:
            fallback = Path.home() / "armcon_ws" / "src" / "armcon_pack" / "Y2RobMotion" / "txtcmd"
            txtcmd_dir = fallback

        config_text = ""
        if config_path is not None:
            try:
                config_text = config_path.read_text(encoding="utf-8")
            except OSError:
                pass
        if self.trajectory_mode < 0:
            match = re.search(r"^\s*trajectory_mode:\s*(\d+)", config_text, re.MULTILINE)
            self.trajectory_mode = int(match.group(1)) if match else 1
        if self.control_period <= 0:
            match = re.search(r"^\s*control_period:\s*([0-9.eE+-]+)", config_text, re.MULTILINE)
            self.control_period = float(match.group(1)) if match else 0.001

        self.txtcmd_dir = txtcmd_dir
        return {
            "6D": {
                "file": txtcmd_dir / "cmd_6D.txt",
                "columns": ["x", "y", "z", "wx", "wy", "wz", "desired_lin_vel", "desired_ang_vel", "holding_time"],
            },
            "9D": {
                "file": txtcmd_dir / "cmd_9D.txt",
                "columns": ["x", "y", "z", "wx", "wy", "wz", "fx", "fy", "fz", "desired_lin_vel", "desired_ang_vel", "holding_time"],
            },
            "CONTINUE9D": {
                "file": txtcmd_dir / "cmd_continue9D.txt",
                "columns": ["x", "y", "z", "wx", "wy", "wz", "fx", "fy", "fz"],
            },
        }

    def find_measured_files(self):
        measured_dir = None
        try:
            config_path = (
                Path(get_package_share_directory("Y2RobMotion"))
                / "config"
                / "y2_rob_motion.yaml"
            )
            package_bundle_dir = self.read_package_bundle_dir(config_path)
            if package_bundle_dir:
                candidate = Path(package_bundle_dir) / "Y2RobMotion" / "measured"
                if candidate.exists():
                    measured_dir = candidate
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Could not resolve Y2RobMotion measured dir from config: {exc}")

        if measured_dir is None:
            measured_dir = (
                Path.home() / "armcon_ws" / "src" / "armcon_pack" / "Y2RobMotion" / "measured"
            )

        return {
            "currentP": {
                "file": measured_dir / "currentP.txt",
                "columns": ["x", "y", "z", "wx", "wy", "wz"],
                "units": ["mm", "mm", "mm", "deg", "deg", "deg"],
            },
            "targetP": {
                "file": measured_dir / "targetP.txt",
                "columns": ["x", "y", "z", "wx", "wy", "wz"],
                "units": ["mm", "mm", "mm", "deg", "deg", "deg"],
            },
            "currentJ": {
                "file": measured_dir / "currentJ.txt",
                "columns": ["j1", "j2", "j3", "j4", "j5", "j6"],
                "units": ["rad", "rad", "rad", "rad", "rad", "rad"],
            },
            "targetJ": {
                "file": measured_dir / "targetJ.txt",
                "columns": ["j1", "j2", "j3", "j4", "j5", "j6"],
                "units": ["rad", "rad", "rad", "rad", "rad", "rad"],
            },
            "currentF": {
                "file": measured_dir / "currentF.txt",
                "columns": ["fx", "fy", "fz", "tx", "ty", "tz"],
                "units": ["N", "N", "N", "Nm", "Nm", "Nm"],
            },
            "targetF": {
                "file": measured_dir / "targetF.txt",
                "columns": ["fx", "fy", "fz", "tx", "ty", "tz"],
                "units": ["N", "N", "N", "Nm", "Nm", "Nm"],
            },
            "currentMDK": {
                "file": measured_dir / "currentMDK.txt",
                "columns": ["m", "d", "k"],
                "units": ["", "", ""],
            },
        }

    def read_package_bundle_dir(self, config_path):
        if not config_path.exists():
            return None

        for line in config_path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if stripped.startswith("package_bundle_dir:"):
                return stripped.split(":", 1)[1].strip().strip("'\"")
        return None

    def measured_file_status(self):
        files = []
        for file_id, spec in self.measured_files.items():
            file_path = spec["file"]
            rows = 0
            if file_path.exists():
                with file_path.open("r", encoding="utf-8") as handle:
                    rows = sum(1 for line in handle if line.strip())
            files.append({
                "id": file_id,
                "file": str(file_path),
                "columns": spec["columns"],
                "units": spec["units"],
                "exists": file_path.exists(),
                "rows": rows,
            })
        return files

    def measured_spec(self, file_id):
        spec = self.measured_files.get(file_id)
        if spec is None:
            raise ValueError(f"unsupported measured file: {file_id}")
        return spec

    def read_measured_file(self, file_id):
        spec = self.measured_spec(file_id)
        file_path = spec["file"]
        if not file_path.exists():
            raise ValueError(f"measured file does not exist: {file_path}")

        rows = []
        for line in file_path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            rows.append(stripped.split())

        return {
            "id": file_id,
            "file": str(file_path),
            "columns": spec["columns"],
            "units": spec["units"],
            "rows": rows,
        }

    def path_spec(self, path_type):
        normalized = path_type.upper()
        spec = self.path_files.get(normalized)
        if spec is None:
            raise ValueError(f"unsupported path type: {path_type}")
        return normalized, spec

    def read_path_file(self, path_type):
        normalized, spec = self.path_spec(path_type)
        file_path = spec["file"]
        if not file_path.exists():
            raise ValueError(f"path file does not exist: {file_path}")

        rows = []
        for line in file_path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            rows.append(stripped.split())

        return {
            "type": normalized,
            "file": str(file_path),
            "columns": spec["columns"],
            "rows": rows,
        }

    def write_path_file(self, path_type, rows):
        normalized, spec = self.path_spec(path_type)
        if not isinstance(rows, list):
            raise ValueError("rows must be a list")

        column_count = len(spec["columns"])
        clean_rows = []
        for row_index, row in enumerate(rows, start=1):
            if not isinstance(row, list):
                raise ValueError(f"row {row_index} must be a list")
            if len(row) != column_count:
                raise ValueError(f"row {row_index} must have {column_count} columns")

            clean_row = []
            for column_index, value in enumerate(row, start=1):
                text = str(value).strip()
                if text == "":
                    raise ValueError(f"row {row_index}, column {column_index} is empty")
                try:
                    float(text)
                except ValueError as exc:
                    raise ValueError(
                        f"row {row_index}, column {column_index} is not numeric: {text}"
                    ) from exc
                clean_row.append(text)
            clean_rows.append(clean_row)

        file_path = spec["file"]
        file_path.parent.mkdir(parents=True, exist_ok=True)
        content = "\n".join(" ".join(row) for row in clean_rows)
        if content:
            content += "\n"
        file_path.write_text(content, encoding="utf-8")
        self.get_logger().info(f"Saved {normalized} path file: {file_path}")

        return {
            "type": normalized,
            "file": str(file_path),
            "columns": spec["columns"],
            "rows": clean_rows,
        }

    def build_process_specs(self):
        specs = [
            {
                "id": "y2_joystick_launch",
                "name": "Y2 Joystick",
                "group": "Input",
                "command": ["ros2", "launch", "Y2_Joystick", "y2_joystick.launch.py"],
                "description": "Start joy_node and Y2 joystick topic mapper.",
            },
            {
                "id": "ur10e_control",
                "name": "UR10e Control",
                "group": "Y2RobMotion",
                "command": ["ros2", "launch", "Y2RobMotion", "ur10e_control.launch.py"],
                "description": "Start UR10e robot control launch.",
                "confirm": True,
            },
            {
                "id": "ur_control",
                "name": "UR Control",
                "group": "Y2RobMotion",
                "command": ["ros2", "launch", "Y2RobMotion", "ur_control.launch.py"],
                "description": "Start UR control launch.",
                "confirm": True,
            },
            {
                "id": "single_arm",
                "name": "Single Arm",
                "group": "Y2RobMotion",
                "command": ["ros2", "launch", "Y2RobMotion", "ur_singleArm.launch.py"],
                "description": "Start single-arm motion and command nodes.",
                "confirm": True,
            },
            {
                "id": "mujoco_arm",
                "name": "MuJoCo Arm",
                "group": "Y2RobMotion",
                "command": ["ros2", "launch", "Y2RobMotion", "ur_mujocoArm.launch.py"],
                "description": "Start MuJoCo arm launch.",
            },
            {
                "id": "isaac_arm",
                "name": "Isaac Arm",
                "group": "Y2RobMotion",
                "command": ["ros2", "launch", "Y2RobMotion", "ur_issacArm.launch.py"],
                "description": "Start Isaac arm launch.",
            },
            {
                "id": "robot_measure",
                "name": "Robot Measure",
                "group": "Y2RobMotion",
                "command": ["ros2", "run", "Y2RobMotion", "robot_measure"],
                "description": "Start measured data recorder.",
                "options": {
                    "mode": {
                        "type": "choice",
                        "label": "Mode",
                        "default": "continuous",
                        "choices": [
                            {"value": "continuous", "label": "Continuous"},
                            {"value": "discrete", "label": "Discrete"},
                        ],
                    },
                },
            },
            {
                "id": "single_arm_motion",
                "name": "Single Arm Motion",
                "group": "Y2RobMotion",
                "command": ["ros2", "run", "Y2RobMotion", "singleArm_motion"],
                "description": "Run the single-arm motion node.",
                "confirm": True,
            },
            {
                "id": "cartesian_id",
                "name": "Cartesian ID",
                "group": "Y2SYS_ID",
                "command": ["ros2", "launch", "Y2SYS_ID", "cartesian_id.launch.py"],
                "description": "Start Cartesian system identification launch.",
                "confirm": True,
            },
            {
                "id": "response_recorder",
                "name": "Response Recorder",
                "group": "Y2SYS_ID",
                "command": ["ros2", "run", "Y2SYS_ID", "response_recorder_node"],
                "description": "Record system identification responses.",
            },
            {
                "id": "online_metrics",
                "name": "Online Metrics",
                "group": "Y2SYS_ID",
                "command": ["ros2", "run", "Y2SYS_ID", "online_metrics_node"],
                "description": "Run online system ID metrics.",
            },
            {
                "id": "system_id",
                "name": "System ID",
                "group": "Y2SYS_ID",
                "command": ["ros2", "run", "Y2SYS_ID", "system_id_node"],
                "description": "Run system identification node.",
            },
            {
                "id": "isaac_bridge",
                "name": "Isaac Bridge",
                "group": "Bridge",
                "command": ["ros2", "run", "y2_isaac_bridge", "joint_command_bridge"],
                "description": "Bridge joint commands for Isaac integration.",
            },
            {
                "id": "ft_get_main",
                "name": "FT Get Main",
                "group": "FT Sensor",
                "command": ["ros2", "run", "Y2FT_AQ", "FTGetMain"],
                "description": "Start force-torque acquisition.",
            },
        ]

        return {
            spec["id"]: {
                **spec,
                "process": None,
                "returncode": None,
                "log": [],
            }
            for spec in specs
        }

    def process_status(self):
        with self.process_lock:
            statuses = []
            for spec in self.managed_processes.values():
                process = spec["process"]
                if process is not None:
                    spec["returncode"] = process.poll()
                    if spec["returncode"] is not None:
                        spec["process"] = None

                statuses.append(self.serialize_process(spec))
            return statuses

    def serialize_process(self, spec):
        process = spec["process"]
        return {
            "id": spec["id"],
            "name": spec["name"],
            "group": spec["group"],
            "description": spec["description"],
            "command": " ".join(spec["command"]),
            "options": spec.get("options", {}),
            "confirm": bool(spec.get("confirm", False)),
            "running": process is not None and process.poll() is None,
            "pid": process.pid if process is not None else None,
            "returncode": spec["returncode"],
            "log": spec["log"][-8:],
        }

    def start_process(self, process_id, options=None):
        with self.process_lock:
            spec = self.get_process_spec(process_id)
            process = spec["process"]
            if process is not None and process.poll() is None:
                return self.serialize_process(spec)

            spec["log"].clear()
            spec["returncode"] = None
            command = self.command_for_options(spec, options or {})
            command_text = shlex.join(command)
            setup_script = shlex.quote(str(self.workspace_setup))
            ros_distro = shlex.quote(os.environ.get("ROS_DISTRO", "humble"))
            launch_command = [
                "bash", "--noprofile", "--norc", "-c",
                f"source /opt/ros/{ros_distro}/setup.bash && "
                f"source {setup_script} && "
                f"exec {command_text}",
            ]
            child_env = os.environ.copy()
            # Remove stale overlays so armcon_ws is the package resolution root.
            for variable in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
                child_env.pop(variable, None)
            process = subprocess.Popen(
                launch_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=child_env,
            )
            spec["process"] = process
            threading.Thread(
                target=self.capture_process_output,
                args=(process_id, process),
                daemon=True,
            ).start()
            self.get_logger().info(
                f"Started managed process {process_id}: {' '.join(command)}")
            return self.serialize_process(spec)

    def stop_process(self, process_id):
        with self.process_lock:
            spec = self.get_process_spec(process_id)
            process = spec["process"]
            if process is None or process.poll() is not None:
                spec["process"] = None
                spec["returncode"] = process.poll() if process is not None else spec["returncode"]
                return self.serialize_process(spec)

            os.killpg(process.pid, signal.SIGINT)

        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)

        with self.process_lock:
            spec["returncode"] = process.returncode
            spec["process"] = None
            self.get_logger().info(f"Stopped managed process {process_id}")
            return self.serialize_process(spec)

    def get_process_spec(self, process_id):
        spec = self.managed_processes.get(process_id)
        if spec is None:
            raise ValueError(f"unknown process id: {process_id}")
        return spec

    def command_for_options(self, spec, selected_options):
        command = list(spec["command"])
        if spec["id"] == "robot_measure":
            mode = selected_options.get("mode", "continuous")
            if mode not in {"continuous", "discrete"}:
                raise ValueError(f"unsupported robot_measure mode: {mode}")
            command += ["--ros-args", "-p", f"mode:={mode}"]
        return command

    def capture_process_output(self, process_id, process):
        if process.stdout is None:
            return

        for line in process.stdout:
            clean_line = line.rstrip()
            if not clean_line:
                continue
            with self.process_lock:
                spec = self.managed_processes.get(process_id)
                if spec is None or spec["process"] is not process:
                    return
                spec["log"].append(clean_line)
                spec["log"] = spec["log"][-80:]

        process.wait()
        with self.process_lock:
            spec = self.managed_processes.get(process_id)
            if spec is not None and spec["process"] is process:
                spec["returncode"] = process.returncode
                spec["process"] = None

    def destroy_node(self):
        for process_id in list(self.managed_processes.keys()):
            try:
                self.stop_process(process_id)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Failed to stop {process_id}: {exc}")
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Y2JoyGuiNode()
    if not node.start_server():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
