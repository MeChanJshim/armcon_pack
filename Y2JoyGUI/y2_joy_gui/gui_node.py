import json
import os
import signal
import subprocess
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class Y2JoyGuiNode(Node):
    def __init__(self):
        super().__init__("y2_joy_gui_node")

        self.robot_name = self.declare_parameter("robot_name", "ur10skku").value
        self.host = self.declare_parameter("host", "0.0.0.0").value
        self.port = int(self.declare_parameter("port", 8080).value)
        self.joy_axis_count = int(self.declare_parameter("joy_axis_count", 8).value)

        base_topic = "/" + self.robot_name.strip("/")
        self.mode_pub = self.create_publisher(String, base_topic + "/cmdMode", 10)
        self.joy_move_pub = self.create_publisher(
            Float64MultiArray, base_topic + "/joy_move", 10)
        self.record_stop_pub = self.create_publisher(
            Bool, base_topic + "/record_boolean_L", 10)
        self.record_start_pub = self.create_publisher(
            Bool, base_topic + "/record_boolean_R", 10)

        self.allowed_modes = {"Idling", "Guiding", "Joystick", "Joystick_force"}
        self.web_dir = Path(get_package_share_directory("Y2JoyGUI")) / "web"
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
                super().end_headers()

            def do_OPTIONS(self):
                self.send_response(204)
                self.end_headers()

            def do_GET(self):
                if self.path == "/api/status":
                    self.write_json({
                        "ok": True,
                        "robot_name": node.robot_name,
                        "topics": {
                            "mode": f"/{node.robot_name}/cmdMode",
                            "joy_move": f"/{node.robot_name}/joy_move",
                            "record_start": f"/{node.robot_name}/record_boolean_R",
                            "record_stop": f"/{node.robot_name}/record_boolean_L",
                        },
                    })
                    return

                if self.path == "/api/processes":
                    self.write_json({
                        "ok": True,
                        "processes": node.process_status(),
                    })
                    return

                if self.path == "/":
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
        return True

    def publish_mode(self, mode):
        if mode not in self.allowed_modes:
            raise ValueError(f"unsupported mode: {mode}")
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Published mode: {mode}")

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
                "id": "single_arm_cmd",
                "name": "Single Arm Command",
                "group": "Y2RobMotion",
                "command": ["ros2", "run", "Y2RobMotion", "singleArm_cmd"],
                "description": "Run the terminal command node.",
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
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=os.environ.copy(),
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
