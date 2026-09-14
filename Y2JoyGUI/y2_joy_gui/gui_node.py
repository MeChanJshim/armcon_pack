import json
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


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

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            name="y2_joy_gui_http",
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            f"Y2JoyGUI ready: http://127.0.0.1:{self.port} "
            f"(serving {self.web_dir})")

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

    def destroy_node(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Y2JoyGuiNode()
    node.start_server()
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
