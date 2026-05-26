"""
bridge_node.py
--------------
Combined MUMT bridge node:
  - Receives aircraft state from UE5 (UDP port 5006) → /mumt/aircraft_states
  - Sends JSON commands from /mumt/aircraft_commands → UE5 (UDP port 5005)
  - Sends autopilot setpoints from /aircraft/setpoint → UE5 (UDP port 5010)
"""

import json
import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from custom_msgs.msg import AircraftSetpoint
except ImportError as exc:
    raise SystemExit(
        "custom_msgs not found — build the workspace first: "
        "colcon build --packages-select custom_msgs"
    ) from exc

_SETPOINT_FMT = "<BfffBBH"   # 17 bytes
assert struct.calcsize(_SETPOINT_FMT) == 17


class MumtBridgeNode(Node):
    def __init__(self):
        super().__init__("mumt_bridge")

        self.declare_parameter("unreal_ip",      "127.0.0.1")
        self.declare_parameter("control_port",   5005)
        self.declare_parameter("state_port",     5006)
        self.declare_parameter("setpoint_port",  5010)

        self._unreal_ip     = self.get_parameter("unreal_ip").value
        self._control_port  = int(self.get_parameter("control_port").value)
        self._state_port    = int(self.get_parameter("state_port").value)
        self._setpoint_port = int(self.get_parameter("setpoint_port").value)

        # UDP sockets
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sp_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.bind(("0.0.0.0", self._state_port))
        self._recv_sock.setblocking(False)

        self._seq: int = 0

        # ROS publishers / subscribers
        self._state_pub = self.create_publisher(String, "/mumt/aircraft_states", 10)

        self.create_subscription(String, "/mumt/aircraft_commands",
                                 self._on_command, 10)
        self.create_subscription(AircraftSetpoint, "/aircraft/setpoint",
                                 self._on_setpoint, 10)

        self.create_timer(0.02, self._recv_state)

        self.get_logger().info(
            f"MUMT bridge started | "
            f"state UDP <- 0.0.0.0:{self._state_port} | "
            f"command UDP -> {self._unreal_ip}:{self._control_port} | "
            f"setpoint UDP -> {self._unreal_ip}:{self._setpoint_port}"
        )

    def _recv_state(self):
        while True:
            try:
                data, _ = self._recv_sock.recvfrom(65535)
            except BlockingIOError:
                break

            text = data.decode("utf-8", errors="replace")
            try:
                json.loads(text)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON from Unreal: {text[:80]}")
                continue

            msg = String()
            msg.data = text
            self._state_pub.publish(msg)

    def _on_command(self, msg: String):
        try:
            json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid command JSON: {msg.data}")
            return
        self._cmd_sock.sendto(msg.data.encode("utf-8"),
                              (self._unreal_ip, self._control_port))

    def _on_setpoint(self, msg: AircraftSetpoint):
        seq = self._seq & 0xFFFF
        self._seq += 1
        packet = struct.pack(
            _SETPOINT_FMT,
            0x01,
            float(msg.heading_deg),
            float(msg.altitude_m),
            float(max(0.0, min(1.0, msg.throttle_norm))),
            int(bool(msg.launch_missile)),
            0,
            seq,
        )
        try:
            self._sp_sock.sendto(packet, (self._unreal_ip, self._setpoint_port))
        except OSError as e:
            self.get_logger().warn(f"Setpoint UDP send error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MumtBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
