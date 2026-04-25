#!/usr/bin/env python3
import os
import socket
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class YoloUdpBridge(Node):
    def __init__(self):
        super().__init__("yolo_udp_bridge")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("topic", "/vision/traffic_sign_detections")
        self.declare_parameter("latest_json_path", "/home/ubuntu/camera_web/latest_detections.json")

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.topic = str(self.get_parameter("topic").value)
        self.latest_json_path = Path(str(self.get_parameter("latest_json_path").value))

        self.publisher = self.create_publisher(String, self.topic, 10)
        self.packet_count = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.bind_host, self.udp_port))
        self.timer = self.create_timer(0.01, self.poll_socket)
        self.get_logger().info(
            f"Listening on udp://{self.bind_host}:{self.udp_port}, publishing {self.topic}, "
            f"writing {self.latest_json_path}"
        )

    def poll_socket(self):
        while True:
            try:
                data, address = self.sock.recvfrom(65535)
            except BlockingIOError:
                return

            msg = String()
            msg.data = data.decode("utf-8", "replace")
            self.publisher.publish(msg)
            self.write_latest(data)
            self.packet_count += 1
            if self.packet_count <= 5 or self.packet_count % 100 == 0:
                self.get_logger().info(
                    f"Published YOLO packet #{self.packet_count} from {address[0]}:{address[1]} "
                    f"({len(data)} bytes)"
                )

    def write_latest(self, data):
        try:
            self.latest_json_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = self.latest_json_path.with_name(self.latest_json_path.name + ".tmp")
            temp_path.write_bytes(data + b"\n")
            os.replace(temp_path, self.latest_json_path)
        except OSError as exc:
            self.get_logger().warning(f"Failed to write latest YOLO JSON: {exc}")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloUdpBridge()
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
