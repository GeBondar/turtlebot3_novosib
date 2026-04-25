#!/usr/bin/env python3
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node


class CameraWebNode(Node):
    def __init__(self):
        super().__init__("camera_web")
        self.declare_parameter("server_script", "/home/ubuntu/camera_web/camera_web.py")
        self.declare_parameter("python_executable", "/usr/bin/python3")
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_clients", 8)
        self.declare_parameter("restart_on_exit", True)

        self.process = None
        self.start_process()
        self.timer = self.create_timer(1.0, self.check_process)

    def parameter_value(self, name):
        return self.get_parameter(name).value

    def start_process(self):
        env = os.environ.copy()
        env["CAMERA_DEVICE"] = str(self.parameter_value("camera_device"))
        env["CAMERA_WIDTH"] = str(self.parameter_value("width"))
        env["CAMERA_HEIGHT"] = str(self.parameter_value("height"))
        env["CAMERA_FPS"] = str(self.parameter_value("fps"))
        env["CAMERA_PORT"] = str(self.parameter_value("port"))
        env["CAMERA_MAX_CLIENTS"] = str(self.parameter_value("max_clients"))

        cmd = [
            str(self.parameter_value("python_executable")),
            str(self.parameter_value("server_script")),
        ]
        self.get_logger().info("Starting camera web server: " + " ".join(cmd))
        self.process = subprocess.Popen(cmd, env=env)

    def check_process(self):
        if self.process is None:
            return
        return_code = self.process.poll()
        if return_code is None:
            return

        self.get_logger().error(f"camera web server exited with code {return_code}")
        self.process = None
        if bool(self.parameter_value("restart_on_exit")):
            self.start_process()
        else:
            rclpy.shutdown()

    def stop_process(self):
        if self.process is None or self.process.poll() is not None:
            return
        self.get_logger().info("Stopping camera web server")
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)

    def destroy_node(self):
        self.stop_process()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
