#!/usr/bin/env python3
import json
import math
import os
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, LookupException, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return {
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def transform_xy(x, y, transform):
    t = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return {
        "x": t.x + x * cos_yaw - y * sin_yaw,
        "y": t.y + x * sin_yaw + y * cos_yaw,
    }


class WebRvizState(Node):
    def __init__(self):
        super().__init__("web_rviz_state")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("fallback_fixed_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_json_path", "/home/ubuntu/camera_web/latest_scan.json")
        self.declare_parameter("pose_json_path", "/home/ubuntu/camera_web/latest_pose.json")
        self.declare_parameter("costmap_json_path", "/home/ubuntu/camera_web/latest_costmaps.json")
        self.declare_parameter("tf_json_path", "/home/ubuntu/camera_web/latest_tf.json")
        self.declare_parameter("initial_pose_command_path", "/home/ubuntu/camera_web/initial_pose_command.json")
        self.declare_parameter("nav_goal_command_path", "/home/ubuntu/camera_web/nav_goal_command.json")
        self.declare_parameter("nav_goal_status_path", "/home/ubuntu/camera_web/nav_goal_status.json")
        self.declare_parameter("nav_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("nav_goal_command_max_age_s", 120.0)
        self.declare_parameter("nav_controller_node", "/controller_server")
        self.declare_parameter("nav_goal_checker_id", "goal_checker")
        self.declare_parameter("nav_goal_xy_tolerance", 0.08)
        self.declare_parameter("nav_goal_yaw_tolerance", 0.17)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("initial_pose_republish_count", 20)
        self.declare_parameter("initial_pose_command_max_age_s", 30.0)
        self.declare_parameter("initial_pose_xy_variance", 0.25)
        self.declare_parameter("initial_pose_yaw_variance", math.radians(15.0) ** 2)
        self.declare_parameter("max_scan_points", 360)
        self.declare_parameter("max_scan_range_m", 4.5)
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("max_costmap_cells", 65000)
        self.declare_parameter("costmap_write_hz", 2.0)
        self.declare_parameter("tf_hz", 2.0)
        self.declare_parameter(
            "tf_watch_pairs",
            "map:odom,odom:base_link,map:base_link,base_link:base_scan,base_link:base_footprint",
        )
        self.declare_parameter("pose_hz", 10.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.fallback_fixed_frame = str(self.get_parameter("fallback_fixed_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.scan_json_path = Path(str(self.get_parameter("scan_json_path").value))
        self.pose_json_path = Path(str(self.get_parameter("pose_json_path").value))
        self.costmap_json_path = Path(str(self.get_parameter("costmap_json_path").value))
        self.tf_json_path = Path(str(self.get_parameter("tf_json_path").value))
        self.initial_pose_command_path = Path(str(self.get_parameter("initial_pose_command_path").value))
        self.nav_goal_command_path = Path(str(self.get_parameter("nav_goal_command_path").value))
        self.nav_goal_status_path = Path(str(self.get_parameter("nav_goal_status_path").value))
        self.nav_to_pose_action = str(self.get_parameter("nav_to_pose_action").value)
        self.nav_goal_command_max_age_s = float(self.get_parameter("nav_goal_command_max_age_s").value)
        self.nav_controller_node = str(self.get_parameter("nav_controller_node").value).rstrip("/") or "/controller_server"
        if not self.nav_controller_node.startswith("/"):
            self.nav_controller_node = "/" + self.nav_controller_node
        self.nav_goal_checker_id = str(self.get_parameter("nav_goal_checker_id").value).strip() or "goal_checker"
        self.nav_goal_xy_tolerance = float(self.get_parameter("nav_goal_xy_tolerance").value)
        self.nav_goal_yaw_tolerance = float(self.get_parameter("nav_goal_yaw_tolerance").value)
        self.initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.initial_pose_republish_count = max(1, int(self.get_parameter("initial_pose_republish_count").value))
        self.initial_pose_command_max_age_s = float(self.get_parameter("initial_pose_command_max_age_s").value)
        self.initial_pose_xy_variance = float(self.get_parameter("initial_pose_xy_variance").value)
        self.initial_pose_yaw_variance = float(self.get_parameter("initial_pose_yaw_variance").value)
        self.max_scan_points = max(8, int(self.get_parameter("max_scan_points").value))
        self.max_scan_range_m = float(self.get_parameter("max_scan_range_m").value)
        self.global_costmap_topic = str(self.get_parameter("global_costmap_topic").value)
        self.local_costmap_topic = str(self.get_parameter("local_costmap_topic").value)
        self.max_costmap_cells = max(1024, int(self.get_parameter("max_costmap_cells").value))
        costmap_write_hz = max(0.2, float(self.get_parameter("costmap_write_hz").value))
        tf_hz = max(0.2, float(self.get_parameter("tf_hz").value))
        self.tf_watch_pairs = self.parse_tf_watch_pairs(str(self.get_parameter("tf_watch_pairs").value))
        pose_hz = max(1.0, float(self.get_parameter("pose_hz").value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            lambda msg: self.on_costmap("global", msg),
            10,
        )
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            lambda msg: self.on_costmap("local", msg),
            10,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_to_pose_action)
        self.nav_param_client = self.create_client(SetParameters, f"{self.nav_controller_node}/set_parameters")
        self.pose_timer = self.create_timer(1.0 / pose_hz, self.write_pose)
        self.tf_timer = self.create_timer(1.0 / tf_hz, self.write_tf_snapshot)
        self.initial_pose_timer = self.create_timer(0.1, self.poll_initial_pose_command)
        self.nav_goal_timer = self.create_timer(0.2, self.poll_nav_goal_command)
        self.nav_tolerance_timer = self.create_timer(2.0, self.apply_nav_goal_tolerances)
        self.costmaps = {}
        self.last_costmap_write_unix = 0.0
        self.costmap_write_interval_s = 1.0 / costmap_write_hz
        self.last_initial_pose_seq = None
        self.pending_initial_pose = None
        self.pending_initial_pose_repeats = 0
        self.last_nav_goal_seq = None
        self.pending_nav_goal = None
        self.nav_goal_in_flight = False
        self.active_nav_goal_seq = None
        self.active_nav_goal_command = None
        self.nav_tolerances_applied = False
        self.nav_tolerance_request_in_flight = False

        self.get_logger().info(
            f"writing web rviz state: scan {self.scan_topic} -> {self.scan_json_path}, "
            f"{self.fixed_frame}/{self.fallback_fixed_frame}->{self.base_frame} -> {self.pose_json_path}, "
            f"costmaps {self.global_costmap_topic}/{self.local_costmap_topic} -> {self.costmap_json_path}, "
            f"tf snapshot -> {self.tf_json_path}, "
            f"initial pose {self.initial_pose_command_path} -> {self.initial_pose_topic}, "
            f"nav goals {self.nav_goal_command_path} -> {self.nav_to_pose_action}, "
            f"goal checker {self.nav_controller_node}/{self.nav_goal_checker_id}: "
            f"xy {self.nav_goal_xy_tolerance} m, yaw {self.nav_goal_yaw_tolerance} rad"
        )

    def write_json(self, path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = path.with_name(path.name + ".tmp")
        temp_path.write_text(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
        os.replace(temp_path, path)

    def read_json_file(self, path, label):
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except Exception as exc:
            self.get_logger().warning(f"failed to read {label}: {exc}")
            return None

    def read_initial_pose_command(self):
        return self.read_json_file(self.initial_pose_command_path, "initial pose command")

    def read_nav_goal_command(self):
        return self.read_json_file(self.nav_goal_command_path, "nav goal command")

    def parse_tf_watch_pairs(self, raw):
        pairs = []
        for item in raw.split(","):
            item = item.strip()
            if not item or ":" not in item:
                continue
            target, source = [part.strip().strip("/") for part in item.split(":", 1)]
            if target and source:
                pairs.append((target, source))
        return pairs

    def write_nav_goal_status(self, status, command=None, **extra):
        payload = {
            "schema": "tb3_nav_goal_status.v1",
            "status": status,
            "stamp_unix": time.time(),
            "action": self.nav_to_pose_action,
        }
        if command is not None:
            payload["seq"] = command.get("seq")
            payload["goal"] = {
                "target_point": command.get("target_point"),
                "pose": command.get("pose"),
                "frame_id": command.get("frame_id"),
            }
        if extra:
            payload.update(extra)
        self.write_json(self.nav_goal_status_path, payload)

    def duration_to_seconds(self, duration_msg):
        if duration_msg is None:
            return None
        sec = getattr(duration_msg, "sec", 0)
        nanosec = getattr(duration_msg, "nanosec", 0)
        return round(float(sec) + float(nanosec) / 1_000_000_000.0, 3)

    def goal_status_label(self, status):
        labels = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return labels.get(status, f"status_{status}")

    def double_parameter(self, name, value):
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )

    def apply_nav_goal_tolerances(self):
        if self.nav_tolerances_applied or self.nav_tolerance_request_in_flight:
            return
        if self.nav_goal_xy_tolerance <= 0 or self.nav_goal_yaw_tolerance <= 0:
            self.nav_tolerances_applied = True
            return
        if not self.nav_param_client.service_is_ready():
            return

        prefix = self.nav_goal_checker_id
        request = SetParameters.Request()
        request.parameters = [
            self.double_parameter(f"{prefix}.xy_goal_tolerance", self.nav_goal_xy_tolerance),
            self.double_parameter(f"{prefix}.yaw_goal_tolerance", self.nav_goal_yaw_tolerance),
        ]
        self.nav_tolerance_request_in_flight = True
        future = self.nav_param_client.call_async(request)
        future.add_done_callback(self.on_nav_tolerances_applied)

    def on_nav_tolerances_applied(self, future):
        self.nav_tolerance_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"failed to set Nav2 goal tolerances: {exc}")
            return

        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            self.get_logger().warning(f"Nav2 goal tolerance update rejected: {failed}")
            return

        self.nav_tolerances_applied = True
        self.get_logger().info(
            f"set Nav2 goal tolerances: {self.nav_goal_checker_id}.xy_goal_tolerance="
            f"{self.nav_goal_xy_tolerance}, {self.nav_goal_checker_id}.yaw_goal_tolerance="
            f"{self.nav_goal_yaw_tolerance}"
        )

    def lookup_transform(self, target_frame, source_frame):
        if target_frame == source_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except (LookupException, TransformException) as exc:
            self.get_logger().debug(f"tf lookup failed {target_frame} <- {source_frame}: {exc}")
            return None

    def serialize_transform(self, target_frame, source_frame):
        transform = self.lookup_transform(target_frame, source_frame)
        if transform is None:
            return {
                "target_frame": target_frame,
                "source_frame": source_frame,
                "status": "missing",
            }

        t = transform.transform.translation
        r = transform.transform.rotation
        return {
            "target_frame": target_frame,
            "source_frame": source_frame,
            "status": "ok",
            "translation": {
                "x": round(float(t.x), 4),
                "y": round(float(t.y), 4),
                "z": round(float(t.z), 4),
            },
            "rotation": {
                "x": round(float(r.x), 6),
                "y": round(float(r.y), 6),
                "z": round(float(r.z), 6),
                "w": round(float(r.w), 6),
            },
            "yaw": round(yaw_from_quaternion(r), 6),
        }

    def write_tf_snapshot(self):
        try:
            frames_text = self.tf_buffer.all_frames_as_yaml()
        except Exception as exc:
            frames_text = f"tf frame dump unavailable: {exc}"

        payload = {
            "schema": "tb3_web_tf.v1",
            "status": "ok",
            "stamp_unix": time.time(),
            "fixed_frame": self.fixed_frame,
            "fallback_fixed_frame": self.fallback_fixed_frame,
            "base_frame": self.base_frame,
            "watched_transforms": [
                self.serialize_transform(target, source)
                for target, source in self.tf_watch_pairs
            ],
            "frames_text": frames_text,
        }
        self.write_json(self.tf_json_path, payload)

    def downsample_costmap_data(self, data, width, height):
        cell_count = max(1, width * height)
        stride = max(1, math.ceil(math.sqrt(cell_count / self.max_costmap_cells)))
        if stride == 1:
            return list(data), width, height, stride

        out_width = math.ceil(width / stride)
        out_height = math.ceil(height / stride)
        out = []
        for out_y in range(out_height):
            y0 = out_y * stride
            y1 = min(height, y0 + stride)
            for out_x in range(out_width):
                x0 = out_x * stride
                x1 = min(width, x0 + stride)
                best = -1
                seen_known = False
                for y in range(y0, y1):
                    row_offset = y * width
                    for x in range(x0, x1):
                        value = int(data[row_offset + x])
                        if value >= 0:
                            seen_known = True
                            if value > best:
                                best = value
                out.append(best if seen_known else -1)
        return out, out_width, out_height, stride

    def serialize_costmap(self, name, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        data, out_width, out_height, stride = self.downsample_costmap_data(msg.data, width, height)
        origin = msg.info.origin
        return {
            "status": "ok",
            "name": name,
            "topic": self.global_costmap_topic if name == "global" else self.local_costmap_topic,
            "stamp_unix": time.time(),
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "resolution": float(msg.info.resolution) * stride,
            "source_resolution": float(msg.info.resolution),
            "width": out_width,
            "height": out_height,
            "source_width": width,
            "source_height": height,
            "stride": stride,
            "origin": {
                "x": round(float(origin.position.x), 4),
                "y": round(float(origin.position.y), 4),
                "yaw": round(yaw_from_quaternion(origin.orientation), 6),
            },
            "data": data,
        }

    def write_costmaps_snapshot(self):
        now = time.time()
        if now - self.last_costmap_write_unix < self.costmap_write_interval_s:
            return
        self.last_costmap_write_unix = now
        payload = {
            "schema": "tb3_web_costmaps.v1",
            "status": "ok" if self.costmaps else "no_data",
            "stamp_unix": now,
            "topics": {
                "global": self.global_costmap_topic,
                "local": self.local_costmap_topic,
            },
            "costmaps": {
                "global": self.costmaps.get("global", {
                    "status": "no_data",
                    "topic": self.global_costmap_topic,
                    "data": [],
                }),
                "local": self.costmaps.get("local", {
                    "status": "no_data",
                    "topic": self.local_costmap_topic,
                    "data": [],
                }),
            },
        }
        self.write_json(self.costmap_json_path, payload)

    def on_costmap(self, name, msg):
        self.costmaps[name] = self.serialize_costmap(name, msg)
        self.write_costmaps_snapshot()

    def on_scan(self, msg):
        count = len(msg.ranges)
        if count == 0:
            return

        stride = max(1, math.ceil(count / self.max_scan_points))
        scan_to_base = self.lookup_transform(self.base_frame, msg.header.frame_id)
        points = []

        for index in range(0, count, stride):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if self.max_scan_range_m > 0 and distance > self.max_scan_range_m:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if scan_to_base is not None:
                point = transform_xy(x, y, scan_to_base)
                x = point["x"]
                y = point["y"]

            points.append({
                "x": round(x, 3),
                "y": round(y, 3),
            })

        payload = {
            "schema": "tb3_web_scan.v1",
            "status": "ok",
            "stamp_unix": time.time(),
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "points_frame": self.base_frame,
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "points": points,
        }
        self.write_json(self.scan_json_path, payload)

    def write_pose(self):
        source_frame = self.fixed_frame
        transform = self.lookup_transform(source_frame, self.base_frame)
        if transform is None and self.fallback_fixed_frame and self.fallback_fixed_frame != self.fixed_frame:
            source_frame = self.fallback_fixed_frame
            transform = self.lookup_transform(source_frame, self.base_frame)
        if transform is None:
            payload = {
                "schema": "tb3_web_pose.v1",
                "status": "no_tf",
                "stamp_unix": time.time(),
                "fixed_frame": self.fixed_frame,
                "fallback_fixed_frame": self.fallback_fixed_frame,
                "base_frame": self.base_frame,
                "pose": None,
            }
            self.write_json(self.pose_json_path, payload)
            return

        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        payload = {
            "schema": "tb3_web_pose.v1",
            "status": "ok",
            "stamp_unix": time.time(),
            "fixed_frame": source_frame,
            "requested_fixed_frame": self.fixed_frame,
            "fallback_fixed_frame": self.fallback_fixed_frame,
            "base_frame": self.base_frame,
            "pose": {
                "x": round(t.x, 4),
                "y": round(t.y, 4),
                "z": round(t.z, 4),
                "yaw": round(yaw, 6),
            },
        }
        self.write_json(self.pose_json_path, payload)

    def poll_initial_pose_command(self):
        command = self.read_initial_pose_command()
        if not isinstance(command, dict):
            return

        seq = command.get("seq") or command.get("stamp_unix")
        if seq is None:
            return

        stamp_unix = command.get("stamp_unix")
        if stamp_unix is not None and self.initial_pose_command_max_age_s > 0:
            try:
                command_age = time.time() - float(stamp_unix)
            except (TypeError, ValueError):
                command_age = 0.0
            if command_age > self.initial_pose_command_max_age_s:
                self.last_initial_pose_seq = seq
                return

        if seq != self.last_initial_pose_seq:
            self.last_initial_pose_seq = seq
            self.pending_initial_pose = command
            self.pending_initial_pose_repeats = self.initial_pose_republish_count
            self.get_logger().info(f"queued initial pose command #{seq}")

        if self.pending_initial_pose is not None and self.pending_initial_pose_repeats > 0:
            if self.publish_initial_pose(self.pending_initial_pose):
                self.pending_initial_pose_repeats -= 1
            else:
                self.pending_initial_pose_repeats = 0

    def publish_initial_pose(self, command):
        pose = command.get("pose")
        if not isinstance(pose, dict):
            pose = command

        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid initial pose command: {exc}")
            return False

        q = quaternion_from_yaw(yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(command.get("frame_id") or self.fixed_frame or "map")
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]

        covariance = [0.0] * 36
        covariance[0] = self.initial_pose_xy_variance
        covariance[7] = self.initial_pose_xy_variance
        covariance[35] = self.initial_pose_yaw_variance
        msg.pose.covariance = covariance
        self.initial_pose_pub.publish(msg)
        return True

    def command_age(self, command):
        stamp_unix = command.get("stamp_unix")
        if stamp_unix is None:
            return 0.0
        try:
            return time.time() - float(stamp_unix)
        except (TypeError, ValueError):
            return 0.0

    def poll_nav_goal_command(self):
        command = self.read_nav_goal_command()
        if not isinstance(command, dict):
            return

        seq = command.get("seq") or command.get("stamp_unix")
        if seq is None:
            return

        if seq != self.last_nav_goal_seq:
            self.last_nav_goal_seq = seq
            self.pending_nav_goal = command
            self.write_nav_goal_status("queued", command)
            self.get_logger().info(f"queued nav goal command #{seq}")

        if self.pending_nav_goal is None or self.nav_goal_in_flight:
            return

        if self.nav_goal_command_max_age_s > 0 and self.command_age(self.pending_nav_goal) > self.nav_goal_command_max_age_s:
            self.write_nav_goal_status("expired", self.pending_nav_goal)
            self.pending_nav_goal = None
            return

        if not self.nav_client.server_is_ready():
            self.write_nav_goal_status("waiting_for_server", self.pending_nav_goal)
            return

        self.send_nav_goal(self.pending_nav_goal)
        self.pending_nav_goal = None

    def send_nav_goal(self, command):
        pose = command.get("pose")
        if not isinstance(pose, dict):
            pose = command

        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            self.write_nav_goal_status("invalid", command, error=str(exc))
            self.get_logger().warning(f"invalid nav goal command: {exc}")
            return

        q = quaternion_from_yaw(yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = str(command.get("frame_id") or self.fixed_frame or "map")
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = q["z"]
        goal_msg.pose.pose.orientation.w = q["w"]

        self.nav_goal_in_flight = True
        self.active_nav_goal_seq = command.get("seq")
        self.active_nav_goal_command = command
        self.write_nav_goal_status("sending", command)
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.on_nav_feedback)
        future.add_done_callback(self.on_nav_goal_response)

    def on_nav_goal_response(self, future):
        command = self.active_nav_goal_command
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.nav_goal_in_flight = False
            self.write_nav_goal_status("send_failed", command, error=str(exc))
            self.get_logger().warning(f"failed to send nav goal: {exc}")
            return

        if not goal_handle.accepted:
            self.nav_goal_in_flight = False
            self.write_nav_goal_status("rejected", command)
            self.get_logger().warning("nav goal rejected")
            return

        self.write_nav_goal_status("accepted", command)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_feedback(self, feedback_msg):
        command = self.active_nav_goal_command
        feedback = feedback_msg.feedback
        current_pose = getattr(feedback, "current_pose", None)
        current = None
        if current_pose is not None:
            current = {
                "frame_id": current_pose.header.frame_id,
                "x": round(float(current_pose.pose.position.x), 4),
                "y": round(float(current_pose.pose.position.y), 4),
                "yaw": round(yaw_from_quaternion(current_pose.pose.orientation), 6),
            }
        self.write_nav_goal_status(
            "executing",
            command,
            feedback={
                "distance_remaining": round(float(getattr(feedback, "distance_remaining", 0.0)), 4),
                "navigation_time_s": self.duration_to_seconds(getattr(feedback, "navigation_time", None)),
                "estimated_time_remaining_s": self.duration_to_seconds(getattr(feedback, "estimated_time_remaining", None)),
                "number_of_recoveries": int(getattr(feedback, "number_of_recoveries", 0)),
                "current_pose": current,
            },
        )

    def on_nav_result(self, future):
        command = self.active_nav_goal_command
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.write_nav_goal_status("result_failed", command, error=str(exc))
            status = GoalStatus.STATUS_UNKNOWN
        else:
            self.write_nav_goal_status(
                self.goal_status_label(status),
                command,
                result_status=status,
            )

        self.nav_goal_in_flight = False
        self.active_nav_goal_seq = None
        self.active_nav_goal_command = None


def main(args=None):
    rclpy.init(args=args)
    node = WebRvizState()
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
