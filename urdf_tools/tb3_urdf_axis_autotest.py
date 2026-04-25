#!/usr/bin/env python3
"""Autotest for detecting TurtleBot3 URDF axis/sign mistakes on a real robot."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import math
import os
import shlex
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


def bootstrap_ros_env_if_needed() -> None:
    if os.environ.get("TB3_URDF_AUTOTEST_BOOTSTRAPPED") == "1":
        return
    if importlib.util.find_spec("rclpy") is not None:
        return

    script_path = os.path.abspath(__file__)
    args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    command = (
        "set -e; "
        "for candidate in /opt/ros/*/setup.bash; do "
        "  if [ -f \"$candidate\" ]; then "
        "    . \"$candidate\"; "
        "    break; "
        "  fi; "
        "done; "
        "if [ -f /home/ubuntu/turtlebot3_ws/install/setup.bash ]; then "
        "  . /home/ubuntu/turtlebot3_ws/install/setup.bash; "
        "fi; "
        "export TB3_URDF_AUTOTEST_BOOTSTRAPPED=1; "
        f"exec python3 {shlex.quote(script_path)} {args}"
    )
    os.execvp("bash", ["bash", "-lc", command])


bootstrap_ros_env_if_needed()

if importlib.util.find_spec("rclpy") is None:
    print(
        "rclpy is not available. Make sure ROS 2 is installed and source "
        "/opt/ros/<distro>/setup.bash plus your workspace install/setup.bash.",
        file=sys.stderr,
    )
    raise SystemExit(1)


import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan


EPS = 1.0e-6


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def sign_or_none(value: float, threshold: float = 1.0e-4) -> Optional[int]:
    if value > threshold:
        return 1
    if value < -threshold:
        return -1
    return None


def circular_mean(angles: Sequence[float]) -> float:
    sin_sum = sum(math.sin(angle) for angle in angles)
    cos_sum = sum(math.cos(angle) for angle in angles)
    if abs(sin_sum) < EPS and abs(cos_sum) < EPS:
        return 0.0
    return math.atan2(sin_sum, cos_sum)


def quaternion_to_yaw(msg) -> float:
    siny_cosp = 2.0 * (msg.w * msg.z + msg.x * msg.y)
    cosy_cosp = 1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_xyz(text: str) -> Tuple[float, float, float]:
    x, y, z = (float(chunk) for chunk in text.split())
    return x, y, z


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_vec_mul(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(
        matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2]
        for row in range(3)
    )


def transpose(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def snap_axis_component(value: float) -> int:
    if value >= 0.5:
        return 1
    if value <= -0.5:
        return -1
    return 0


def axis_vector_to_text(vector: Sequence[float]) -> str:
    snapped = [snap_axis_component(value) for value in vector]
    return f"{snapped[0]} {snapped[1]} {snapped[2]}"


def snap_angle_for_report(angle: float) -> float:
    candidates = [
        0.0,
        math.pi / 2.0,
        -math.pi / 2.0,
        math.pi,
        -math.pi,
    ]
    for candidate in candidates:
        if abs(normalize_angle(angle - candidate)) < 0.08:
            return candidate
    return angle


def format_rpy_triplet(roll: float, pitch: float, yaw: float) -> str:
    return f"{roll:.4f} {pitch:.4f} {yaw:.4f}"


def format_observation(observation: Optional[FrontObservation]) -> str:
    if observation is None:
        return "not available"
    return (
        f"distance={observation.distance:.3f} m, "
        f"angle={observation.angle:.3f} rad ({math.degrees(observation.angle):.1f} deg)"
    )


@dataclass
class JointConfig:
    name: str
    origin_xyz_text: str
    origin_rpy_text: str
    axis_text: str
    origin_rpy: Tuple[float, float, float]
    axis_local: Tuple[float, float, float]


@dataclass
class ScanConfig:
    origin_xyz_text: str
    origin_rpy_text: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]


@dataclass
class FrontObservation:
    distance: float
    angle: float
    count: int


@dataclass
class Sample:
    front: Optional[FrontObservation]
    nearest: Optional[FrontObservation]
    joint_positions: Dict[str, float]
    odom_yaw: float


@dataclass
class WheelTestResult:
    wheel_name: str
    start_angle: float
    end_angle: float
    angle_shift: float
    start_distance: float
    end_distance: float
    joint_delta: float
    odom_yaw_shift: float
    robot_yaw_direction: Optional[str]
    wheel_motion_direction: Optional[str]
    expected_axis_local_text: Optional[str]
    current_axis_local_text: str
    suggestion_needed: bool


class Tb3UrdfAxisAutotest(Node):
    def __init__(
        self,
        scan_topic: str,
        joint_states_topic: str,
        odom_topic: str,
        cmd_vel_topic: str,
        cmd_vel_mode: str,
        front_distance_threshold: float,
        cluster_distance_delta: float,
    ) -> None:
        super().__init__("tb3_urdf_axis_autotest")
        self.scan_topic = scan_topic
        self.joint_states_topic = joint_states_topic
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.cmd_vel_mode_request = cmd_vel_mode
        self.front_distance_threshold = front_distance_threshold
        self.cluster_distance_delta = cluster_distance_delta

        self.latest_scan: Optional[LaserScan] = None
        self.latest_joint_states: Optional[JointState] = None
        self.latest_odom: Optional[Odometry] = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_states_callback,
            qos_profile_sensor_data,
        )
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            odom_qos,
        )

        self._cmd_vel_mode_resolved: Optional[str] = None
        self._cmd_vel_publisher = None

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _joint_states_callback(self, msg: JointState) -> None:
        self.latest_joint_states = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def wait_for_topics(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_scan and self.latest_joint_states and self.latest_odom:
                return True
        return False

    def resolve_cmd_vel_mode(self) -> str:
        if self._cmd_vel_mode_resolved:
            return self._cmd_vel_mode_resolved
        if self.cmd_vel_mode_request in {"twist", "twist_stamped"}:
            self._cmd_vel_mode_resolved = self.cmd_vel_mode_request
            return self._cmd_vel_mode_resolved

        topic_types: List[str] = []
        for topic_name, types in self.get_topic_names_and_types():
            if topic_name == self.cmd_vel_topic:
                topic_types = types
                break

        if "geometry_msgs/msg/TwistStamped" in topic_types:
            self._cmd_vel_mode_resolved = "twist_stamped"
        else:
            self._cmd_vel_mode_resolved = "twist"
        return self._cmd_vel_mode_resolved

    def ensure_cmd_vel_publisher(self) -> None:
        if self._cmd_vel_publisher is not None:
            return
        mode = self.resolve_cmd_vel_mode()
        qos = QoSProfile(depth=10)
        if mode == "twist_stamped":
            self._cmd_vel_publisher = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)
        else:
            self._cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.get_logger().info(f"Publishing {mode} on {self.cmd_vel_topic}")

    def publish_velocity(self, linear_x: float, angular_z: float) -> None:
        self.ensure_cmd_vel_publisher()
        if self._cmd_vel_mode_resolved == "twist_stamped":
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = linear_x
            msg.twist.angular.z = angular_z
        else:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
        self._cmd_vel_publisher.publish(msg)

    def stop_robot(self, repeats: int = 6) -> None:
        if self._cmd_vel_publisher is None:
            return
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def hold_still(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.publish_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def command_for(self, linear_x: float, angular_z: float, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.publish_velocity(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop_robot()

    def joint_positions(self) -> Dict[str, float]:
        if not self.latest_joint_states:
            return {}
        return {
            name: position
            for name, position in zip(self.latest_joint_states.name, self.latest_joint_states.position)
        }

    def extract_observation(self, max_distance: Optional[float]) -> Optional[FrontObservation]:
        scan = self.latest_scan
        if scan is None:
            return None

        valid: List[Tuple[float, float, int]] = []
        upper_bound = scan.range_max if max_distance is None else min(scan.range_max, max_distance)
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > upper_bound:
                continue
            angle = normalize_angle(scan.angle_min + index * scan.angle_increment)
            valid.append((distance, angle, index))

        if not valid:
            return None

        min_distance = min(item[0] for item in valid)
        cluster_angles = [
            angle
            for distance, angle, _ in valid
            if distance <= min_distance + self.cluster_distance_delta
        ]
        return FrontObservation(
            distance=min_distance,
            angle=circular_mean(cluster_angles),
            count=len(cluster_angles),
        )

    def extract_front_observation(self) -> Optional[FrontObservation]:
        return self.extract_observation(self.front_distance_threshold)

    def extract_nearest_observation(self) -> Optional[FrontObservation]:
        return self.extract_observation(None)

    def sample_state(self, observation_timeout_sec: float) -> Sample:
        deadline = time.monotonic() + observation_timeout_sec
        front = None
        nearest = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            front = self.extract_front_observation()
            nearest = self.extract_nearest_observation()
            if front is not None or nearest is not None:
                break
        odom_yaw = 0.0
        if self.latest_odom:
            odom_yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        return Sample(
            front=front,
            nearest=nearest,
            joint_positions=self.joint_positions(),
            odom_yaw=odom_yaw,
        )


def parse_urdf(urdf_path: Path) -> Tuple[ET.ElementTree, JointConfig, JointConfig, ScanConfig]:
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()

    def find_joint_config(joint_name: str) -> JointConfig:
        for joint in root.findall("joint"):
            if joint.get("name", "").endswith(joint_name):
                origin = joint.find("origin")
                axis = joint.find("axis")
                if origin is None or axis is None:
                    raise RuntimeError(f"Joint {joint_name} is missing origin or axis")
                origin_rpy_text = origin.get("rpy", "0 0 0")
                axis_text = axis.get("xyz", "0 0 1")
                return JointConfig(
                    name=joint_name,
                    origin_xyz_text=origin.get("xyz", "0 0 0"),
                    origin_rpy_text=origin_rpy_text,
                    axis_text=axis_text,
                    origin_rpy=parse_xyz(origin_rpy_text),
                    axis_local=parse_xyz(axis_text),
                )
        raise RuntimeError(f"Joint {joint_name} was not found in {urdf_path}")

    def find_scan_config() -> ScanConfig:
        for joint in root.findall("joint"):
            if joint.get("name", "").endswith("scan_joint"):
                origin = joint.find("origin")
                if origin is None:
                    raise RuntimeError("scan_joint is missing origin")
                origin_xyz_text = origin.get("xyz", "0 0 0")
                origin_rpy_text = origin.get("rpy", "0 0 0")
                return ScanConfig(
                    origin_xyz_text=origin_xyz_text,
                    origin_rpy_text=origin_rpy_text,
                    origin_xyz=parse_xyz(origin_xyz_text),
                    origin_rpy=parse_xyz(origin_rpy_text),
                )
        raise RuntimeError(f"scan_joint was not found in {urdf_path}")

    return (
        tree,
        find_joint_config("wheel_left_joint"),
        find_joint_config("wheel_right_joint"),
        find_scan_config(),
    )


def build_expected_axis_local_text(joint_config: JointConfig, wheel_motion_direction: Optional[str], joint_delta: float) -> Optional[str]:
    motion_sign = {"forward": 1, "backward": -1}.get(wheel_motion_direction)
    delta_sign = sign_or_none(joint_delta, threshold=1.0e-3)
    if motion_sign is None or delta_sign is None:
        return None

    expected_parent_y_sign = motion_sign * delta_sign
    expected_parent_axis = (0.0, float(expected_parent_y_sign), 0.0)
    rotation = rotation_matrix_from_rpy(*joint_config.origin_rpy)
    expected_local_axis = mat_vec_mul(transpose(rotation), expected_parent_axis)
    return axis_vector_to_text(expected_local_axis)


def joint_position_by_suffix(joint_positions: Dict[str, float], joint_name: str) -> float:
    if joint_name in joint_positions:
        return joint_positions[joint_name]
    for candidate_name, value in joint_positions.items():
        if candidate_name.endswith(joint_name):
            return value
    return 0.0


def robot_yaw_direction_from_front_shift(angle_shift: float) -> Optional[str]:
    shift_sign = sign_or_none(angle_shift, threshold=0.04)
    if shift_sign is None:
        return None
    return "ccw" if shift_sign < 0 else "cw"


def wheel_motion_from_yaw_direction(wheel_name: str, yaw_direction: Optional[str]) -> Optional[str]:
    if yaw_direction is None:
        return None
    if wheel_name == "wheel_right_joint":
        return "forward" if yaw_direction == "ccw" else "backward"
    if wheel_name == "wheel_left_joint":
        return "forward" if yaw_direction == "cw" else "backward"
    return None


def run_wheel_test(
    node: Tb3UrdfAxisAutotest,
    joint_config: JointConfig,
    wheel_linear: float,
    wheel_angular: float,
    move_duration: float,
    settle_duration: float,
    observation_timeout: float,
) -> WheelTestResult:
    node.get_logger().info(f"Preparing {joint_config.name} test")
    before = node.sample_state(observation_timeout)
    if before.front is None:
        raise RuntimeError(
            "Nearest object was not found within the configured threshold before wheel test. "
            "Place an object within 0.40 m in front of the robot."
        )
    node.hold_still(0.8)
    before = node.sample_state(observation_timeout)
    if before.front is None:
        raise RuntimeError("Lost front object before wheel test")

    node.get_logger().info(
        f"{joint_config.name}: start angle={before.front.angle:.3f} rad, "
        f"distance={before.front.distance:.3f} m"
    )
    node.command_for(wheel_linear, wheel_angular, move_duration)
    node.hold_still(settle_duration)
    after = node.sample_state(observation_timeout)
    if after.front is None:
        raise RuntimeError("Lost front object after wheel test")

    tested_joint_delta = (
        joint_position_by_suffix(after.joint_positions, joint_config.name)
        - joint_position_by_suffix(before.joint_positions, joint_config.name)
    )
    angle_shift = normalize_angle(after.front.angle - before.front.angle)
    odom_yaw_shift = normalize_angle(after.odom_yaw - before.odom_yaw)
    robot_yaw_direction = robot_yaw_direction_from_front_shift(angle_shift)
    wheel_motion_direction = wheel_motion_from_yaw_direction(joint_config.name, robot_yaw_direction)
    expected_axis_local_text = build_expected_axis_local_text(joint_config, wheel_motion_direction, tested_joint_delta)

    return WheelTestResult(
        wheel_name=joint_config.name,
        start_angle=before.front.angle,
        end_angle=after.front.angle,
        angle_shift=angle_shift,
        start_distance=before.front.distance,
        end_distance=after.front.distance,
        joint_delta=tested_joint_delta,
        odom_yaw_shift=odom_yaw_shift,
        robot_yaw_direction=robot_yaw_direction,
        wheel_motion_direction=wheel_motion_direction,
        expected_axis_local_text=expected_axis_local_text,
        current_axis_local_text=joint_config.axis_text,
        suggestion_needed=(
            expected_axis_local_text is not None
            and expected_axis_local_text != joint_config.axis_text
        ),
    )


def replace_line_in_joint_block(
    lines: List[str],
    joint_name: str,
    marker: str,
    new_line: str,
) -> Tuple[int, str, str]:
    inside_joint = False
    for index, line in enumerate(lines):
        if f'name="${{namespace}}{joint_name}"' in line or f'name="{joint_name}"' in line:
            inside_joint = True
            continue
        if inside_joint and "</joint>" in line:
            break
        if inside_joint and marker in line:
            old_line = line.rstrip("\n")
            lines[index] = new_line + "\n"
            return index + 1, old_line, new_line
    raise RuntimeError(f"Could not find {marker!r} inside {joint_name}")


def build_patch_text(
    urdf_path: Path,
    left_result: WheelTestResult,
    right_result: WheelTestResult,
    left_joint: JointConfig,
    right_joint: JointConfig,
    scan_config: ScanConfig,
    scan_yaw_suggestion: Optional[float],
) -> Tuple[str, List[str]]:
    original_lines = urdf_path.read_text(encoding="utf-8").splitlines(keepends=True)
    updated_lines = copy.deepcopy(original_lines)
    patch_lines = [f"--- {urdf_path}", f"+++ {urdf_path}.suggested"]
    human_lines: List[str] = []

    if left_result.expected_axis_local_text and left_result.expected_axis_local_text != left_joint.axis_text:
        new_line = f'    <axis xyz="{left_result.expected_axis_local_text}"/>'
        line_no, old_line, replacement = replace_line_in_joint_block(
            updated_lines,
            left_joint.name,
            "<axis ",
            new_line,
        )
        patch_lines.extend([f"@@ line {line_no} @@", f"-{old_line}", f"+{replacement}"])
        human_lines.append(
            f"{left_joint.name}: line {line_no}: {old_line.strip()} -> {replacement.strip()}"
        )

    if right_result.expected_axis_local_text and right_result.expected_axis_local_text != right_joint.axis_text:
        new_line = f'    <axis xyz="{right_result.expected_axis_local_text}"/>'
        line_no, old_line, replacement = replace_line_in_joint_block(
            updated_lines,
            right_joint.name,
            "<axis ",
            new_line,
        )
        patch_lines.extend([f"@@ line {line_no} @@", f"-{old_line}", f"+{replacement}"])
        human_lines.append(
            f"{right_joint.name}: line {line_no}: {old_line.strip()} -> {replacement.strip()}"
        )

    if scan_yaw_suggestion is not None:
        current_roll, current_pitch, _ = scan_config.origin_rpy
        new_rpy_text = format_rpy_triplet(current_roll, current_pitch, scan_yaw_suggestion)
        if new_rpy_text != scan_config.origin_rpy_text:
            new_line = (
                f'    <origin xyz="{scan_config.origin_xyz_text}" '
                f'rpy="{new_rpy_text}"/>'
            )
            line_no, old_line, replacement = replace_line_in_joint_block(
                updated_lines,
                "scan_joint",
                "<origin ",
                new_line,
            )
            patch_lines.extend([f"@@ line {line_no} @@", f"-{old_line}", f"+{replacement}"])
            human_lines.append(
                f"scan_joint: line {line_no}: {old_line.strip()} -> {replacement.strip()}"
            )

    if len(patch_lines) == 2:
        patch_lines.append("# No URDF axis corrections were inferred from this run.")
    return "\n".join(patch_lines) + "\n", human_lines


def build_report(
    urdf_path: Path,
    left_result: WheelTestResult,
    right_result: WheelTestResult,
    initial_front: FrontObservation,
    straight_before: FrontObservation,
    straight_after: FrontObservation,
    straight_distance_delta: float,
    straight_interpretation: str,
    scan_yaw_suggestion: Optional[float],
    human_patch_lines: Sequence[str],
    patch_path: Path,
    warnings: Sequence[str],
) -> str:
    report_lines = [
        "TurtleBot3 URDF axis autotest report",
        f"URDF: {urdf_path}",
        "",
        "Initial front-object observation:",
        (
            f"  nearest object distance={initial_front.distance:.3f} m, "
            f"angle={initial_front.angle:.3f} rad "
            f"({math.degrees(initial_front.angle):.1f} deg)"
        ),
        "",
        "Wheel tests:",
        (
            f"  right wheel: angle shift={right_result.angle_shift:.3f} rad, "
            f"joint delta={right_result.joint_delta:.4f} rad, "
            f"odom yaw shift={right_result.odom_yaw_shift:.3f} rad, "
            f"physical motion={right_result.wheel_motion_direction or 'inconclusive'}, "
            f"suggested axis={right_result.expected_axis_local_text or 'inconclusive'}"
        ),
        (
            f"  left wheel: angle shift={left_result.angle_shift:.3f} rad, "
            f"joint delta={left_result.joint_delta:.4f} rad, "
            f"odom yaw shift={left_result.odom_yaw_shift:.3f} rad, "
            f"physical motion={left_result.wheel_motion_direction or 'inconclusive'}, "
            f"suggested axis={left_result.expected_axis_local_text or 'inconclusive'}"
        ),
        "",
        "Straight cmd_vel test:",
        (
            f"  front distance before={straight_before.distance:.3f} m, "
            f"after={straight_after.distance:.3f} m, "
            f"delta={straight_distance_delta:+.3f} m"
        ),
        f"  interpretation: {straight_interpretation}",
        "",
        "scan_joint suggestion:",
    ]
    if scan_yaw_suggestion is None:
        report_lines.append("  no yaw correction was inferred")
    else:
        report_lines.append(
            f"  suggested yaw={scan_yaw_suggestion:.4f} rad "
            f"({math.degrees(scan_yaw_suggestion):.1f} deg)"
        )

    report_lines.extend(["", "Suggested URDF edits:"])
    if human_patch_lines:
        for line in human_patch_lines:
            report_lines.append(f"  {line}")
    else:
        report_lines.append("  no changes suggested")

    report_lines.extend(["", f"Patch file: {patch_path}"])
    if warnings:
        report_lines.extend(["", "Warnings:"])
        for warning in warnings:
            report_lines.append(f"  {warning}")
    return "\n".join(report_lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a physical TurtleBot3 test sequence and suggest URDF axis corrections "
            "for wheels and scan_joint orientation."
        )
    )
    parser.add_argument(
        "--urdf-path",
        default="/home/ubuntu/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf",
        help="Path to turtlebot3_burger.urdf on the robot",
    )
    parser.add_argument(
        "--report-path",
        default="",
        help="Optional path for the text report. Default: <urdf>.autotest_report.txt",
    )
    parser.add_argument(
        "--patch-path",
        default="",
        help="Optional path for the suggested patch. Default: <urdf>.autotest_patch.diff",
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument(
        "--cmd-vel-mode",
        choices=["auto", "twist", "twist_stamped"],
        default="auto",
        help="How to publish cmd_vel. 'auto' inspects the ROS graph.",
    )
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--observation-timeout", type=float, default=4.0)
    parser.add_argument("--front-distance-threshold", type=float, default=0.40)
    parser.add_argument("--cluster-distance-delta", type=float, default=0.03)
    parser.add_argument("--wheel-speed", type=float, default=0.08)
    parser.add_argument("--wheel-separation", type=float, default=0.169)
    parser.add_argument("--wheel-test-duration", type=float, default=1.5)
    parser.add_argument("--straight-speed", type=float, default=0.05)
    parser.add_argument("--straight-duration", type=float, default=1.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    args = parser.parse_args()

    urdf_path = Path(args.urdf_path)
    if not urdf_path.exists():
        print(f"URDF file was not found: {urdf_path}", file=sys.stderr)
        return 2

    report_path = Path(args.report_path) if args.report_path else Path(str(urdf_path) + ".autotest_report.txt")
    patch_path = Path(args.patch_path) if args.patch_path else Path(str(urdf_path) + ".autotest_patch.diff")

    _, left_joint, right_joint, scan_config = parse_urdf(urdf_path)
    warnings: List[str] = []

    rclpy.init()
    node = Tb3UrdfAxisAutotest(
        scan_topic=args.scan_topic,
        joint_states_topic=args.joint_states_topic,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        cmd_vel_mode=args.cmd_vel_mode,
        front_distance_threshold=args.front_distance_threshold,
        cluster_distance_delta=args.cluster_distance_delta,
    )

    try:
        node.get_logger().info("Waiting for /scan, /joint_states and /odom")
        if not node.wait_for_topics(args.startup_timeout):
            raise RuntimeError(
                "Required ROS topics did not appear in time. "
                "Make sure TurtleBot3 bringup is already running on the robot."
            )

        node.ensure_cmd_vel_publisher()
        node.hold_still(1.0)
        initial_state = node.sample_state(args.observation_timeout)
        if initial_state.front is None:
            nearest_text = format_observation(initial_state.nearest)
            raise RuntimeError(
                "No object was detected within the configured front threshold "
                f"({args.front_distance_threshold:.2f} m). "
                f"Nearest visible object: {nearest_text}. "
                "Place a clear obstacle in the lidar plane closer to the robot, "
                "or rerun with a larger --front-distance-threshold."
            )

        node.get_logger().info(
            f"Initial object: angle={initial_state.front.angle:.3f} rad, "
            f"distance={initial_state.front.distance:.3f} m"
        )

        right_linear = args.wheel_speed / 2.0
        right_angular = args.wheel_speed / args.wheel_separation
        left_linear = args.wheel_speed / 2.0
        left_angular = -args.wheel_speed / args.wheel_separation

        right_result = run_wheel_test(
            node=node,
            joint_config=right_joint,
            wheel_linear=right_linear,
            wheel_angular=right_angular,
            move_duration=args.wheel_test_duration,
            settle_duration=args.settle_duration,
            observation_timeout=args.observation_timeout,
        )

        left_result = run_wheel_test(
            node=node,
            joint_config=left_joint,
            wheel_linear=left_linear,
            wheel_angular=left_angular,
            move_duration=args.wheel_test_duration,
            settle_duration=args.settle_duration,
            observation_timeout=args.observation_timeout,
        )

        straight_before_state = node.sample_state(args.observation_timeout)
        if straight_before_state.front is None:
            raise RuntimeError("Lost front object before straight cmd_vel test")

        node.get_logger().info("Preparing straight cmd_vel test")
        node.hold_still(0.8)
        straight_before_state = node.sample_state(args.observation_timeout)
        if straight_before_state.front is None:
            raise RuntimeError("Lost front object right before straight cmd_vel test")

        node.command_for(args.straight_speed, 0.0, args.straight_duration)
        node.hold_still(args.settle_duration)
        straight_after_state = node.sample_state(args.observation_timeout)
        if straight_after_state.front is None:
            raise RuntimeError("Lost front object after straight cmd_vel test")

        straight_distance_delta = straight_after_state.front.distance - straight_before_state.front.distance
        if straight_distance_delta < -0.02:
            straight_interpretation = "positive cmd_vel moved toward the physical front object"
        elif straight_distance_delta > 0.02:
            straight_interpretation = (
                "positive cmd_vel moved away from the physical front object. "
                "That usually points to wheel/controller direction issues, not only URDF."
            )
            warnings.append(
                "Positive cmd_vel increased distance to the object placed in front. "
                "Check motor wiring or turtlebot3_node/OpenCR direction settings in addition to URDF."
            )
        else:
            straight_interpretation = "straight motion was too small or too noisy to classify confidently"
            warnings.append(
                "Straight cmd_vel test was inconclusive. Consider increasing object contrast or straight duration slightly."
            )

        current_scan_yaw = scan_config.origin_rpy[2]
        suggested_scan_yaw = normalize_angle(current_scan_yaw + initial_state.front.angle)
        suggested_scan_yaw = snap_angle_for_report(suggested_scan_yaw)
        if abs(normalize_angle(suggested_scan_yaw - current_scan_yaw)) < 0.08:
            suggested_scan_yaw = None

        if right_result.robot_yaw_direction and sign_or_none(right_result.odom_yaw_shift, 0.04):
            odom_dir = "ccw" if right_result.odom_yaw_shift > 0.0 else "cw"
            if odom_dir != right_result.robot_yaw_direction:
                warnings.append(
                    "Right-wheel test: odom yaw direction disagreed with lidar-based physical rotation direction."
                )
        if left_result.robot_yaw_direction and sign_or_none(left_result.odom_yaw_shift, 0.04):
            odom_dir = "ccw" if left_result.odom_yaw_shift > 0.0 else "cw"
            if odom_dir != left_result.robot_yaw_direction:
                warnings.append(
                    "Left-wheel test: odom yaw direction disagreed with lidar-based physical rotation direction."
                )

        patch_text, human_patch_lines = build_patch_text(
            urdf_path=urdf_path,
            left_result=left_result,
            right_result=right_result,
            left_joint=left_joint,
            right_joint=right_joint,
            scan_config=scan_config,
            scan_yaw_suggestion=suggested_scan_yaw,
        )
        report_text = build_report(
            urdf_path=urdf_path,
            left_result=left_result,
            right_result=right_result,
            initial_front=initial_state.front,
            straight_before=straight_before_state.front,
            straight_after=straight_after_state.front,
            straight_distance_delta=straight_distance_delta,
            straight_interpretation=straight_interpretation,
            scan_yaw_suggestion=suggested_scan_yaw,
            human_patch_lines=human_patch_lines,
            patch_path=patch_path,
            warnings=warnings,
        )

        report_path.write_text(report_text, encoding="utf-8")
        patch_path.write_text(patch_text, encoding="utf-8")

        print(report_text)
        print("Suggested patch:")
        print(patch_text)
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user", file=sys.stderr)
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"Autotest failed: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            node.stop_robot()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
