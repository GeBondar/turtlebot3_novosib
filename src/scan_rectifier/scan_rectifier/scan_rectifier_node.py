import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ScanRectifier(Node):
    """Apply a calibrated angular remap and republish a uniform LaserScan."""

    def __init__(self) -> None:
        super().__init__('scan_rectifier')

        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('center_deg', -90.25911842021618)
        self.declare_parameter('a1', -0.27114187951999213)
        self.declare_parameter('a2', -0.0783772704666994)
        self.declare_parameter('a3', -0.029267482199569955)
        self.declare_parameter('fill_small_gaps', True)
        self.declare_parameter('max_fill_gap', 2)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.center = math.radians(float(self.get_parameter('center_deg').value))
        self.a1 = float(self.get_parameter('a1').value)
        self.a2 = float(self.get_parameter('a2').value)
        self.a3 = float(self.get_parameter('a3').value)
        self.fill_small_gaps = bool(self.get_parameter('fill_small_gaps').value)
        self.max_fill_gap = int(self.get_parameter('max_fill_gap').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(LaserScan, self.output_topic, qos)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._on_scan,
            qos,
        )

        self.get_logger().info(
            'rectifying %s -> %s with center=%.3f deg, a1=%.6f, a2=%.6f, a3=%.6f'
            % (
                self.input_topic,
                self.output_topic,
                math.degrees(self.center),
                self.a1,
                self.a2,
                self.a3,
            )
        )

    def _correct_angle(self, angle: float) -> float:
        delta = _wrap_pi(angle - self.center)
        return angle + (
            self.a1 * math.sin(delta)
            + self.a2 * math.sin(2.0 * delta)
            + self.a3 * math.sin(3.0 * delta)
        )

    def _fill_gaps(self, ranges: List[float]) -> None:
        if not self.fill_small_gaps or self.max_fill_gap <= 0:
            return

        n = len(ranges)
        finite = [math.isfinite(value) for value in ranges]
        i = 0
        while i < n:
            if finite[i]:
                i += 1
                continue

            start = i
            while i < n and not finite[i]:
                i += 1
            end = i
            gap = end - start
            if gap > self.max_fill_gap:
                continue

            left = start - 1
            right = end
            if left < 0 or right >= n or not finite[left] or not finite[right]:
                continue

            left_value = ranges[left]
            right_value = ranges[right]
            for offset, index in enumerate(range(start, end), start=1):
                alpha = offset / (gap + 1)
                ranges[index] = left_value * (1.0 - alpha) + right_value * alpha

    def _on_scan(self, msg: LaserScan) -> None:
        count = len(msg.ranges)
        if count < 2 or msg.angle_increment == 0.0:
            self.publisher.publish(msg)
            return

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [math.inf] * count
        out.intensities = [0.0] * count if msg.intensities else []

        angle_span = msg.angle_increment * (count - 1)
        output_min = msg.angle_min

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                continue

            input_angle = msg.angle_min + index * msg.angle_increment
            corrected = self._correct_angle(input_angle)
            corrected = output_min + ((_wrap_pi(corrected - output_min) + 2.0 * math.pi) % (2.0 * math.pi))
            if corrected > output_min + angle_span:
                corrected -= 2.0 * math.pi

            output_index = int(round((corrected - output_min) / msg.angle_increment))
            if output_index < 0:
                output_index += count
            elif output_index >= count:
                output_index -= count
            if output_index < 0 or output_index >= count:
                continue

            current = out.ranges[output_index]
            if not math.isfinite(current) or distance < current:
                out.ranges[output_index] = distance
                if msg.intensities:
                    out.intensities[output_index] = msg.intensities[index]

        self._fill_gaps(out.ranges)
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRectifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
