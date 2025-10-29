#!/usr/bin/env python3
import math
import sys
import time
import threading
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class AnglePlanner(Node):
    """
    Publishes target joint angles as a string on /serial_command.

    Message format is configurable via parameters:
      - message_prefix (str): prefix before the list, default ""
      - message_suffix (str): suffix after the list, default "\n"
      - separator (str): separator between values, default ","

    The payload will look like: f"{prefix}{a0}{sep}{a1}{sep}...{suffix}"
    where ai are angles in degrees.

    Trajectory per joint: angle = amplitude_deg * sin(2*pi*freq*t + phase)
    Phases are evenly spaced across joints.
    """

    def __init__(self):
        super().__init__('angle_planner')
        # Parameters
        self.declare_parameter('joints', 2)
        self.declare_parameter('amplitude_deg', 2.0)
        self.declare_parameter('freq_hz', 0.5)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('message_prefix', '')
        self.declare_parameter('message_suffix', '\n')
        self.declare_parameter('separator', ',')
        self.declare_parameter('interactive', False)
        self.declare_parameter('publish_string_command', False)
        # Read parameters
        self.joints = int(self.get_parameter('joints').get_parameter_value().integer_value)
        self.amplitude_deg = float(self.get_parameter('amplitude_deg').get_parameter_value().double_value)
        self.freq_hz = float(self.get_parameter('freq_hz').get_parameter_value().double_value)
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        self.prefix = self.get_parameter('message_prefix').get_parameter_value().string_value
        self.suffix = self.get_parameter('message_suffix').get_parameter_value().string_value
        self.sep = self.get_parameter('separator').get_parameter_value().string_value
        self.interactive = bool(self.get_parameter('interactive').get_parameter_value().bool_value)
        self.publish_string_command = bool(self.get_parameter('publish_string_command').get_parameter_value().bool_value)

        if self.joints <= 0:
            self.get_logger().warn('Parameter joints <= 0, forcing to 1')
            self.joints = 1
        if self.rate_hz <= 0.0:
            self.get_logger().warn('Parameter rate_hz <= 0, forcing to 50.0')
            self.rate_hz = 50.0

        # 可选：仅在需要 ASCII 直通时发布字符串（默认关闭）
        if self.publish_string_command:
            self.publisher_ = self.create_publisher(String, 'serial_command', 10)
        # 数值角度目标（度）——让串口节点填充到二进制帧
        self.pub_angle_target = self.create_publisher(Float32, 'angle_target', 10)
        self.t0 = time.time()

        # Precompute phases evenly spaced
        self.phases = [2.0 * math.pi * i / max(1, self.joints) for i in range(self.joints)]

        period = 1.0 / self.rate_hz
        self.timer_ = self.create_timer(period, self.publish_command)
        self.get_logger().info(
            f'Starting AnglePlanner: joints={self.joints}, amplitude_deg={self.amplitude_deg}, '
            f'freq_hz={self.freq_hz}, rate_hz={self.rate_hz}, interactive={self.interactive}, '
            f'publish_string_command={self.publish_string_command}')

        # Optional interactive mode (read lines from stdin and send directly)
        if self.interactive and self.publish_string_command:
            self._input_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self._input_thread.start()

    def publish_command(self):
        t = time.time() - self.t0
        angles_deg = [
            self.amplitude_deg * math.sin(2.0 * math.pi * self.freq_hz * t + ph)
            for ph in self.phases
        ]
        if self.publish_string_command:
            payload = self.prefix + self.sep.join(f'{a:.2f}' for a in angles_deg) + self.suffix
            msg = String()
            msg.data = payload
            self.publisher_.publish(msg)
        # 同步发布第一个关节的角度为 angle_target（度）
        if angles_deg:
            self.pub_angle_target.publish(Float32(data=float(angles_deg[0])))
        # 仅在发布字符串时打印调试
        if self.publish_string_command:
            self.get_logger().debug(f'Published command: {msg.data!r}')

    def _stdin_loop(self):
        self.get_logger().info('Interactive mode: type a line to send; Ctrl+C to quit.')
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05)
                    continue
                # strip only trailing newline, keep other spaces
                line = line.rstrip('\n')
                if line == '':
                    continue
                payload = f"{self.prefix}{line}{self.suffix}"
                msg = String()
                msg.data = payload
                if self.publish_string_command:
                    self.publisher_.publish(msg)
                self.get_logger().info(f'Sent (interactive): {msg.data!r}')
            except Exception as e:
                self.get_logger().error(f'Interactive input error: {e}')
                time.sleep(0.2)


def main():
    rclpy.init()
    node = AnglePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
