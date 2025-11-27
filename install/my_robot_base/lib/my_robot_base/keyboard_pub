#!/usr/bin/env python3
"""键盘控制器: 发布 motion / mode / control_mode 等话题."""

import sys
import termios
import tty
import select
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


HELP_TEXT = """
================= Keyboard Controls =================
 0 : motion = 0 (停止)
 1 : motion = 1 (站立)
 2 : motion = 2 (行走)
 u/i/o : mode = 0 / 1 / 2
 j/k/l : control_mode = 0 / 1 / 2
 h or ? : 显示帮助
 q or Ctrl+C : 退出
=====================================================
"""


def get_key(timeout: float = 0.02) -> Optional[str]:
	"""非阻塞读取按键."""
	if select.select([sys.stdin], [], [], timeout)[0]:
		return sys.stdin.read(1)
	return None


class KeyboardPublisher(Node):
	"""通过键盘发布 motion/mode/control_mode."""

	def __init__(self):
		super().__init__('keyboard_pub')
		self.motion_pub = self.create_publisher(Int32, 'motion', 10)
		self.publish_mode = self.declare_parameter('publish_mode', False).value # 是否发布 mode 话题
		self.publish_control_mode = self.declare_parameter('publish_control_mode', False).value # 是否发布 control_mode 话题
		self.mode_pub = self.create_publisher(Int32, 'mode', 10) if self.publish_mode else None
		self.control_mode_pub = self.create_publisher(Int32, 'control_mode', 10) if self.publish_control_mode else None

		self.motion = 0
		self.mode = 0
		self.control_mode = 0

		self._publish_all(initial=True)
		self.get_logger().info(HELP_TEXT)
		if not self.publish_mode:
			self.get_logger().info("Mode topic publishing DISABLED via parameter publish_mode=false")
		if not self.publish_control_mode:
			self.get_logger().info("control_mode publishing DISABLED via parameter publish_control_mode=false")
		
		# 创建两个定时器：
		# 1. 键盘轮询 (50ms / 20Hz)
		self.poll_timer = self.create_timer(0.05, self._poll_keyboard)
		# 2. 状态持续发布 (20ms / 50Hz) - 保证下位机能稳定收到
		self.pub_timer = self.create_timer(0.01, self._publish_state_loop)

	def _publish_state_loop(self):
		"""持续发布当前状态，确保不丢包."""
		msg = Int32()
		msg.data = int(self.motion)
		self.motion_pub.publish(msg)
		if self.publish_mode and self.mode_pub:
			msg = Int32()
			msg.data = int(self.mode)
			self.mode_pub.publish(msg)
		if self.publish_control_mode and self.control_mode_pub:
			msg = Int32()
			msg.data = int(self.control_mode)
			self.control_mode_pub.publish(msg)

	def _publish_all(self, initial: bool = False):
		if initial:
			self.get_logger().info(
				f"Initial State: motion={self.motion}, mode={self.mode}, ctl_mode={self.control_mode}"
			)
			self.get_logger().info(
				f"Mode publish={'ON' if self.publish_mode else 'OFF'}, Control publish={'ON' if self.publish_control_mode else 'OFF'}"
			)
		self._publish_int(self.motion_pub, self.motion, 'motion', initial)
		self._publish_int(self.mode_pub, self.mode, 'mode', initial)
		self._publish_int(self.control_mode_pub, self.control_mode, 'control_mode', initial)

	def _publish_int(self, pub, value: int, name: str, initial: bool = False):
		if pub is None:
			if not initial:
				self.get_logger().info(f"{name} 发布已禁用，当前值 {value}")
			return
		msg = Int32()
		msg.data = int(value)
		pub.publish(msg)
		if not initial:
			self.get_logger().info(f"{name} -> {value}")

	def _poll_keyboard(self):
		key = get_key()
		if key is None:
			return
		if key in ('\x03', 'q', 'Q'):
			raise KeyboardInterrupt
		if key in ('h', 'H', '?'):
			self.get_logger().info(HELP_TEXT)
			return

		if key == '0':
			self.motion = 0
			self._publish_int(self.motion_pub, self.motion, 'motion')
		elif key == '1':
			self.motion = 1
			self._publish_int(self.motion_pub, self.motion, 'motion')
		elif key == '2':
			self.motion = 2
			self._publish_int(self.motion_pub, self.motion, 'motion')
		elif key in ('u', 'U'):
			self.mode = 0
			self._publish_int(self.mode_pub, self.mode, 'mode')
		elif key in ('i', 'I'):
			self.mode = 1
			self._publish_int(self.mode_pub, self.mode, 'mode')
		elif key in ('o', 'O'):
			self.mode = 2
			self._publish_int(self.mode_pub, self.mode, 'mode')
		elif key in ('j', 'J'):
			self.control_mode = 0
			self._publish_int(self.control_mode_pub, self.control_mode, 'control_mode')
		elif key in ('k', 'K'):
			self.control_mode = 1
			self._publish_int(self.control_mode_pub, self.control_mode, 'control_mode')
		elif key in ('l', 'L'):
			self.control_mode = 2
			self._publish_int(self.control_mode_pub, self.control_mode, 'control_mode')
		else:
			self.get_logger().info("未定义按键，请按 h 查看帮助")


def main(args=None):
	settings = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())
	rclpy.init(args=args)
	node = KeyboardPublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
	main()
