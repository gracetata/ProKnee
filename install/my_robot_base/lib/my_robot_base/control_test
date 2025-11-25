#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
#输入 m - 切换到 mode topic（发布整数）
#输入 a - 切换到 angle_target topic（发布浮点数）
#输入 t - 切换到 torque_target topic（发布浮点数）
#输入 c - 切换到 control_mode topic（发布整数：0=角度控制，1=力矩控制）
#输入数字 - 发布到当前选中的 topic
class ControlTest(Node):
    """
    Reads a target angle from the keyboard and publishes it to /angle_target.
    Can switch to mode, torque, and control_mode topics.
    """
    def __init__(self):
        super().__init__('control_test')
        self.angle_publisher_ = self.create_publisher(Float32, 'angle_target', 1)
        self.torque_publisher_ = self.create_publisher(Float32, 'torque_target', 1)
        self.mode_publisher_ = self.create_publisher(Int32, 'mode', 1)
        self.control_mode_publisher_ = self.create_publisher(Int32, 'control_mode', 1)
        self.current_topic = 'angle'  # 'angle', 'torque', 'mode', or 'control_mode'

        # Initialize values
        self.target_angle = 0.0
        self.target_torque = 0.0
        self.target_mode = 0
        self.target_control_mode = 0
        
        # Create timer for continuous publishing (10Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info('Control Test node started.')
        self.get_logger().info('Commands:')
        self.get_logger().info('  m - Switch to mode topic (integer)')
        self.get_logger().info('  a - Switch to angle_target topic (float)')
        self.get_logger().info('  t - Switch to torque_target topic (float)')
        self.get_logger().info('  c - Switch to control_mode topic (0=angle, 1=torque)')
        self.get_logger().info('  <number> - Publish value to current topic')
        self.get_logger().info(f'Current topic: angle_target')
        
        # Start a thread to read from stdin
        self._input_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._input_thread.start()

    def timer_callback(self):
        msg_angle = Float32()
        msg_angle.data = self.target_angle
        self.angle_publisher_.publish(msg_angle)

        msg_torque = Float32()
        msg_torque.data = self.target_torque
        self.torque_publisher_.publish(msg_torque)

        msg_mode = Int32()
        msg_mode.data = self.target_mode
        self.mode_publisher_.publish(msg_mode)

        msg_control_mode = Int32()
        msg_control_mode.data = self.target_control_mode
        self.control_mode_publisher_.publish(msg_control_mode)

    def _stdin_loop(self):
        """
        Loop to read lines from stdin and publish them based on current topic.
        """
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    # End of file, exit thread
                    break
                
                input_str = line.strip().lower()
                
                # Handle topic switch commands
                if input_str == 'm':
                    self.current_topic = 'mode'
                    self.get_logger().info('Switched to mode topic (integer values)')
                    continue
                elif input_str == 'a':
                    self.current_topic = 'angle'
                    self.get_logger().info('Switched to angle_target topic (float values)')
                    continue
                elif input_str == 't':
                    self.current_topic = 'torque'
                    self.get_logger().info('Switched to torque_target topic (float values)')
                    continue
                elif input_str == 'c':
                    self.current_topic = 'control_mode'
                    self.get_logger().info('Switched to control_mode topic (0=angle, 1=torque)')
                    continue
                
                # Try to parse as number and update current topic value
                try:
                    if self.current_topic == 'angle':
                        # Convert input to float for angle
                        value = float(input_str)
                        self.target_angle = value
                        self.get_logger().info(f'Set angle_target: {value:.2f} degrees')
                    
                    elif self.current_topic == 'torque':
                        # Convert input to float for torque
                        value = float(input_str)
                        self.target_torque = value
                        self.get_logger().info(f'Set torque_target: {value:.2f} Nm')
                    
                    elif self.current_topic == 'mode':
                        # Convert input to int for mode
                        value = int(float(input_str))  # Allow float input but convert to int
                        self.target_mode = value
                        self.get_logger().info(f'Set mode: {value}')
                    
                    elif self.current_topic == 'control_mode':
                        # Convert input to int for control_mode
                        value = int(float(input_str))  # Allow float input but convert to int
                        if value not in [0, 1]:
                            self.get_logger().warn(f'Invalid control_mode: {value}. Use 1 (angle) or 2 (torque)')
                            continue
                        self.target_control_mode = value
                        self.get_logger().info(f'Set control_mode: {value} ({"angle" if value==0 else "torque"} control)')
                    
                except ValueError:
                    # Handle cases where input is not a valid number
                    if input_str: # Avoid warning on empty lines
                        self.get_logger().warn(f'Invalid input: "{input_str}". Please enter a number or command (m/a/t/c).')

            except Exception as e:
                self.get_logger().error(f'An error occurred in the input loop: {e}')
                break
def main(args=None):
    rclpy.init(args=args)
    node = ControlTest()
    try:
        # rclpy.spin() keeps the node alive to handle callbacks and shutdown
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
