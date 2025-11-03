#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, String
from typing import Optional, Dict

# 宏观相位定义 (与高层控制器保持一致)
MACRO_STANCE = 0
MACRO_SWING = 1

# 传感器数据结构
class SensorState:
    """从下位机反馈解析得到的关键传感量。"""
    knee_angle_deg: Optional[float] = None  # 膝角（度），伸直为0，屈曲为正
    knee_vel_dps: Optional[float] = None    # 膝角速度（度/秒），屈曲为正

    def __repr__(self):
        return f"SensorState(angle={self.knee_angle_deg:.2f} deg, vel={self.knee_vel_dps:.2f} dps)"

def parse_sensor_feedback(raw: str) -> SensorState:
    """
    一个简化的解析器，用于从 'key=value, ...' 格式的字符串中提取膝关节角度和速度。
    """
    state = SensorState()
    try:
        parts = [p.strip() for p in raw.split(',')]
        kv = {}
        for p in parts:
            if '=' in p:
                key, value = p.split('=', 1)
                kv[key.strip()] = value.strip()

        # 别名处理，兼容多种可能的键名
        angle_keys = ['Angle', 'knee_angle', 'knee_deg']
        vel_keys = ['Speed', 'knee_vel', 'knee_vel_dps']

        for key in angle_keys:
            if key in kv:
                state.knee_angle_deg = float(kv[key])
                break
        
        for key in vel_keys:
            if key in kv:
                state.knee_vel_dps = float(kv[key])
                break
    except (ValueError, IndexError) as e:
        # 解析失败时，保持值为 None
        pass
    return state


class ProsthesisControl(Node):
    """
    假肢中层控制器。
    - 订阅步态分类结果和传感器数据。
    - 根据支撑/摆动相，切换角度/阻抗控制。
    - 发布目标角度或目标力矩。
    """
    def __init__(self):
        super().__init__('prosthesis_control')

        # --- 控制参数 ---
        # 阻抗控制参数 (Stance)
        self.impedance_k = 10.0  # 刚度 (Nm/rad)
        self.impedance_b = 1.0   # 阻尼 (Nm/(rad/s))
        self.impedance_theta_eq = 5.0  # 平衡角度 (度)

        # 角度控制参数 (Swing) - 示例：一个简单的正弦轨迹
        self.swing_amplitude_deg = 60.0 # 摆动最大屈曲角
        self.swing_freq_hz = 1.0 # 摆动频率

        # --- 发布者 ---
        self.pub_angle_target = self.create_publisher(Float32, 'angle_target', 10)
        self.pub_torque_target = self.create_publisher(Float32, 'torque_target', 10)
        # 速度目标暂时不用，但可以置零
        self.pub_speed_target = self.create_publisher(Float32, 'speed_target', 10)

        # --- 订阅者 ---
        self.create_subscription(
            Int32MultiArray,
            'motion_classification',
            self.classification_callback,
            10
        )
        self.create_subscription(
            String,
            'serial_feedback',
            self.feedback_callback,
            10
        )

        # --- 状态变量 ---
        self.current_mode = 0
        self.current_macro_phase = MACRO_STANCE # 默认为支撑相
        self.sensor_state = SensorState()
        self.swing_start_time = time.time()

        # --- 控制循环 ---
        self.control_timer = self.create_timer(0.02, self.run_control_loop) # 50Hz

        self.get_logger().info('Prosthesis Control Node started.')

    def classification_callback(self, msg: Int32MultiArray):
        """处理来自高层控制器的步态分类信息。"""
        if len(msg.data) >= 2:
            new_mode = msg.data[0]
            new_macro_phase = msg.data[1]

            if new_macro_phase != self.current_macro_phase:
                self.get_logger().info(f"Phase transition: {'STANCE' if self.current_macro_phase==0 else 'SWING'} -> {'STANCE' if new_macro_phase==0 else 'SWING'}")
                if new_macro_phase == MACRO_SWING:
                    # 刚进入摆动期，重置时间戳
                    self.swing_start_time = time.time()

            self.current_mode = new_mode
            self.current_macro_phase = new_macro_phase

    def feedback_callback(self, msg: String):
        """处理来自串口节点的传感器反馈。"""
        self.sensor_state = parse_sensor_feedback(msg.data)

    def run_control_loop(self):
        """主控制循环，根据当前相位执行相应控制策略。"""
        if self.current_macro_phase == MACRO_STANCE:
            self.run_stance_control()
        else: # MACRO_SWING
            self.run_swing_control()

    def run_stance_control(self):
        """执行支撑相的阻抗控制。"""
        if self.sensor_state.knee_angle_deg is None or self.sensor_state.knee_vel_dps is None:
            # 传感器数据无效，不发送控制指令
            return

        # 将角度和角速度单位转换为弧度
        theta = math.radians(self.sensor_state.knee_angle_deg)
        theta_dot = math.radians(self.sensor_state.knee_vel_dps)
        theta_eq = math.radians(self.impedance_theta_eq)

        # 阻抗控制公式
        torque = -self.impedance_k * (theta - theta_eq) - self.impedance_b * theta_dot
        
        # 发布目标力矩
        self.pub_torque_target.publish(Float32(data=float(torque)))
        
        # 在Stance阶段，角度和速度目标可以设为0或NaN，让下位机忽略它们
        self.pub_angle_target.publish(Float32(data=float('nan')))
        self.pub_speed_target.publish(Float32(data=float('nan')))

        self.get_logger().debug(f"STANCE: angle={theta:.2f}, vel={theta_dot:.2f} -> torque={torque:.2f}")

    def run_swing_control(self):
        """执行摆动相的角度控制。"""
        # 示例：基于时间的简单正弦轨迹
        # 实际应用中应替换为更复杂的轨迹生成器（如多项式、贝塞尔曲线等）
        t = time.time() - self.swing_start_time
        
        # 一个简化的摆动轨迹：半个周期的正弦波，模拟屈曲到伸展
        # 确保在 t=0 时角度为0，并在摆动中期达到最大屈曲
        swing_duration = 1.0 / (2 * self.swing_freq_hz) # 假设摆动占半个周期
        if t > swing_duration:
            target_angle_deg = 0.0 # 摆动末期，膝关节伸直
        else:
            target_angle_deg = self.swing_amplitude_deg * math.sin((math.pi / swing_duration) * t)

        # 发布目标角度
        self.pub_angle_target.publish(Float32(data=float(target_angle_deg)))

        # 在Swing阶段，力矩目标可以设为0或NaN
        self.pub_torque_target.publish(Float32(data=float('nan')))
        self.pub_speed_target.publish(Float32(data=float('nan')))

        self.get_logger().debug(f"SWING: t={t:.2f} -> target_angle={target_angle_deg:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ProsthesisControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
