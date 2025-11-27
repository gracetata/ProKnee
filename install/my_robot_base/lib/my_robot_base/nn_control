#!/usr/bin/env python3
"""
nn_control.py - 基于ONNX模型的实时步态控制节点

功能:
- 订阅串口反馈数据 /serial_feedback
- 解析传感器数据 (Angle, Speed, macro_phase_now 等)
- 根据 macro_phase_now (1=Stance, 2=Swing) 选择ONNX模型
- 执行ONNX模型推理，预测控制目标 (力矩/角度)
- 发布控制指令到 /torque_target, /angle_target, /control_mode
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
import numpy as np
import onnxruntime as ort
import time
from collections import deque
from pathlib import Path
import sys
from typing import Tuple

try:
    from ament_index_python.packages import (
        get_package_share_directory,
        PackageNotFoundError,
    )
except ImportError:  # 直接运行脚本时可能尚未安装
    get_package_share_directory = None
    class PackageNotFoundError(Exception):
        pass

# --- 动态添加路径以导入辅助模块 ---
script_dir = Path(__file__).parent


def _find_workspace_root(start_path: Path) -> Path:
    resolved = start_path.resolve()
    for parent in resolved.parents:
        if (parent / 'src').is_dir():
            return parent
    return resolved


WORKSPACE_ROOT = _find_workspace_root(script_dir)
PACKAGE_SHARE_DIR = None
if get_package_share_directory is not None:
    try:
        PACKAGE_SHARE_DIR = Path(get_package_share_directory('my_robot_base'))
    except PackageNotFoundError:
        PACKAGE_SHARE_DIR = None


def resolve_resource_path(path_str: str) -> Path:
    path = Path(path_str)
    if path.is_absolute():
        return path
    search_bases = [PACKAGE_SHARE_DIR, WORKSPACE_ROOT, script_dir]
    for base in [b for b in search_bases if b is not None]:
        candidate = (base / path).resolve()
        if candidate.exists():
            return candidate
    # 如果仍未找到，返回第一个可用基准路径的推测值
    for base in [b for b in search_bases if b is not None]:
        return (base / path).resolve()
    return path.resolve()


support_candidates = []
support_candidates.append(script_dir / 'Inference_Support')
if PACKAGE_SHARE_DIR is not None:
    support_candidates.append(PACKAGE_SHARE_DIR / 'Inference_Support')
if WORKSPACE_ROOT is not None:
    support_candidates.append(WORKSPACE_ROOT / 'src/my_robot_base/scripts/Inference_Support')

support_dir = next((candidate for candidate in support_candidates if candidate.is_dir()), None)
if support_dir is not None:
    sys.path.insert(0, str(support_dir))
else:
    candidates_str = "\n  ".join(str(path) for path in support_candidates)
    print("错误: 无法找到 'Inference_Support' 目录。已检查:\n  " + candidates_str)
    sys.exit(1)

try:
    from positional_encoding import get_encoder, FeatureEngineer
except ImportError:
    print("错误: 无法导入 'positional_encoding'。请确保 positional_encoding.py 在 'Inference_Support' 目录中。")
    sys.exit(1)
# FiLM网络定义不是必需的，因为我们只使用ONNX进行推理
# from FiLM.film_network import FiLMNetworkDirect


class RealtimeGaitPredictor:
    """
    实时步态预测器 - ONNX部署版本
    
    使用 onnxruntime 加速推理，适配ROS 2节点。
    """
    
    def __init__(self, 
                 stance_model_path='Models/stance_model.onnx',
                 swing_model_path='Models/swing_model.onnx',
                 device='cpu',
                 height=1.75,
                 weight=70.0,
                 ramp_incline=0.0,
                 logger=None):
        """
        初始化预测器
        
        Args:
            stance_model_path: Stance ONNX模型路径
            swing_model_path: Swing ONNX模型路径
            device: 'cpu' 或 'cuda'
            height, weight, ramp_incline: 受试者和环境参数
            logger: ROS 2 logger
        """
        self.logger = logger if logger else rclpy.logging.get_logger('RealtimeGaitPredictor')
        stance_model_path = str(resolve_resource_path(stance_model_path))
        swing_model_path = str(resolve_resource_path(swing_model_path))
        
        if device == 'cuda' and ort.get_device() == 'GPU':
            self.provider = ['CUDAExecutionProvider']
            self.logger.info("[Predictor] 使用设备: CUDA")
        else:
            self.provider = ['CPUExecutionProvider']
            self.logger.info("[Predictor] 使用设备: CPU")

        # 保存个体特征
        self.height = height
        self.weight = weight
        self.ramp_incline = ramp_incline
        
        # 初始化编码器
        self.ramp_encoder = get_encoder('positional', 32, max_value=10.0)
        self.height_encoder = FeatureEngineer(encoding_dim=4, max_value=2.0)
        self.weight_encoder = FeatureEngineer(encoding_dim=4, max_value=150.0)
        
        # ========== 加载ONNX模型 ==========
        self.logger.info(f"[Predictor] 加载 Stance ONNX 模型: {stance_model_path}")
        self.stance_session = ort.InferenceSession(stance_model_path, providers=self.provider)
        
        self.logger.info(f"[Predictor] 加载 Swing ONNX 模型: {swing_model_path}")
        self.swing_session = ort.InferenceSession(swing_model_path, providers=self.provider)
        
        self.logger.info("[Predictor] ✅ 双模型加载完成")
        
        # 状态变量
        self.current_phase = 'STANCE'
        
        # 性能统计
        self.inference_times = deque(maxlen=100)
        
        # 预编码条件向量
        self.condition_vector = self._encode_condition()

    def _encode_condition(self):
        """编码身高、体重、坡度等条件"""
        height_encoded = self.height_encoder.encode(self.height)
        weight_encoded = self.weight_encoder.encode(self.weight)
        ramp_encoded = self.ramp_encoder.encode(self.ramp_incline)
        
        # 组合成40维条件向量
        condition = np.concatenate([ramp_encoded, height_encoded, weight_encoded]).astype(np.float32)
        return np.expand_dims(condition, 0) # -> (1, 40)

    def predict(self, 
                phase_str: str,
                hip_sagittal: float,
                t_in_phase: float,
                knee_angle_r: float,
                knee_angular_velocity: float) -> Tuple[str, float, float]:
        """
        实时预测
        
        Args:
            phase_str: "STANCE" 或 "SWING"
            hip_sagittal, t_in_phase, knee_angle_r, knee_angular_velocity: 传感器输入
        
        Returns:
            (phase, predicted_phi, predicted_output)
        """
        start_time = time.perf_counter()
        
        self.current_phase = phase_str
        
        # 准备输入张量
        phase_input = np.array([[hip_sagittal, t_in_phase]], dtype=np.float32)
        main_input = np.array([[hip_sagittal, knee_angle_r, knee_angular_velocity]], dtype=np.float32)
        
        # 根据相位选择模型
        if self.current_phase == 'STANCE':
            session = self.stance_session
        else: # SWING
            session = self.swing_session
            
        # ONNX模型输入是字典
        onnx_inputs = {
            session.get_inputs()[0].name: phase_input,
            session.get_inputs()[1].name: main_input,
            session.get_inputs()[2].name: self.condition_vector
        }
        
        # 执行推理
        predicted_phase_onnx, output_onnx = session.run(None, onnx_inputs)
        
        predicted_phi = predicted_phase_onnx[0][0]
        predicted_output = output_onnx[0][0]
        
        elapsed = (time.perf_counter() - start_time) * 1000  # ms
        self.inference_times.append(elapsed)
        
        return (self.current_phase, predicted_phi, predicted_output)

    def get_performance_stats(self):
        """获取性能统计"""
        if not self.inference_times:
            return None
        times = list(self.inference_times)
        mean_time = np.mean(times)
        return {
            'mean_ms': mean_time,
            'std_ms': np.std(times),
            'frequency_hz': 1000 / mean_time if mean_time > 0 else 0,
        }

    def update_ramp_incline(self, ramp_incline: float):
        """更新坡度并重新编码条件向量"""
        self.ramp_incline = ramp_incline
        self.condition_vector = self._encode_condition()
        self.logger.info(f"[Predictor] 坡度已更新为: {ramp_incline}°")


class NNControlNode(Node):
    """
    神经网络控制节点
    """
    def __init__(self):
        super().__init__('nn_control_node')
        self.get_logger().info("神经网络控制节点 (NN Control Node) 启动...")

        # 1. 声明参数
        self.declare_parameter('stance_model_path', 'Models/stance_model.onnx')
        self.declare_parameter('swing_model_path', 'Models/swing_model.onnx')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('height', 1.75)
        self.declare_parameter('weight', 70.0)
        self.declare_parameter('ramp_incline', 0.0)

        # 2. 获取参数
        stance_path = self.get_parameter('stance_model_path').get_parameter_value().string_value
        swing_path = self.get_parameter('swing_model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        height = self.get_parameter('height').get_parameter_value().double_value
        weight = self.get_parameter('weight').get_parameter_value().double_value
        ramp_incline = self.get_parameter('ramp_incline').get_parameter_value().double_value

        # 将模型路径解析为绝对路径
        stance_model_abs_path = str(resolve_resource_path(stance_path))
        swing_model_abs_path = str(resolve_resource_path(swing_path))

        # 3. 初始化预测器
        self.predictor = RealtimeGaitPredictor(
            stance_model_path=stance_model_abs_path,
            swing_model_path=swing_model_abs_path,
            device=device,
            height=height,
            weight=weight,
            ramp_incline=ramp_incline,
            logger=self.get_logger()
        )

        # 4. 创建订阅和发布
        self.feedback_sub = self.create_subscription(
            String,
            'serial_feedback',
            self.feedback_callback,
            10)
        
        self.torque_pub = self.create_publisher(Float32, 'torque_target', 10)
        self.angle_pub = self.create_publisher(Float32, 'angle_target', 10)
        self.control_mode_pub = self.create_publisher(Int32, 'control_mode', 10)
        self.mode_pub = self.create_publisher(Int32, 'mode', 10)
        self.motion_pub = self.create_publisher(Int32, 'motion', 10)
        self.motion_sub = self.create_subscription(
            Int32,
            'motion',
            self.motion_callback,
            10)
        self.motion_mode = 2  # 2: walk, 1: stand, 0: stop
        self.prev_macro_phase = None
        
        self.get_logger().info("✅ 节点初始化完成，等待 /serial_feedback 数据...")

    def motion_callback(self, msg: Int32):
        self._set_motion_mode(int(msg.data))

    def _set_motion_mode(self, new_mode: int, publish: bool = False):
        if new_mode not in (0, 1, 2):
            self.get_logger().warning(f"收到无效 motion 模式: {new_mode}")
            return
        if self.motion_mode == new_mode:
            return
        self.motion_mode = new_mode
        if publish:
            motion_msg = Int32()
            motion_msg.data = int(new_mode)
            self.motion_pub.publish(motion_msg)
        self.get_logger().info(f"motion 状态切换为 {new_mode}")

    def _publish_mode_value(self, value: int):
        msg = Int32()
        msg.data = int(value)
        self.mode_pub.publish(msg)

    def _publish_control_mode_value(self, value: int):
        msg = Int32()
        msg.data = int(value)
        self.control_mode_pub.publish(msg)

    def _publish_angle_target(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self.angle_pub.publish(msg)

    def _publish_torque_target(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self.torque_pub.publish(msg)

    def _apply_stand_control(self):  # motion 1
        self._publish_mode_value(2)  # 后续改为2
        self._publish_control_mode_value(2)
        self._publish_angle_target(0.0)
        self._publish_torque_target(0.0)

    def _apply_stop_control(self): # motion 0
        self._publish_mode_value(0)  # 保持原状
        self._publish_control_mode_value(0)
        self._publish_angle_target(0.0)
        self._publish_torque_target(0.0)

    def feedback_callback(self, msg: String):
        """
        处理来自 /serial_feedback 的数据
        数据格式: "key1=val1, key2=val2, ..."
        """
        try:
            # 解析数据
            data = dict(item.split('=') for item in msg.data.split(', '))
            
            # --- 从数据中提取模型输入 ---
            # !! 注意：这里的键名需要与 serial_comm_node.cpp 中发布的一致 !!
            # !! 并且，部分输入可能需要从其他话题或传感器获取 !!
            
            # 核心输入
            macro_phase_now = int(data.get('macro_phase_now', 1)) # 默认为1 (Stance)
            knee_angle_r = float(data.get('Angle', 0.0))
            knee_angular_velocity = float(data.get('Speed', 0.0))
            Hip_sagittal = float(data.get('Hip_Sagittal_Angle', 0.0))
            T_in_Phase = float(data.get('Cnt_in_Phase', 0.0)) * 0.01
            # 假设的输入 (需要根据实际情况修改)
            
            

            # 根据 macro_phase_now 确定当前相位
            phase_str = 'STANCE' if macro_phase_now == 1 else 'SWING'

            if (self.prev_macro_phase == 2 and macro_phase_now == 1 
                    and self.motion_mode == 1):
                self.get_logger().info("检测到宏相位由 SWING 切换到 STANCE，进入行走模式")
                self._set_motion_mode(2, publish=True)
            self.prev_macro_phase = macro_phase_now

            if self.motion_mode == 0:
                self._apply_stop_control()
                return
            if self.motion_mode == 1:
                self._apply_stand_control()
                return

            # 执行预测
            phase, phi, output = self.predictor.predict(
                phase_str=phase_str,
                hip_sagittal=Hip_sagittal,
                t_in_phase=T_in_Phase,
                knee_angle_r=knee_angle_r,
                knee_angular_velocity=knee_angular_velocity
            )

            self._publish_mode_value(2) # 后续改成2

            # 根据相位发布不同控制目标
            if phase == 'STANCE':
                # 发布力矩控制
                torque_msg = Float32()
                torque_msg.data = float(output)
                self.torque_pub.publish(torque_msg)
                
                # 设置控制模式为力矩模式 (假设 1 是力矩模式)
                control_mode_msg = Int32()
                control_mode_msg.data = 1
                self.control_mode_pub.publish(control_mode_msg)
                
                self.get_logger().info(f"推断结果 [STANCE]: 目标力矩={output:.4f} Nm/kg, 预测相位={phi:.2f}")

            else: # SWING
                # 发布角度控制
                angle_msg = Float32()
                angle_msg.data = float(-output)
                self.angle_pub.publish(angle_msg)

                # 设置控制模式为角度/位置模式 (假设 2 是位置模式)
                control_mode_msg = Int32()
                control_mode_msg.data = 2
                self.control_mode_pub.publish(control_mode_msg)

                self.get_logger().info(f"推断结果 [SWING]: 目标角度={-output:.2f} rad, 预测相位={phi:.2f}")

        except Exception as e:
            self.get_logger().error(f"处理feedback消息时出错: {e}\n原始消息: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    node = NNControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
