#!/usr/bin/env python3
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Int32, String
from std_msgs.msg import Int32MultiArray


MODE_MAP: Dict[str, int] = {
    '0': 0,  # 平地
    '1': 1,  # 上楼
    '2': 2,  # 下楼
    '3': 3,  # 上坡
    '4': 4,  # 下坡
}

MODE_DESC: Dict[int, str] = {
    0: '平地',
    1: '上楼',
    2: '下楼',
    3: '上坡',
    4: '下坡',
}

# 四相命名（可按需调整到具体模式语义）
PHASE_DESC: Dict[int, str] = {
    0: 'P0',
    1: 'P1',
    2: 'P2',
    3: 'P3',
}

# 宏观相位（一级状态）：支撑/摆动
MACRO_STANCE = 0
MACRO_SWING = 1
MACRO_DESC = {MACRO_STANCE: 'STANCE', MACRO_SWING: 'SWING'}

# ---------------------------
#  传感器数据结构与解析
# ---------------------------
@dataclass
class SensorState:
    """从下位机反馈解析得到的关键传感量。

    按需扩展字段。若某字段暂不可得，保持为 None。
    """
    t: float = 0.0  # 时间戳（秒）
    knee_angle_deg: Optional[float] = None  # 膝角（度），伸直为0，屈曲为正
    knee_vel_dps: Optional[float] = None    # 膝角速度（度/秒），屈曲为正
    foot_contact: Optional[int] = None      # 足底接触(1)/离地(0)
    raw: str = ''                           # 原始字符串


def parse_sensor_feedback(raw: str, prev: SensorState, now: float) -> SensorState:
        """解析下位机反馈文本为 SensorState。

        支持两类示例格式：
            - 键值对: "knee=12.3, vel=-5.1, foot_pressure=1" 或 "... , foot_contact=1" 等
            - CSV:    "12.3,-5.1,1" 约定顺序 [knee_deg, knee_vel_dps, foot_contact]

        变量名对齐：
            - knee / knee_deg / angle / theta → 膝角（度）
            - vel / knee_vel / omega → 膝角速度（度/秒）
            - foot_pressure / foot_contact / contact / foot / fc → 足底接触（非零判定为接触）

        若解析失败，将沿用 prev 中历史值，仅更新时间与 raw。
        """
        s = SensorState(t=now, raw=raw,
                        knee_angle_deg=prev.knee_angle_deg,
                        knee_vel_dps=prev.knee_vel_dps,
                        foot_contact=prev.foot_contact)

        txt = raw.strip()
        if not txt:
            return s

        try:
            if '=' in txt:
                # 键值对解析
                parts = [p.strip() for p in txt.replace(';', ',').split(',') if p.strip()]
                kv = {}
                for p in parts:
                    if '=' in p:
                        k, v = p.split('=', 1)
                        kv[k.strip().lower()] = v.strip()
                # 常见别名
                def getf(*keys) -> Optional[float]:
                    for k in keys:
                        if k in kv:
                            try:
                                return float(kv[k])
                            except ValueError:
                                return None
                    return None
                def geti(*keys) -> Optional[int]:
                    v = getf(*keys)
                    return int(v) if v is not None else None

                ang = getf('knee', 'knee_deg', 'angle', 'theta')
                vel = getf('knee_vel', 'vel', 'omega')
                # 接触信号可从 foot_contact 或 foot_pressure 等字段推断；非零即接触
                fc_val = getf('foot_pressure', 'foot_contact', 'contact', 'foot', 'fc')
                if ang is not None:
                    s.knee_angle_deg = ang
                if vel is not None:
                    s.knee_vel_dps = vel
                if fc_val is not None:
                    s.foot_contact = 1 if fc_val != 0 else 0
                return s
            else:
                # CSV解析
                parts = [p.strip() for p in txt.split(',')]
                if len(parts) >= 1:
                    try:
                        s.knee_angle_deg = float(parts[0])
                    except ValueError:
                        pass
                if len(parts) >= 2:
                    try:
                        s.knee_vel_dps = float(parts[1])
                    except ValueError:
                        pass
                if len(parts) >= 3:
                    try:
                        s.foot_contact = 1 if int(float(parts[2])) != 0 else 0
                    except ValueError:
                        pass
                return s
        except Exception:
            # 容错：保持历史值
            return s


"""
两级状态机结构：
1) 宏观层（宏相位）：STANCE(支撑)/SWING(摆动)。仅负责 HS/TO 事件切换。
2) 模式层（微相位）：每种模式内部自定义多个子相位与顺序及判据。

你可以只改各 ModeMachine 的 update_* 方法来写规则，而无需碰事件检测与宏观切换逻辑。
"""


class EventDetector:
    """基础事件检测：Heel-Strike(HS) 与 Toe-Off(TO)。

    默认用 foot_contact 的边沿： 0->1 判定 HS；1->0 判定 TO。
    若缺少 foot_contact，可选用角速度启发（阈值可加参数化）。
    """

    def __init__(self, macro_min_time: float = 0.1, swing_vel_thresh: float = 20.0):
        self.macro_min_time = float(macro_min_time)
        self.swing_vel_thresh = float(swing_vel_thresh)
        self._last_macro = MACRO_STANCE
        self._last_change = 0.0
        self._last_contact: Optional[int] = None

    def update(self, state: SensorState, now: float) -> Tuple[int, Optional[str]]:
        """返回 (new_macro, reason)；若不变则返回当前宏观状态与 None。"""
        macro = self._last_macro

        # 驻留时间限制，避免 chatter
        if now - self._last_change < self.macro_min_time:
            return macro, None

        reason: Optional[str] = None
        fc = state.foot_contact
        if fc is not None:
            if self._last_contact is not None and self._last_contact != fc:
                if fc == 1 and macro != MACRO_STANCE:
                    macro = MACRO_STANCE
                    reason = 'HS by foot_contact 0→1'
                elif fc == 0 and macro != MACRO_SWING:
                    macro = MACRO_SWING
                    reason = 'TO by foot_contact 1→0'
            self._last_contact = fc
        else:
            # 没有接触信号：使用角速度高阈值启发进入摆动
            if state.knee_vel_dps is not None and state.knee_vel_dps > self.swing_vel_thresh and macro != MACRO_SWING:
                macro = MACRO_SWING
                reason = f'TO by knee_vel>{self.swing_vel_thresh}'

        if macro != self._last_macro:
            self._last_macro = macro
            self._last_change = now
            return macro, reason or 'macro changed'

        return macro, None


class BaseModeMachine:
    """每种模式的微相位状态机基类。

    你需要重写：
      - phase_names_stance / phase_names_swing（各自相位名称列表，长度可不同）
      - on_enter_macro(macro, now) 进入支撑/摆动时复位逻辑
      - update_in_macro(macro, phase_idx, state, now) 根据传感器在当前宏内推进子相位

    update_in_macro 返回 (new_phase_idx, reason)。
    """

    name = 'Base'

    def __init__(self):
        self.phase_names_stance: List[str] = ['S0', 'S1']
        self.phase_names_swing: List[str] = ['W0', 'W1']
        self.current_phase_idx: int = 0

    def on_enter_macro(self, macro: int, now: float):
        # 缺省：每次进入宏观状态，从该宏的第0相开始
        self.current_phase_idx = 0

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        # 缺省：不改变
        return phase_idx, 'hold'

    def phase_name(self, macro: int, phase_idx: int) -> str:
        names = self.phase_names_stance if macro == MACRO_STANCE else self.phase_names_swing
        if 0 <= phase_idx < len(names):
            return names[phase_idx]
        return f'P{phase_idx}'


# ---------------------------
# 各模式的微相位机（仅提供骨架）
# ---------------------------

class FlatGroundMachine(BaseModeMachine):
    """平地：示例四相（每个宏各2相，共4）
    - STANCE: S0(初触/加载) → S1(推离)
    - SWING:  W0(初摆) → W1(末摆)
    你可以在 update_in_macro 中写具体阈值规则。
    """
    name = 'Flat'

    def __init__(self):
        super().__init__()
        self.phase_names_stance = ['S0_touch', 'S1_push']
        self.phase_names_swing = ['W0_early', 'W1_late']

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        # 示例逻辑：仅做占位，便于替换
        if macro == MACRO_STANCE:
            # 用膝角阈值区分加载/推离
            ang = state.knee_angle_deg or 0.0
            if phase_idx == 0 and ang > 10.0:
                return 1, f'knee {ang:.1f}>10 → S1'
        else:
            vel = state.knee_vel_dps or 0.0
            if phase_idx == 0 and vel < 0.0:
                return 1, f'vel {vel:.1f}<0 → W1'
        return phase_idx, 'hold'


class StairUpMachine(BaseModeMachine):
    name = 'StairUp'

    def __init__(self):
        super().__init__()
        # 可使用不同数量相位，例如上楼支撑更多细分
        self.phase_names_stance = ['S0_load', 'S1_raise', 'S2_lift']
        self.phase_names_swing = ['W0_clear', 'W1_place']

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        # TODO: 根据台阶高度、膝角峰值等编写实际判据
        return phase_idx, 'hold'


class StairDownMachine(BaseModeMachine):
    name = 'StairDown'

    def __init__(self):
        super().__init__()
        self.phase_names_stance = ['S0_cushion', 'S1_control']
        self.phase_names_swing = ['W0_descent', 'W1_place']

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        return phase_idx, 'hold'


class RampUpMachine(BaseModeMachine):
    name = 'RampUp'

    def __init__(self):
        super().__init__()
        self.phase_names_stance = ['S0_load', 'S1_prop']
        self.phase_names_swing = ['W0_early', 'W1_late']

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        return phase_idx, 'hold'


class RampDownMachine(BaseModeMachine):
    name = 'RampDown'

    def __init__(self):
        super().__init__()
        self.phase_names_stance = ['S0_absorb', 'S1_control']
        self.phase_names_swing = ['W0_early', 'W1_late']

    def update_in_macro(self, macro: int, phase_idx: int, state: SensorState, now: float) -> Tuple[int, str]:
        return phase_idx, 'hold'


def build_mode_machines() -> Dict[int, BaseModeMachine]:
    return {
        0: FlatGroundMachine(),
        1: StairUpMachine(),
        2: StairDownMachine(),
        3: RampUpMachine(),
        4: RampDownMachine(),
    }


class KeyboardMotionMode(Node):
    """
    键盘选择运动模式并发布到 ROS2 话题。

    - 话题: motion_mode (std_msgs/Int32)
    - QoS: Transient Local (新订阅者获取最近一次模式)
    - 默认按键: 0~4；q 退出；h 显示帮助
    """

    def __init__(self):
        super().__init__('keyboard_motion_mode')

        # 参数
        self.declare_parameter('topic_name', 'motion_mode')
        self.declare_parameter('print_help_on_start', True)
        self.declare_parameter('republish_period_sec', 0.0)  # 0 表示不定期重发
        # 传感器/相位相关参数
        self.declare_parameter('sensor_feedback_topic', 'serial_feedback')
        self.declare_parameter('gait_phase_topic', 'gait_phase')
        self.declare_parameter('gait_macro_topic', 'gait_macro_phase')
        self.declare_parameter('gait_phase_name_topic', 'gait_phase_name')
        self.declare_parameter('classification_topic', 'motion_classification')
        self.declare_parameter('phase_republish_period_sec', 0.5)  # 定期重发相位
        self.declare_parameter('min_phase_time_sec', 0.15)  # 相位最小驻留时间，避免抖动
        self.declare_parameter('macro_min_time_sec', 0.1)   # 宏观驻留
        self.declare_parameter('publish_phase_name', True)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        print_help = bool(self.get_parameter('print_help_on_start').get_parameter_value().bool_value)
        self.republish_period = float(self.get_parameter('republish_period_sec').get_parameter_value().double_value)
        self.sensor_feedback_topic = self.get_parameter('sensor_feedback_topic').get_parameter_value().string_value
        self.gait_phase_topic = self.get_parameter('gait_phase_topic').get_parameter_value().string_value
        self.gait_macro_topic = self.get_parameter('gait_macro_topic').get_parameter_value().string_value
        self.gait_phase_name_topic = self.get_parameter('gait_phase_name_topic').get_parameter_value().string_value
        self.classification_topic = self.get_parameter('classification_topic').get_parameter_value().string_value
        self.phase_republish_period = float(self.get_parameter('phase_republish_period_sec').get_parameter_value().double_value)
        self.min_phase_time = float(self.get_parameter('min_phase_time_sec').get_parameter_value().double_value)
        self.macro_min_time = float(self.get_parameter('macro_min_time_sec').get_parameter_value().double_value)
        self.publish_phase_name = bool(self.get_parameter('publish_phase_name').get_parameter_value().bool_value)

        # QoS：Transient Local 以便后加入的订阅者拿到最近一次模式
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Int32, topic_name, qos)
        # 微相位发布者（Int32: 0..N-1）
        self.phase_pub = self.create_publisher(Int32, self.gait_phase_topic, qos)
        # 宏相位发布者（0:STANCE,1:SWING）
        self.macro_pub = self.create_publisher(Int32, self.gait_macro_topic, qos)
        # 相位名称（可选）
        self.phase_name_pub = self.create_publisher(String, self.gait_phase_name_topic, qos) if self.publish_phase_name else None
        # 聚合发布（mode, macro, phase_idx）
        self.classification_pub = self.create_publisher(Int32MultiArray, self.classification_topic, qos)

        self._last_mode = 0
        self._running = True
        # 宏相位状态
        self._macro_state: int = MACRO_STANCE
        # 微相位状态
        self._phase_idx: int = 0
        self._last_phase_change_t: float = time.time()
        # 传感器状态缓存
        self._sensor_state: 'SensorState' = SensorState()
        # 事件检测器
        self._event = EventDetector(macro_min_time=self.macro_min_time)
        # 各模式微相位机
        self._machines = build_mode_machines()

        if print_help:
            self._print_help()

        # 初次发布一次默认模式 & 宏/微相位
        self._publish_mode(self._last_mode, announce=True)
        self._publish_macro(self._macro_state, announce=True)
        self._publish_phase(self._phase_idx, announce=True)
        self._publish_classification()

        # 可选：定期重发模式，防止消费端丢包或需要心跳
        if self.republish_period > 0.0:
            self.create_timer(self.republish_period, self._republish_timer)

        # 可选：定期重发相位
        if self.phase_republish_period > 0.0:
            self.create_timer(self.phase_republish_period, self._republish_phase_timer)

        # 订阅下位机反馈，进行相位分类
        self.feedback_sub = self.create_subscription(
            String,
            self.sensor_feedback_topic,
            self._on_feedback,
            50,
        )

        # 输入线程（单独线程读取 stdin，避免阻塞 rclpy.spin）
        self._input_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._input_thread.start()

    def _print_help(self):
        self.get_logger().info(
            '\n选择运动模式：\n'
            '  0: 平地\n'
            '  1: 上楼\n'
            '  2: 下楼\n'
            '  3: 上坡\n'
            '  4: 下坡\n'
            '  h: 显示帮助\n'
            '  q: 退出\n'
            '提示：直接按键即可（不需要回车）。\n')

    def _publish_mode(self, mode: int, announce: bool = False):
        msg = Int32()
        msg.data = mode
        self.pub.publish(msg)
        if announce:
            self.get_logger().info(f'当前模式: {mode} ({MODE_DESC.get(mode, "未知")})')
        else:
            self.get_logger().info(f'切换模式 -> {mode} ({MODE_DESC.get(mode, "未知")})')

    def _republish_timer(self):
        self._publish_mode(self._last_mode, announce=True)

    def _republish_phase_timer(self):
        # 周期性广播当前相位（Transient Local也会让新订阅者拿到最近一次）
        self._publish_phase(self._phase_idx, announce=True)

    # ---------------------------
    #  传感器回调与相位分类（FSM）
    # ---------------------------
    def _on_feedback(self, msg: String):
        now = self.get_clock().now().nanoseconds * 1e-9
        raw = msg.data
        state = parse_sensor_feedback(raw, prev=self._sensor_state, now=now)
        self._sensor_state = state

        # 1) 事件检测：决定是否切换宏相位
        new_macro, macro_reason = self._event.update(state, now)
        if new_macro != self._macro_state:
            self._macro_state = new_macro
            self._on_enter_macro(new_macro, now, reason=macro_reason or '')
            # 进入宏相位后立即发布
            self._publish_macro(new_macro)
            self._publish_phase(self._phase_idx)
            self._publish_classification()
            return

        # 2) 在当前模式/宏观内更新微相位
        machine = self._machines.get(self._last_mode)
        if machine is None:
            return
        new_idx, reason = machine.update_in_macro(self._macro_state, self._phase_idx, state, now)
        if new_idx != self._phase_idx:
            self._phase_idx = new_idx
            self._last_phase_change_t = now
            self._publish_phase(self._phase_idx)
            if self.publish_phase_name:
                name = machine.phase_name(self._macro_state, self._phase_idx)
                self._publish_phase_name(name)
            self._publish_classification()
            self.get_logger().info(f'微相位切换: macro={MACRO_DESC[self._macro_state]} -> {machine.phase_name(self._macro_state, self._phase_idx)} | {reason}')

    def _stdin_loop(self):
        """原生终端单键读取（无需回车）。不适用于某些IDE的内置终端。"""
        try:
            import termios
            import tty
            import select
        except Exception as e:
            self.get_logger().warn(f'终端原生模式不可用，回退到行输入模式: {e}')
            self._line_input_loop()
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)  # 单字符无回车
            while self._running and rclpy.ok():
                # 使用 select 进行非阻塞检测
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ch == 'q':
                    self.get_logger().info('收到退出指令 q，正在退出...')
                    self._running = False
                    # 触发 rclpy.spin 结束
                    rclpy.shutdown()
                    break
                if ch == 'h' or ch == 'H':
                    self._print_help()
                    continue
                if ch in MODE_MAP:
                    mode = MODE_MAP[ch]
                    self._last_mode = mode
                    self._publish_mode(mode)
                    # 模式切换：重置微相位机
                    self._reset_machine_on_mode_change()
                    self._publish_classification()
                else:
                    # 忽略不可识别按键（忽略换行/方向键序列等）
                    if ch not in ('\n', '\r', '\x1b'):  # 过滤常见控制字符
                        self.get_logger().warn(f'未知按键: {repr(ch)} (按 h 查看帮助)')
        except Exception as e:
            self.get_logger().error(f'输入线程异常: {e}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _line_input_loop(self):
        """回退：逐行读取（需要回车）。"""
        self.get_logger().info('回退到行输入模式：输入 0~4 设置模式，h 显示帮助，q 退出。')
        while self._running and rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05)
                    continue
                line = line.strip()
                if line == 'q':
                    self.get_logger().info('收到退出指令 q，正在退出...')
                    self._running = False
                    rclpy.shutdown()
                    break
                if line in ('h', 'H'):
                    self._print_help()
                    continue
                if line in MODE_MAP:
                    mode = MODE_MAP[line]
                    self._last_mode = mode
                    self._publish_mode(mode)
                    self._reset_machine_on_mode_change()
                    self._publish_classification()
                else:
                    self.get_logger().warn('请输入 0~4，或按 h 查看帮助，q 退出。')
            except Exception as e:
                self.get_logger().error(f'读取输入失败: {e}')
                time.sleep(0.2)

    # 发布当前相位
    def _publish_phase(self, phase: int, announce: bool = False):
        m = Int32()
        m.data = int(phase)
        self.phase_pub.publish(m)
        if announce:
            self.get_logger().debug(f'相位: {phase}')

    def _publish_macro(self, macro: int, announce: bool = False):
        m = Int32()
        m.data = int(macro)
        self.macro_pub.publish(m)
        if announce:
            self.get_logger().debug(f'宏相位: {MACRO_DESC.get(macro, str(macro))}')

    def _publish_phase_name(self, name: str):
        if self.phase_name_pub is None:
            return
        msg = String()
        msg.data = name
        self.phase_name_pub.publish(msg)

    def _publish_classification(self):
        """发布聚合后的 (mode, macro, phase_idx) 到 /motion_classification。"""
        arr = Int32MultiArray()
        arr.data = [int(self._last_mode), int(self._macro_state), int(self._phase_idx)]
        self.classification_pub.publish(arr)

    def _reset_machine_on_mode_change(self):
        machine = self._machines.get(self._last_mode)
        if machine is None:
            return
        machine.on_enter_macro(self._macro_state, time.time())
        self._phase_idx = machine.current_phase_idx
        # 立刻广播变化
        self._publish_phase(self._phase_idx)
        if self.publish_phase_name:
            self._publish_phase_name(machine.phase_name(self._macro_state, self._phase_idx))

    def _on_enter_macro(self, macro: int, now: float, reason: str = ''):
        machine = self._machines.get(self._last_mode)
        if machine is None:
            return
        machine.on_enter_macro(macro, now)
        self._phase_idx = machine.current_phase_idx
        self._last_phase_change_t = now
        if self.publish_phase_name:
            self._publish_phase_name(machine.phase_name(macro, self._phase_idx))


def main():
    rclpy.init()
    node = KeyboardMotionMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
