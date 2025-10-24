# my_robot_base

本包 包含基础节点，包括：
- `serial_comm_node`：串口通信（C++）
- `angle_planner.py`：发布串口字符串指令（Python）
- `keyboard_motion_mode.py`：划分运动模式和运动类型（Python）

## 串口通信（二进制帧协议 + 可配置字段）
节点：`serial_comm_node.cpp`

- 帧格式：`[0xAA][Payload][0xBB]`（小端，pack(1) 无填充）
  - Payload 固定长度 53 字节：`10×float (40B) + 5×int16 (10B) + 3×int8 (3B)`
  - 整帧长度 55 字节/帧（含头尾 2B）

- 发送（Jetson→MCU）与接收（MCU→Jetson）使用相同的数组布局：
  - `chf[10]` 浮点通道（float）
  - `chs[5]` 16位整数通道（int16）
  - `chb[3]` 8位整数通道（int8）

- 字段命名与顺序一处配置，处处生效（无需改多处代码）
  - 在 `serial_comm_node.cpp` 顶部通过 X-macro 列表集中管理：
    - 发送浮点：`TX_CHF_FIELD_LIST`（例如：target_torque, target_angle, kp, kd）
    - 发送字节：`TX_CHB_FIELD_LIST`（例如：mode, macro, phase_idx）
    - 接收浮点：`RX_CHF_FIELD_LIST`（例如：knee, vel, torque, curr）
    - 接收短整：`RX_CHS_FIELD_LIST`（例如：foot_contact）
  - 你可以在上述列表里增删/改名/换序，代码会自动：
    - 生成对应的成员变量
    - 生成参数（索引映射）名，如 `tx_chf_kp_idx`、`rx_chs_foot_contact_idx`
    - 打包发送 / 解析接收 / 发布 `/serial_feedback` 的 key 名
    - `/motion_classification` 的订阅赋值顺序

- 输入与输出话题：
  - 输入：`/motion_classification` (std_msgs/Int32MultiArray)，按 `TX_CHB_FIELD_LIST` 顺序提供字节字段（例如 `[mode, macro, phase_idx]`）。
  - 输出：`/serial_feedback` (std_msgs/String)，以 `key=value` 串输出，key 名来自 `RX_*_FIELD_LIST`，例如：`knee=12.300, vel=-5.100, foot_contact=1, torque=3.200, curr=0.800`。

- 可调参数：
  - 基本：
    - `port`（默认 `/dev/ttyUSB0`）
    - `baudrate`（默认 `115200`）
    - `tx_rate_hz`（默认 `50.0`）
    - `enable_ascii_command`（默认 `false`）
  - 索引映射（按列表顺序给出默认值，可用参数覆盖）：
    - 发送浮点：`tx_chf_<name>_idx`，例如 `tx_chf_kp_idx`
    - 发送字节：`tx_chb_<name>_idx`，例如 `tx_chb_mode_idx`
    - 接收浮点：`rx_chf_<name>_idx`，例如 `rx_chf_knee_idx`
    - 接收短整：`rx_chs_<name>_idx`，例如 `rx_chs_foot_contact_idx`

> 吞吐量估算：55 B/帧 × 50 Hz ≈ 2.75 kB/s（约 22 kbps，8N1 计每字节10 bit 时约 27.5 kbps）。若收发同频，总占用约翻倍，仍低于 115200 bps。

### 构建

```bash
cd /home/nvidia/proknee_ws
colcon build --packages-select my_robot_base --symlink-install
source install/setup.bash
```

### 运行

```bash
ros2 run my_robot_base serial_comm_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p tx_rate_hz:=50.0
```

观察接收结果（推荐另开终端）：

```bash
ros2 topic echo /serial_feedback
```

如需在没有其他节点的情况下运行：
- 不发布 `/motion_classification` 时，默认会持续发送 55 字节空帧（字段为默认值 0）。
- 你也可以用一条命令临时发布分类数组（顺序与 `TX_CHB_FIELD_LIST` 保持一致）：

```bash
ros2 topic pub /motion_classification std_msgs/Int32MultiArray "{data: [0, 0, 0]}" -r 5
```

### 测试方法（无下位机时）

1) 使用虚拟串口对（socat）模拟 STM32 发送 Rx 帧（验证接收与 `/serial_feedback`）

```bash
# 创建虚拟串口对（输出两端路径，如 /dev/pts/5 和 /dev/pts/6）
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

在一个终端运行节点，端口指向其中一端（例如 /dev/pts/5）：

```bash
ros2 run my_robot_base serial_comm_node --ros-args -p port:=/dev/pts/5 -p baudrate:=115200
```

在另一个终端向另一端（/dev/pts/6）写入一帧模拟数据（Python 示例，按当前固定 55B 帧格式构造）：

```bash
python3 - <<'PY'
import sys,struct,os
dev = sys.argv[1] if len(sys.argv)>1 else '/dev/pts/6'
fd = os.open(dev, os.O_WRONLY)
hdr, tail = b'\xAA', b'\xBB'

# RxPayload: < 10f 5h 3b
chf = [12.3, -5.1, 3.2, 0.8] + [0.0]*6   # 示例: knee, vel, torque, curr 在前 4 个
chs = [1] + [0]*4                         # 示例: foot_contact=1 在 chs[0]
chb = [0,0,0]
payload = struct.pack('<10f5h3b', *(chf+chs+chb))
os.write(fd, hdr + payload + tail)
os.close(fd)
PY
```

此时应可在 `/serial_feedback` 看到示例：`knee=12.300, vel=-5.100, foot_contact=1, torque=3.200, curr=0.800`。

2) 验证发送帧（Tx）与分类输入：

- 用一条命令周期发布 `/motion_classification`（顺序与 `TX_CHB_FIELD_LIST` 对齐，例如 `[mode, macro, phase_idx]`）：

```bash
ros2 topic pub /motion_classification std_msgs/Int32MultiArray "{data: [2, 1, 3]}" -r 5
```

- 在虚拟串口的另一端接收并解析（Python 示例）观察 55B 帧头尾与 chb 三个字节：

```bash
python3 - <<'PY'
import sys,os,struct
dev = sys.argv[1] if len(sys.argv)>1 else '/dev/pts/6'
fd = os.open(dev, os.O_RDONLY)
while True:
  hdr = os.read(fd,1)
  if not hdr: break
  if hdr != b'\xAA': continue
  payload = os.read(fd,53)
  tail = os.read(fd,1)
  if len(payload)!=53 or tail!=b'\xBB': continue
  # TxPayload 同布局: <10f5h3b
  vals = struct.unpack('<10f5h3b', payload)
  chb = vals[10+5*1:10+5*1+3]  # 紧随 10f 和 5h 之后的 3b
  print('TX chb (by order in TX_CHB_FIELD_LIST):', chb)
PY
```

> 说明：旧的 `/serial_command` ASCII 话题已不再推荐使用，默认关闭。如需兼容历史测试，可通过 `-p enable_ascii_command:=true` 临时启用。

### 如何修改字段（增删/改名/换序）

打开 `src/my_robot_base/src/serial_comm_node.cpp`，在文件顶部编辑以下列表：

- 发送浮点：`TX_CHF_FIELD_LIST`（例：删除 `X(kd)`、新增 `X(ki)`、或调整 `X(kp)`/`X(kd)` 顺序）
- 发送字节：`TX_CHB_FIELD_LIST`（例：`X(mode) X(macro) X(phase_idx)`）
- 接收浮点：`RX_CHF_FIELD_LIST`（例：`X(knee) X(vel) X(torque) X(curr)`）
- 接收短整：`RX_CHS_FIELD_LIST`（例：`X(foot_contact)`）

改完只需重新编译即可。若 MCU 固定了通道索引，可用参数覆盖映射（如 `tx_chf_kp_idx:=5`）。


## 运动模式分类
  - `0` 平地
  - `1` 上楼
  - `2` 下楼
  - `3` 上坡
  - `4` 下坡
  - `h` 帮助
  - `q` 退出
- QoS：Transient Local（新订阅者能获取最近一次模式）

### 运行

构建（如果尚未构建）：

```bash
colcon build --packages-select my_robot_base --symlink-install
source install/setup.bash
```

运行节点：

```bash
ros2 run my_robot_base keyboard_motion_mode
```

自定义参数：

```bash
ros2 run my_robot_base keyboard_motion_mode --ros-args \
  -p topic_name:=motion_mode \
  -p print_help_on_start:=true \
  -p republish_period_sec:=0.0
```

在另一个终端观察：

```bash
ros2 topic echo /motion_mode
```

> 注意：某些 IDE 的内置终端不支持原生单键读取，程序会自动回退到“需要回车”的行输入模式。
