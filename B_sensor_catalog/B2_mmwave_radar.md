# B2. 毫米波雷达（mmWave Radar）接入指南

## 1. 这篇文章要解决什么问题？

完成毫米波雷达从硬件接入到点云/目标数据消费的全流程，包括 FMCW 基本原理、接口接入、协议解析、坐标系统一和标定。

---

## 2. 数据链路图

```
[mmWave Radar 硬件]
  （FMCW 天线阵列）
        │
[接口层]
  CAN / Ethernet / UART / USB
        │
[驱动 / SDK 层]
  厂商驱动 or 自研协议解析器
        │
[数据格式层]
  ├─ 目标列表（Object List）：ID、距离、速度、方位角、RCS
  ├─ 点云（Point Cloud）：[x, y, z, v_r, intensity]
  └─ 栅格地图（Occupancy Grid）（部分型号）
        │
[后处理]
  坐标系转换（雷达系 → 车体系 → 世界系）
  时间戳对齐（PTP / CAN 时间戳）
        │
[上层应用]
  障碍物检测 / 目标追踪 / 自动驾驶感知融合
```

---

## 3. FMCW 基本原理

### 3.1 关键参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| 工作频段 | 77 GHz（车载主流）/ 24 GHz | 76–81 GHz |
| 带宽（B） | 影响距离分辨率 | 1–4 GHz |
| 距离分辨率 | `c / (2B)` | ~5 cm（4 GHz 带宽） |
| 最大探测距离 | 依发射功率 | 近程 30 m / 远程 250 m |
| 速度分辨率 | `λ / (2 × N_chirps × T_chirp)` | ~0.1 m/s |
| 最大无模糊速度 | `λ / (4 × T_chirp)` | ±20–±80 m/s |
| 角度分辨率 | ~1–2°（水平，依天线数） | — |

### 3.2 距离-速度测量原理（简化）
```
发射：锯齿波调频信号（FMCW Chirp）
接收：与发射信号混频 → 差频（IF 信号）
  距离 ↔ IF 频率（Range FFT）
  速度 ↔ 相位差（Doppler FFT）
  角度 ↔ 天线相位差（Angle FFT）
```

---

## 4. 接口接入

### 4.1 CAN 接口

```bash
# 配置 CAN 接口（500 kbps）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 抓取 CAN 报文
candump can0

# 过滤特定 ID（如 0x200–0x20F 为目标列表）
candump can0,200:FF0
```

**典型 CAN 报文结构（示意，依型号而定）：**
```
CAN ID: 0x200  DLC: 8
Byte[0:1]  目标ID (uint16)
Byte[2:3]  距离  (uint16, 分辨率 0.01 m)
Byte[4:5]  相对速度 (int16, 分辨率 0.01 m/s)
Byte[6:7]  方位角  (int16, 分辨率 0.01°)
```

### 4.2 Ethernet（UDP）

```python
import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7778))  # 厂商指定端口

while True:
    data, addr = sock.recvfrom(65535)
    # 解析帧头
    header_fmt = ">HHI"  # magic, frame_id, timestamp_ms
    magic, frame_id, ts_ms = struct.unpack_from(header_fmt, data, 0)
    # 解析目标列表...
```

### 4.3 UART / USB

```bash
# 打开串口（115200，8N1）
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb
cat /dev/ttyUSB0 | hexdump -C

# Python pyserial
import serial
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
frame = ser.read(256)  # 读取一帧（长度依协议）
```

---

## 5. 数据格式与坐标系

### 5.1 点云格式

```cpp
struct RadarPoint {
    float x;          // 前向（m）
    float y;          // 左向（m）
    float z;          // 上向（m）
    float v_r;        // 径向速度（m/s，正为远离）
    float rcs;        // 雷达截面积（dBsm）
    uint8_t snr;      // 信噪比
    uint64_t ts_ns;   // 时间戳（ns）
};
```

### 5.2 目标列表格式

```cpp
struct RadarObject {
    uint16_t id;
    float dist_m;          // 距离
    float azimuth_deg;     // 水平方位角（度）
    float elevation_deg;   // 垂直仰角（度，若支持）
    float vel_ms;          // 径向速度（m/s）
    float rcs_dbsm;        // RCS
    uint8_t dynamic_state; // 0=静止, 1=移动, 2=驶离
};
```

### 5.3 坐标系约定

```
雷达坐标系（FLU）：
  X → 正前方
  Y → 正左方
  Z → 正上方

转换到车体坐标系：
  T_body_radar = [R | t]（外参，需标定）
```

---

## 6. 误差与标定

### 6.1 外参标定（雷达 → 车体）

- **方法 1**：使用角反射器（Trihedral Reflector）在标定场中测量；
- **方法 2**：与相机点云融合对齐（ICP）；
- 精度目标：位置 < 5 cm，角度 < 0.5°。

### 6.2 速度偏差校正

```python
# 利用自车速度（来自轮速计/GNSS）校正雷达速度偏差
# 理论：静止目标的雷达径向速度 = -v_ego * cos(azimuth)
def correct_velocity(v_r_measured, v_ego, azimuth_rad):
    v_r_expected = -v_ego * math.cos(azimuth_rad)
    bias = v_r_measured - v_r_expected  # 速度偏差
    return v_r_measured - bias
```

### 6.3 时间延迟标定

- CAN：报文时间戳与 CAN 总线调度延迟（通常 < 1 ms）；
- 以太网：使用 PTP 同步（见 A5）；
- 方法：与 IMU 加速度计数据互相关估计延迟。

---

## 7. 性能指标与验收标准

| 指标 | 目标值 |
|------|--------|
| 探测距离（近程） | ≥ 0.2 m |
| 探测距离（远程） | ≥ 100 m（行人）；≥ 200 m（车辆） |
| 距离精度 | ≤ 0.1 m（1σ） |
| 速度精度 | ≤ 0.1 m/s（1σ） |
| 方位角精度 | ≤ 1°（1σ） |
| 更新率 | ≥ 10 Hz |
| CAN 总线负载 | < 60% |

---

## 8. 常见问题与排查步骤（Checklist）

- [ ] 无数据输出 → 检查电源电压（典型 12 V）；CAN 终端电阻（120 Ω）是否接好
- [ ] 目标距离偏差大 → 检查安装高度/角度是否与标定一致
- [ ] 速度方向反向 → 检查坐标系符号约定（进/离）
- [ ] 角度偏差 → 重新进行外参标定
- [ ] 检测不到静止目标 → 多数 77 GHz 雷达默认过滤零速目标，检查静止目标模式配置
- [ ] CAN 报文缺失 → `candump` 检查总线负载；是否有 Bus-Off？
- [ ] 幻象目标（Ghost） → 减少多路径反射（调整安装位置）；使用滤波器

---

## 9. 参考资料

- [TI mmWave SDK 文档](https://www.ti.com/tool/MMWAVE-SDK)
- [Continental ARS 系列接口手册](https://www.continental-automotive.com/en-gl/Passenger-Cars/Safety/Advanced-Driver-Assistance-Systems/Radar-Sensors)
- [ROS radar_msgs 消息格式](https://github.com/ros-perception/radar_msgs)
- [FMCW 雷达原理（TI 应用笔记）](https://www.ti.com/lit/wp/spyy005a/spyy005a.pdf)
