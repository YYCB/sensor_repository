# B5. 超声波 / 麦克风阵列接入指南

## 1. 这篇文章要解决什么问题？

完成超声波测距传感器和麦克风阵列的驱动接入、数据解析、同步策略及常见问题排查。

---

## 2. 数据链路图

### 2.1 超声波传感器
```
[超声波换能器]
  发射 40 kHz 脉冲 → 接收回波
        │
[接口层]
  GPIO（Trig/Echo）/ I2C / CAN / UART
        │
[驱动 / 解析层]
  飞行时间（ToF）= 回波时间 / 2 × 声速
        │
[数据字段]
  距离（m）、时间戳（ns）、传感器ID
        │
[后处理]
  多传感器融合、盲区处理、温度补偿
        │
[上层应用]
  泊车辅助、防碰撞、近距离感知
```

### 2.2 麦克风阵列
```
[麦克风阵列]
  PDM / I2S / USB 音频
        │
[音频驱动层]
  ALSA / PortAudio / libusb
        │
[数字信号处理]
  波束成形（Beamforming）→ 降噪（NS）→ 声源定向（DOA）
        │
[输出]
  增强语音、声源方向角（方位/仰角）
        │
[上层应用]
  语音唤醒 / 识别 / 声源追踪
```

---

## 3. 超声波传感器

### 3.1 接口类型对比

| 接口 | 典型传感器 | 特点 |
|------|-----------|------|
| GPIO Trig/Echo | HC-SR04 | 最简单，需要精确计时 |
| I2C | MB1242 | 多传感器共享总线 |
| UART | MaxSonar EZ4 | 连续输出，TTL 电平 |
| CAN | 车载 USS 模块 | 多传感器总线，OEM 首选 |
| PWM | Maxbotix | 脉宽编码距离 |

### 3.2 GPIO 接口（HC-SR04）

**硬件连接**：
```
VCC  → 5V
GND  → GND
TRIG → MCU GPIO_OUT（10 µs 脉冲触发）
ECHO → MCU GPIO_IN（回波宽度 = 飞行时间）
```

**Linux GPIO 读取（libgpiod）**：
```c
#include <gpiod.h>
#include <time.h>

struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
struct gpiod_line *trig = gpiod_chip_get_line(chip, 23);
struct gpiod_line *echo = gpiod_chip_get_line(chip, 24);

gpiod_line_request_output(trig, "ultrasonic", 0);
gpiod_line_request_input(echo, "ultrasonic");

// 触发：10 µs 高脉冲
gpiod_line_set_value(trig, 1);
usleep(10);
gpiod_line_set_value(trig, 0);

// 等待回波上升沿
struct timespec t_start, t_end;
while (gpiod_line_get_value(echo) == 0);
clock_gettime(CLOCK_MONOTONIC, &t_start);
while (gpiod_line_get_value(echo) == 1);
clock_gettime(CLOCK_MONOTONIC, &t_end);

double elapsed_us = (t_end.tv_nsec - t_start.tv_nsec) / 1000.0
                  + (t_end.tv_sec - t_start.tv_sec) * 1e6;
// 声速 343 m/s（20°C），单程距离
double distance_m = elapsed_us * 1e-6 * 343.0 / 2.0;
```

**注意**：用户态 GPIO 时序抖动较大（> 10 µs），推荐使用 MCU 或 FPGA 进行精确计时。

### 3.3 I2C 接口（MB1242）
```python
import smbus2, time

bus = smbus2.SMBus(1)
ADDR = 0x70  # MB1242 默认地址

def read_distance_cm():
    bus.write_byte(ADDR, 0x51)  # 触发测量
    time.sleep(0.08)            # 等待 80 ms
    high = bus.read_byte_data(ADDR, 0xE1)
    low  = bus.read_byte_data(ADDR, 0xE2)
    return (high << 8 | low)    # 单位：cm

while True:
    dist = read_distance_cm()
    print(f"距离: {dist / 100:.2f} m")
    time.sleep(0.1)
```

### 3.4 温度补偿
```
声速 c（m/s）= 331.5 + 0.607 × T（°C）

// 修正后距离
distance_m = elapsed_s × c / 2
```

### 3.5 多传感器防串扰（时分复用）
```python
# 依次触发各传感器，间隔足够时间防止串扰
SENSORS = [0, 1, 2, 3, 4, 5, 6, 7]  # 8 路超声波
INTERVAL_MS = 25  # 每路间隔 25 ms

for sensor_id in SENSORS:
    trigger(sensor_id)
    time.sleep(INTERVAL_MS / 1000.0)
    dist = read_echo(sensor_id)
    publish(sensor_id, dist)
```

---

## 4. 超声波性能指标与验收标准

| 指标 | 典型值 | 说明 |
|------|--------|------|
| 量程 | 0.02–4 m（HC-SR04）；0.2–15 m（车载 USS） | 依型号 |
| 盲区 | 0.02–0.3 m | 近距离无法测量 |
| 精度 | ±1–3 mm（近距）；±1%（远距） | 温度补偿后 |
| 更新率 | 10–50 Hz | 依测量周期 |
| 视角 | 15°–40°（半角） | 依换能器设计 |
| 温度范围 | -40°C–85°C | 车规级 |

---

## 5. 麦克风阵列

### 5.1 接口类型对比

| 接口 | 协议 | 特点 |
|------|------|------|
| I2S | 数字 PCM | 嵌入式平台首选（Jetson / RPi） |
| PDM | Pulse Density Modulation | 单线，需 PDM→PCM 转换 |
| USB | UAC 2.0 | 即插即用（ReSpeaker USB） |
| Ethernet | AVB / AES67 | 专业音频，远距离 |

### 5.2 ALSA 配置与采集（I2S / USB）
```bash
# 查看音频设备
aplay -l
arecord -l

# 录制测试（16 kHz，立体声，PCM16）
arecord -D hw:1,0 -f S16_LE -r 16000 -c 2 -d 5 test.wav

# 查看实时电平
alsamixer
```

### 5.3 Python 音频采集（PortAudio / PyAudio）
```python
import pyaudio, numpy as np

SAMPLE_RATE = 16000
CHANNELS    = 4       # 4 路麦克风阵列
CHUNK_SIZE  = 1024
FORMAT      = pyaudio.paInt16

pa = pyaudio.PyAudio()
stream = pa.open(
    format=FORMAT, channels=CHANNELS,
    rate=SAMPLE_RATE, input=True,
    frames_per_buffer=CHUNK_SIZE
)

while True:
    raw = stream.read(CHUNK_SIZE, exception_on_overflow=False)
    data = np.frombuffer(raw, dtype=np.int16)
    # data shape: (CHUNK_SIZE × CHANNELS,) → reshape → (CHUNK_SIZE, CHANNELS)
    frames = data.reshape(-1, CHANNELS)
    # 送入波束成形 / DOA 算法
    process_frames(frames)
```

### 5.4 波束成形（Delay-and-Sum，简化示例）
```python
import numpy as np

def delay_and_sum_beamform(frames, mic_positions, target_angle_deg,
                            sample_rate=16000, sound_speed=343.0):
    """
    frames: (N_samples, N_mics)
    mic_positions: (N_mics, 2) 坐标（m）
    """
    angle_rad = np.deg2rad(target_angle_deg)
    direction = np.array([np.cos(angle_rad), np.sin(angle_rad)])

    delays_s = -mic_positions @ direction / sound_speed  # 每个麦克风的延迟
    delays_samples = np.round(delays_s * sample_rate).astype(int)

    output = np.zeros(frames.shape[0])
    for i, d in enumerate(delays_samples):
        output += np.roll(frames[:, i], d)
    return output / frames.shape[1]
```

### 5.5 声源定向（DOA，以 SRP-PHAT 为例）
```bash
# 使用 ODAS（Open embeddeD Audition System）
# https://github.com/introlab/odas
# 配置 odas.cfg 指定麦克风位置
odaslive -c odas.cfg  # 输出实时 DOA 角度（JSON 流）
```

---

## 6. 时间同步

### 6.1 音视频同步
- 音频硬件时钟（ALSA `DMA_CLOCK`）与 PTP 之间需要 `phc2sys` 对齐；
- 记录每个音频块的 `capture_timestamp`：

```python
import time
timestamp_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
# 或使用 ALSA `snd_pcm_status_get_htstamp` 获取硬件时间戳
```

### 6.2 多阵列同步
- 使用 AVB（Audio Video Bridging）或 AES67 协议统一时钟；
- 简单方案：所有阵列接同一 USB Hub，利用 USB SOF 同步。

---

## 7. 常见问题与排查步骤（Checklist）

**超声波**：
- [ ] 距离为 0 或最大值 → 检查 TRIG 脉冲宽度（需 ≥ 10 µs）；ECHO 线是否浮空
- [ ] 距离抖动大 → 添加中位数滤波；检查反射面是否平整
- [ ] 多传感器串扰 → 确认时分复用间隔足够（≥ 20 ms）
- [ ] 近距盲区问题 → 更换量程更小的型号（如 VL53L1X TOF 激光测距）
- [ ] 温度变化导致偏差 → 添加温度传感器，动态修正声速

**麦克风阵列**：
- [ ] 无音频输入 → `arecord -l` 确认设备；检查权限（`audio` 组）
- [ ] 采集有噪声/爆音 → 降低麦克风增益；检查电源纹波
- [ ] DOA 角度不准 → 重新标定麦克风位置；检查延迟补偿
- [ ] 音视频不同步（> 200 ms） → 检查音频缓冲区大小；时间戳对齐策略

---

## 8. 参考资料

- [HC-SR04 数据手册](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- [libgpiod 文档](https://libgpiod.readthedocs.io/)
- [ODAS 声源定向系统](https://github.com/introlab/odas)
- [ReSpeaker 麦克风阵列](https://wiki.seeedstudio.com/ReSpeaker_6-Mic_Circular_Array_kit_for_Raspberry_Pi/)
- [ROS ultrasonic_sensor 驱动](https://github.com/ros-drivers/ultrasonic_sensor)
- [AES67 音频网络标准](https://www.aes.org/publications/standards/search.cfm?docID=96)
