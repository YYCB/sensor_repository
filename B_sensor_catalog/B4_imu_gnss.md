# B4. IMU / GNSS 接入指南

## 1. 这篇文章要解决什么问题？

完成 IMU 和 GNSS 的接入、时间同步、噪声模型建立、与相机/LiDAR 的外参标定，以及为 EKF/因子图融合提供合格的时间戳数据。

---

## 2. 数据链路图

```
[IMU（加速度计 + 陀螺仪）]          [GNSS 接收机]
   SPI / I2C / UART / CAN               UART / USB / Ethernet
        │                                       │
[驱动层]                              [NMEA / u-blox 协议解析]
  Linux IIO / 厂商 SDK                         │
        │                                       │
[数据字段]                            [数据字段]
  ts_ns, ax, ay, az                     ts_ns, lat, lon, alt
  wx, wy, wz, temp                      heading, speed, fix_quality
        │                                       │
        └──────────────┬────────────────────────┘
                       ▼
             [时间同步（PTP / PPS）]
                       │
                       ▼
             [融合算法（EKF / 因子图）]
             ├─ IMU 预积分（高频 ~200 Hz）
             └─ GNSS 位置更新（低频 ~1–10 Hz）
```

---

## 3. IMU 接口与驱动

### 3.1 接口类型

| 接口 | 典型速率 | 适用 IMU |
|------|----------|---------|
| SPI | 10–50 MHz | 高速 MEMS（ICM-42688-P）|
| I2C | 100–400 kHz | 低速原型（MPU-6050）|
| UART | 115200–4M baud | 战术级（VectorNav）|
| CAN | 1 Mbps | 车载 IMU（Bosch SMI）|
| USB | 12 Mbps | 工业 IMU（Xsens MTi）|

### 3.2 Linux IIO 驱动
```bash
# 查看 IIO 设备
ls /sys/bus/iio/devices/
cat /sys/bus/iio/devices/iio:device0/name  # 芯片名称

# 读取加速度（原始值需乘以 scale）
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
cat /sys/bus/iio/devices/iio:device0/in_accel_scale  # m/s² per LSB

# 配置采样率
echo 200 > /sys/bus/iio/devices/iio:device0/sampling_frequency
```

### 3.3 UART IMU（VectorNav / Xsens）
```python
import serial, struct

ser = serial.Serial("/dev/ttyUSB0", 921600, timeout=0.1)

def parse_vnymr(line: bytes):
    """解析 VectorNav VNYMR 报文"""
    # $VNYMR,yaw,pitch,roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyrX,GyrY,GyrZ*checksum
    fields = line.decode().strip().split(",")
    yaw, pitch, roll = float(fields[1]), float(fields[2]), float(fields[3])
    ax, ay, az = float(fields[7]), float(fields[8]), float(fields[9])
    wx, wy, wz = float(fields[10]), float(fields[11]), float(fields[12].split("*")[0])
    return yaw, pitch, roll, ax, ay, az, wx, wy, wz
```

---

## 4. GNSS 接口与协议

### 4.1 NMEA 协议解析
```python
import pynmea2, serial

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
while True:
    line = ser.readline().decode("ascii", errors="replace").strip()
    if line.startswith("$G"):
        try:
            msg = pynmea2.parse(line)
            if isinstance(msg, pynmea2.GGA):
                print(f"lat={msg.latitude:.7f} lon={msg.longitude:.7f} "
                      f"alt={msg.altitude}m fix={msg.gps_qual}")
        except pynmea2.ParseError:
            pass
```

### 4.2 u-blox UBX 协议（高精度）
```bash
# 配置 u-blox M9N 输出 UBX-NAV-PVT（位置/速度/时间）
ubxtool -p CFG-MSG,0x01,0x07,1 /dev/ttyACM0

# 读取高精度时间戳
ubxtool -p CFG-TP5 /dev/ttyACM0  # 配置 PPS 输出
```

### 4.3 RTK GNSS（差分增强）
- 基站：搭建本地 NTRIP 基站 or 订阅公共 CORS 网络；
- 流动站：通过串口或以太网接收 RTCM3 差分数据；
- 精度：水平 1–2 cm（Fix 状态），垂直 2–3 cm。

---

## 5. 噪声模型与 Allan 方差

### 5.1 IMU 噪声参数

| 参数 | 符号 | 单位 | 典型值（MEMS） |
|------|------|------|--------------|
| 加速度计噪声密度 | σ_a | m/s²/√Hz | 0.001–0.01 |
| 陀螺仪噪声密度 | σ_g | rad/s/√Hz | 0.0001–0.001 |
| 加速度计随机游走 | σ_ba | m/s²/s^0.5 | 0.0001–0.001 |
| 陀螺仪随机游走 | σ_bg | rad/s/s^0.5 | 0.000001–0.00001 |

### 5.2 Allan 方差标定
```bash
# 使用 imu_utils（ROS）进行 Allan 方差分析
# 静止采集 2–6 小时 IMU 数据，然后：
rosrun imu_utils imu_an \
  -imu_topic /imu/data_raw \
  -imu_frequency 200 \
  -dev_name xsens_mti \
  -ave_num 400
```

---

## 6. 时间同步

### 6.1 PPS + NMEA 授时
```bash
# GNSS 输出 PPS 信号接到主机 GPIO / DCD 引脚
# 使用 chrony + PPS 驱动对齐系统时钟
# /etc/chrony.conf:
refclock PPS /dev/pps0 lock GNSS refid PPS precision 1e-9
refclock SHM 0 offset 0.5 delay 0.2 refid GNSS

# 检查同步状态
chronyc sources -v
```

### 6.2 IMU 硬件时间戳
```bash
# 支持硬件时间戳的 IMU（如 Xsens MTi-670）：
# 启用 SyncOut（PPS 同步输出）对齐到 GNSS PPS
# 在 ROS driver 配置中：
#   syncOutMode: PPS
#   syncInSkipFactor: 0
```

### 6.3 时间戳质量要求（融合算法输入）

| 要求 | 说明 |
|------|------|
| 单调递增 | 不允许时间回跳 |
| 时间戳精度 | IMU < 1 ms；GNSS < 100 µs（PPS 授时） |
| 连续性 | 允许短暂中断（< 0.5 s），需标记 gap |
| 时钟漂移 | < 50 ppm（需温度补偿 TCXO） |

---

## 7. Camera-IMU 外参标定

```bash
# Kalibr 标定（棋盘格靶标 + 手持激励运动）
kalibr_calibrate_cameras \
  --target april_6x6.yaml \
  --bag calib.bag \
  --models pinhole-equi \
  --topics /camera/image_raw

kalibr_calibrate_imu_camera \
  --target april_6x6.yaml \
  --imu imu.yaml \
  --imu-models calibrated \
  --cam camchain.yaml \
  --bag calib.bag
```

**输出**（`imu_camera_calibration.yaml`）：
```yaml
cam0:
  T_cam_imu:    # 4×4 变换矩阵（IMU → Camera）
    - [r00, r01, r02, tx]
    - [r10, r11, r12, ty]
    - [r20, r21, r22, tz]
    - [  0,   0,   0,  1]
  timeshift_cam_imu: -0.005123  # 秒，正值表示 camera 时间滞后
```

---

## 8. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| IMU 采样率 | 200–400 Hz | 融合算法通常需要 ≥ 100 Hz |
| GNSS 更新率 | 1–10 Hz | RTK 通常 10 Hz |
| 加速度计量程 | ±4 g（动态驾驶）| 激烈机动用 ±16 g |
| 陀螺仪量程 | ±250°/s（慢速）；±2000°/s（高动态）| — |
| PPS 精度 | < 100 ns | 高精度 GNSS 授时 |

---

## 9. 性能指标与验收标准

| 指标 | 目标值 |
|------|--------|
| IMU 时间戳抖动 | < 1 ms |
| GNSS 定位精度（RTK Fix） | 水平 < 2 cm |
| Camera-IMU 时间偏差 | < 1 ms（Kalibr 标定后） |
| EKF 融合输出频率 | ≥ 100 Hz |

---

## 10. 常见问题与排查步骤（Checklist）

- [ ] IMU 数据全零 → 检查 I2C/SPI 地址；`i2cdetect -y 1` 扫描总线
- [ ] 加速度计静态偏置大 → 重新标定零偏（6 面静止标定法）
- [ ] 时间戳不单调 → 检查时钟源；使用硬件时间戳替代软件时间戳
- [ ] GNSS 无定位（Fix） → 检查天线视野（仰角 > 15°）；信噪比 SNR > 35 dBHz
- [ ] Camera-IMU 外参误差大 → 增加激励运动幅度；检查标靶质量
- [ ] EKF 发散 → 检查噪声参数（Allan 方差）；初始化阶段静止时间是否足够

---

## 11. 参考资料

- [Xsens MTi 系列文档](https://www.xsens.com/products)
- [VectorNav IMU 文档](https://www.vectornav.com/docs)
- [Kalibr IMU-Camera 标定](https://github.com/ethz-asl/kalibr)
- [imu_utils Allan 方差](https://github.com/gaowenliang/imu_utils)
- [u-blox ZED-F9P RTK 模块](https://www.u-blox.com/en/product/zed-f9p-module)
- [chrony PPS 配置](https://chrony.tuxfamily.org/doc/4.3/chrony.conf.html)
