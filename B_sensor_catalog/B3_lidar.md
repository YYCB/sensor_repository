# B3. 激光雷达（LiDAR）接入指南

## 1. 这篇文章要解决什么问题？

完成激光雷达从 UDP 数据包到有效点云帧的全流程接入，包括网络配置、packet → frame 解析、点云格式、时间同步、外参标定和运动畸变补偿。

---

## 2. 数据链路图

```
[LiDAR 硬件]
  旋转/固态，多线激光 + 探测器阵列
        │（UDP / Ethernet）
[网络层]
  专用 LAN / 交换机
        │
[驱动 / SDK 层]
  厂商 ROS Driver / 自研 UDP 解析器
        │
[点云帧组装]
  Packet（~1248 字节/包）→ Frame（一圈 = 一帧，通常 10–20 Hz）
        │
[点云格式]
  [x, y, z, intensity, ring, timestamp]
        │
[后处理]
  ├─ 去畸变（Motion Compensation）
  ├─ 外参变换（Sensor → Vehicle → World）
  └─ 下采样 / 滤波（VoxelGrid / PassThrough）
        │
[上层应用]
  SLAM / 障碍物检测 / 高精地图
```

---

## 3. 网络配置

### 3.1 静态 IP 配置（推荐）
```bash
# 设置主机网卡 IP（与 LiDAR 同网段）
sudo ip addr add 192.168.1.100/24 dev eth1
sudo ip link set eth1 up

# 验证连通性
ping 192.168.1.201  # LiDAR 默认 IP（各型号不同）
```

### 3.2 MTU 与网络优化
```bash
# 开启 Jumbo Frame（推荐，降低 CPU 中断）
sudo ip link set eth1 mtu 9000

# 增大 UDP 接收缓冲区（防丢包）
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# 写入 /etc/sysctl.conf 持久化
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.conf
```

### 3.3 中断亲和性（高点频 LiDAR）
```bash
# 查看网卡队列中断
cat /proc/interrupts | grep eth1

# 绑定中断到指定 CPU
echo 4 > /proc/irq/<IRQ_NUM>/smp_affinity_list
```

---

## 4. Packet → Frame 解析

### 4.1 典型 UDP 包结构（以 Velodyne VLP-16 为例）
```
UDP Payload（1248 字节）：
  [0:1199]   12 个 Data Block，每块 100 字节
    Block:
      [0:1]   Block ID（0xFFEE）
      [2:3]   方位角（0.01°单位）
      [4:99]  32 个通道数据（distance × 2 + intensity × 1）
  [1200:1203] GPS 时间戳（µs since top of hour）
  [1204]      返回模式
  [1205]      产品 ID
```

### 4.2 帧组装逻辑
```python
def assemble_frame(packets):
    """
    当方位角从 ~359° 回绕到 0° 时，认为一帧结束。
    """
    points = []
    prev_angle = None
    for pkt in packets:
        for block in pkt.blocks:
            angle = block.azimuth
            if prev_angle is not None and angle < prev_angle - 180:
                yield points  # 输出完整一帧
                points = []
            points.extend(decode_block(block))
            prev_angle = angle
```

### 4.3 XYZ 坐标计算
```python
import math

def polar_to_xyz(distance_m, azimuth_deg, elevation_deg):
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    x = distance_m * math.cos(el) * math.sin(az)
    y = distance_m * math.cos(el) * math.cos(az)
    z = distance_m * math.sin(el)
    return x, y, z
```

---

## 5. 点云格式

### 5.1 标准点云字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `x`, `y`, `z` | float32 | 3D 坐标（m，传感器坐标系） |
| `intensity` | float32 / uint8 | 反射强度（0–255） |
| `ring` | uint16 | 激光线束编号（0 起） |
| `timestamp` | float64 | 该点的硬件时间戳（s） |
| `distance` | float32 | 原始距离值（m） |
| `return_type` | uint8 | 单/双回波 |

### 5.2 ROS PointCloud2 消息
```
sensor_msgs/PointCloud2
  header.stamp    ← 帧起始时间戳（PTP/GNSS 对齐）
  fields[]:
    name="x"      offset=0  datatype=7(FLOAT32)
    name="y"      offset=4  datatype=7
    name="z"      offset=8  datatype=7
    name="intensity" offset=12 datatype=7
    name="ring"   offset=16 datatype=4(UINT16)
    name="timestamp" offset=20 datatype=8(FLOAT64)
  point_step = 28
  row_step   = point_step × width
```

---

## 6. 时间同步

### 6.1 PTP 同步（推荐）
```bash
# LiDAR 配置为 PTP Slave（通过 Web UI 或配置文件）
# 主机运行 ptp4l（见 A5 章节）
sudo ptp4l -i eth1 -H -m

# 验证 LiDAR 与主机时钟偏差（< 1 µs 理想）
# 通过厂商 Web API 读取 LiDAR 时间偏移
```

### 6.2 GPS/PPS 同步（无 PTP 时）
```bash
# 主机使用 gpsd + chrony 对齐 GNSS 时间
sudo gpsd /dev/ttyS0 -F /var/run/gpsd.sock
# /etc/chrony.conf 中添加：
# refclock SHM 0 offset 0.1 delay 0.2 refid GPS
```

---

## 7. 运动畸变补偿（Motion Compensation）

### 7.1 问题描述
旋转 LiDAR 的一帧数据采集时间约为 100 ms（@10 Hz），期间车辆移动会导致点云畸变。

### 7.2 补偿方法

```python
def compensate_motion(points, T_start, T_end):
    """
    线性插值：假设帧内匀速运动
    T_start, T_end: 帧起止时的 6DOF 位姿（来自 IMU/里程计）
    """
    for p in points:
        alpha = (p.timestamp - T_start.time) / (T_end.time - T_start.time)
        T_interp = interpolate_pose(T_start, T_end, alpha)
        p.xyz = T_interp @ p.xyz  # 变换到帧起始时刻的坐标系
    return points
```

**高精度方案**：使用 IMU 积分得到每个点的精确位姿（见 B4）。

---

## 8. 外参标定

### 8.1 LiDAR → Camera 外参
```bash
# 使用 cam_lidar_calibration
rosrun cam_lidar_calibration run_optimiser \
  --camera_info /camera/camera_info \
  --lidar /velodyne_points \
  --board_dims 0.8 0.6  # 标定板尺寸（m）
```

### 8.2 多 LiDAR 外参对齐
- 工具：[lidar_align](https://github.com/ethz-asl/lidar_align)（自动化 ICP）
- 方法：在静止场景下采集各 LiDAR 点云，最小化 ICP 残差

---

## 9. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 帧率 | 10–20 Hz | 依分辨率要求 |
| UDP 接收缓冲 | 26 MB | `net.core.rmem_max` |
| MTU | 9000 | Jumbo Frame |
| 组帧方式 | 方位角回绕 | 最通用 |
| 时间同步 | PTP | 精度 < 1 µs |

---

## 10. 性能指标与验收标准

| 指标 | 目标值 |
|------|--------|
| 点频 | ≥ 标称值（如 VLP-16：300,000 点/s） |
| 帧率偏差 | < 1%（稳态） |
| UDP 丢包率 | 0%（局域网直连） |
| PTP 时钟偏差 | < 1 µs |
| 外参标定残差（ICP） | < 3 cm |

---

## 11. 常见问题与排查步骤（Checklist）

- [ ] 无点云输出 → `tcpdump -i eth1 udp port 2368`（Velodyne 默认端口）确认 UDP 包到达
- [ ] 点云丢失/不完整 → 检查 `rmem_max`；交换机是否有丢包
- [ ] 点云畸变严重 → 运动补偿是否开启？IMU 时间同步是否正确？
- [ ] 强度值异常 → 检查目标材质反射率；多回波模式设置
- [ ] 坐标系偏差 → 外参标定结果；确认旋转方向（FLU vs FRD）
- [ ] 帧率不稳 → CPU 处理瓶颈；增大接收缓冲；绑定中断到专用 CPU

---

## 12. 参考资料

- [Velodyne VLP-16 用户手册](https://velodynelidar.com/products/puck/)
- [Livox Mid-360 SDK](https://github.com/Livox-SDK/Livox-SDK2)
- [ROS velodyne_driver](https://github.com/ros-drivers/velodyne)
- [lidar_align 外参标定](https://github.com/ethz-asl/lidar_align)
- [cam_lidar_calibration](https://github.com/acfr/cam_lidar_calibration)
