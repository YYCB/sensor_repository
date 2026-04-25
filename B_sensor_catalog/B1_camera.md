# B1. 相机（Camera）完整接入指南

## 1. 这篇文章要解决什么问题？

覆盖相机从硬件选型到算法消费的完整接入路径，汇总 MIPI/GMSL/USB/GigE 接口差异、Linux 驱动调试、编码传输选型、标定和常见 Debug 方法。

---

## 2. 数据链路图

```
[Camera Sensor]
    │（MIPI CSI-2 / GMSL2 / USB / GigE）
    ▼
[Linux 驱动层]
  V4L2 / UVC / 厂商 SDK / ISP Bridge
    │
    ▼
[ISP / 预处理]
  去马赛克 → AWB → NR → 格式转换（→ NV12）
    │
    ▼
[编码器]
  NVENC / V4L2 M2M / x264
    │
    ├──► 传输（RTSP / WebRTC / HLS）──► 远端消费
    └──► 本地算法（检测/SLAM/标定）
```

---

## 3. 接口选型矩阵

| 接口 | 带宽上限 | 距离 | 成本 | 典型平台 | 推荐场景 |
|------|----------|------|------|---------|---------|
| USB 2.0 UVC | ~40 MB/s | 5 m | 低 | 通用 PC | 低帧率 / 低分辨率 |
| USB 3.x UVC | ~400 MB/s | 3 m | 低-中 | 通用 PC | 工业 USB 相机 |
| MIPI CSI-2 | 1–16 Gbps | 板级 | 低 | Jetson / RPi | 嵌入式平台 |
| GMSL2 | 6 Gbps | 15 m（同轴） | 高 | 车载 SoC | 车载 ADAS |
| GigE Vision | 1–10 GbE | 100 m | 中-高 | 工业 PC | 工业视觉 |
| Ethernet（RTSP/ONVIF） | 网络限制 | 无限（IP） | 低-中 | 任意 | 监控 IPC |

---

## 4. Linux 驱动调试步骤

### 4.1 快速验证（USB UVC）
```bash
# 枚举设备
v4l2-ctl --list-devices

# 查看支持格式
v4l2-ctl -d /dev/video0 --list-formats-ext

# 取流预览（MJPEG → 本地播放）
ffplay -f v4l2 -input_format mjpeg -video_size 1920x1080 \
       -framerate 30 /dev/video0
```

### 4.2 MIPI CSI-2（Jetson）
```bash
# 检查媒体控制拓扑
media-ctl -d /dev/media0 --print-topology

# 设置 sensor 格式
media-ctl --set-v4l2 '"IMX477 10-001a":0 [fmt:SRGGB10_1X10/1920x1080]'

# V4L2 取流
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1920,height=1080,pixelformat=RG10 \
  --stream-mmap --stream-count=30
```

### 4.3 GMSL2（MAX9295/9296）
```bash
# 检查 Link Lock 状态（I2C 寄存器）
i2cget -y 0 0x48 0x04   # MAX9296 Link Lock 寄存器（示例）

# dmesg 检查初始化日志
dmesg | grep -i "max9296\|gmsl\|link"
```

### 4.4 GigE Vision（Aravis）
```bash
arv-tool-0.8 detect
arv-tool-0.8 -n "Basler-acA1920" set Width=1920 Height=1080
arv-tool-0.8 -n "Basler-acA1920" record --duration=5 output.avi
```

---

## 5. 编码与传输快速配置

### 5.1 RTSP 推流（GStreamer + Jetson）
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  'video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1' ! \
  nvv4l2h264enc bitrate=4000000 iframeinterval=30 ! \
  h264parse ! rtph264pay config-interval=1 ! \
  udpsink host=127.0.0.1 port=5004
```

### 5.2 多路相机同时推流
```bash
# 使用 MediaMTX 配置文件（mediamtx.yml）
paths:
  camera0:
    source: "v4l2:///dev/video0?video_size=1920x1080&framerate=30"
  camera1:
    source: "v4l2:///dev/video2?video_size=1920x1080&framerate=30"
```

---

## 6. 标定

### 6.1 内参标定（单目）
```bash
# 使用 ROS camera_calibration
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

**输出文件**（`camera.yaml`）：
```yaml
camera_matrix:
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  data: [k1, k2, p1, p2, k3]
```

### 6.2 外参标定（多相机 / 相机-雷达）
- 工具：[Kalibr](https://github.com/ethz-asl/kalibr)（多相机 / IMU-Camera）；
- 工具：[cam_lidar_calibration](https://github.com/acfr/cam_lidar_calibration)（相机-激光雷达）；
- 标靶：棋盘格（≥ 8×6）或 AprilTag。

### 6.3 时间同步标定（Camera-IMU）
```bash
# Kalibr IMU-Camera 标定
kalibr_calibrate_imu_camera \
  --target aprilgrid.yaml \
  --imu imu.yaml --imu-models calibrated \
  --cam camchain.yaml \
  --bag data.bag
```

---

## 7. 常见 Debug 场景

### 7.1 黑屏 / 无图像
```
1. dmesg | grep video → 确认驱动加载
2. v4l2-ctl --list-devices → 确认节点存在
3. 确认分辨率/格式匹配（list-formats-ext）
4. MIPI：media-ctl 拓扑是否正确连接？
5. GMSL：Link Lock 是否拉高？（i2cget 检查）
6. GigE：arv-tool detect 能看到相机？防火墙是否开放？
```

### 7.2 花屏 / 图像噪点
```
1. 带宽是否足够？（重新计算未压缩带宽）
2. USB：是否有 Hub？是否共享 USB Host 控制器？
3. MIPI：Lane 速率是否匹配？（dtsi 配置）
4. GigE：MTU 是否一致？丢包率 ping -f 检测
5. 编码：输入格式/stride 是否对齐？
```

### 7.3 丢帧
```
1. v4l2 sequence gap → 采集线程 CPU 占用高，提高优先级（SCHED_FIFO）
2. 队列深度过小 → 增加缓冲 buffer 数
3. 网络丢包 → 检查 switch/cable；启用 NACK/FEC
```

### 7.4 延迟高
```
1. 逐级分解延迟（A3 编码 + A4 传输参考章节）
2. 编码：关闭 B 帧，tune=zerolatency
3. 传输：使用 RTP/WebRTC 替代 HLS
4. 解码：确认使用硬解，无额外缓冲
```

---

## 8. 性能指标与验收标准

| 指标 | 目标值 |
|------|--------|
| 采集帧率偏差 | < 1%（稳态） |
| 端到端延迟（LAN 直播） | < 150 ms |
| 多路丢帧率 | < 0.01% |
| 内参重投影误差 | < 0.5 px |
| 外参旋转误差 | < 0.5° |

---

## 9. 参考资料

- [V4L2 API](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- [Jetson Camera Developer Guide](https://developer.nvidia.com/embedded/jetson-partner-supported-cameras)
- [Kalibr 多相机标定](https://github.com/ethz-asl/kalibr)
- [ROS camera_calibration](http://wiki.ros.org/camera_calibration)
- [MediaMTX（多协议媒体服务器）](https://github.com/bluenviron/mediamtx)
