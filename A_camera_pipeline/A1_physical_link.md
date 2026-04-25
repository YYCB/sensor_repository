# A1. 物理与链路层（Camera → Host）

## 1. 这篇文章要解决什么问题？

选型相机接口并将相机可靠接入主机，涵盖带宽预算、驱动枚举、缓冲与丢帧策略。

---

## 2. 数据链路图

```
[Camera Sensor]
      │  RAW / YUV / MJPEG
      ▼
[物理接口层]
  ┌──────────────────────────────────────────────────┐
  │  USB UVC  │  MIPI CSI-2  │  GMSL2  │  GigE / ETH│
  └──────────────────────────────────────────────────┘
      │
      ▼
[Host 驱动层]
  Linux: V4L2 / UVC / 厂商 SDK / 内核 ISP Bridge
      │
      ▼
[设备节点 / API]
  /dev/videoX  ·  media controller  ·  I2C 设备
      │
      ▼
[缓冲队列]
  MMAP / USERPTR / DMABUF  →  Ring Buffer
      │
      ▼
[上层应用]  ← 帧数据（含 frame_id / timestamp）
```

---

## 3. 关键接口 / 协议 / 格式

| 接口 | 典型带宽 | 最大距离 | 主要用途 |
|------|----------|----------|----------|
| USB 2.0 UVC | 480 Mbps（实际 ~40 MB/s） | 5 m | 低成本 PC 摄像头 |
| USB 3.x UVC | 5 / 10 Gbps | 3 m（无中继） | 工业 USB 相机 |
| MIPI CSI-2 | 1–16 Gbps（Lane × 速率） | PCB 板级，< 30 cm | 移动/嵌入式（Jetson、RPi） |
| GMSL2 | 6 Gbps/lane | 15 m（同轴线） | 车载前装摄像头 |
| GigE Vision | 1 / 2.5 / 10 GbE | 100 m（Cat5e）| 工业机器视觉 |
| Ethernet RTSP/ONVIF | 受网络限制 | 不限（走 IP） | 监控 IPC、IP 相机 |

---

## 4. 带宽预算

```
带宽（bit/s） = 宽 × 高 × 帧率 × bit_depth × 通道数

示例：1920×1080 @ 30fps，10bit，RAW Bayer（1通道）
     = 1920 × 1080 × 30 × 10 ≈ 622 Mbps（未压缩）

YUV420（NV12）≈ 622 × (12/10) / 2 ≈ 373 Mbps
MJPEG 压缩后通常降至 50–150 Mbps（依质量因子）
```

**多路估算**：N 路相机带宽需求 = 单路 × N，需评估总线（USB Hub、PCIe、Ethernet Switch）的实际瓶颈。

---

## 5. 驱动与设备枚举

### 5.1 USB UVC
```bash
# 枚举 UVC 设备
lsusb -t
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext

# 查看能力
v4l2-ctl -d /dev/video0 --all
```

### 5.2 MIPI CSI-2（以 Jetson 为例）
```bash
# 内核驱动一般以 sensor driver + VI（Video Input）+ ISP 三层叠加
# 检查媒体控制拓扑
media-ctl -d /dev/media0 --print-topology

# 设置格式（例如 RAW10 1920x1080）
media-ctl -d /dev/media0 \
  --set-v4l2 '"IMX477 10-001a":0 [fmt:SRGGB10_1X10/1920x1080]'
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=RG10
```

### 5.3 GMSL2（以 MAX9296 Deserializer 为例）
- 需要厂商内核模块（`.ko`）及设备树（DTS）配置 I2C 映射；
- 典型拓扑：`Camera → Serializer(MAX9295) --同轴线--> Deserializer(MAX9296) --> SoC MIPI RX`；
- 调试时先确认 Link Lock（GPIO 状态或 I2C 寄存器读值）。

### 5.4 GigE Vision
```bash
# 使用 aravis（开源 GigE Vision 库）
arv-tool-0.8 detect            # 自动发现网络上的相机
arv-tool-0.8 -n <camera_id> features
```

---

## 6. 取流与缓冲

### 6.1 V4L2 缓冲类型

| 类型 | 说明 | 适用场景 |
|------|------|----------|
| `V4L2_MEMORY_MMAP` | 内核分配，用户 mmap | 通用，最简单 |
| `V4L2_MEMORY_USERPTR` | 用户分配物理连续内存 | 需要与其他子系统共享 |
| `V4L2_MEMORY_DMABUF` | 零拷贝，fd 共享 | GPU / ISP / 编码器互联 |

### 6.2 DMABUF 零拷贝路径
```
V4L2 DMABUF fd  →  Export  →  V4L2 M2M Encoder Import
                          ↘  GPU Texture Import（OpenGL EGL）
```

### 6.3 丢帧策略

| 策略 | 说明 | 延迟特性 |
|------|------|----------|
| 环形覆盖（最新帧） | 丢弃最旧帧，保留最新 | 低延迟，不保证每帧 |
| 阻塞等待 | 队列满时生产者阻塞 | 零丢帧，延迟可能升高 |
| 有界丢弃 | 超过阈值丢帧并记日志 | 折中 |

**推荐**：实时预览/算法流用**环形覆盖**；录制/存档流用**有界丢弃**并告警。

---

## 7. 关键参数与默认值

| 参数 | 推荐值 / 范围 | 说明 |
|------|--------------|------|
| 缓冲队列深度 | 3–6 | 太少丢帧，太多增加延迟 |
| MIPI Lane 数 | 2 / 4 | 依分辨率/帧率选择 |
| I2C 速率 | 100 / 400 kHz | 配置寄存器用 |
| GigE MTU | 8228 bytes（Jumbo Frame） | 降低 CPU 中断，建议开启 |
| USB 传输模式 | Isochronous | UVC 标准，保帧率 |

---

## 8. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| 采集延迟（采样到应用拿帧） | < 2 帧周期（@30fps < 66 ms） | 硬件时间戳比对 |
| 丢帧率 | < 0.1%（稳态） | `v4l2-ctl --stream-count` |
| 驱动 CPU 占用 | < 5%（单路 1080p30） | `top` / `perf` |

---

## 9. 常见问题与排查步骤（Checklist）

- [ ] `lsusb` / `lspci` 能看到设备？→ 硬件/驱动问题
- [ ] `/dev/videoX` 节点存在？→ `dmesg | grep video`
- [ ] `v4l2-ctl --list-formats-ext` 格式符合预期？
- [ ] 带宽是否超过物理接口上限？（重新计算）
- [ ] 多路 USB 时是否经过同一个 USB Host 控制器？（`lspci -tv`）
- [ ] MIPI 时钟是否匹配？（`media-ctl` 打印 pad 格式是否一致）
- [ ] GigE 相机 MTU 与交换机配置是否匹配？
- [ ] DMABUF 是否正确 import/export（dma_buf_fd 合法）？
- [ ] 时间戳单调递增？无时间跳变？

---

## 10. 参考资料

- [Linux Kernel V4L2 Documentation](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- [Jetson MIPI CSI-2 Camera Developer Guide](https://developer.nvidia.com/embedded/jetson-partner-supported-cameras)
- [Aravis GigE Vision Library](https://github.com/AravisProject/aravis)
- [GMSL2 Automotive Camera Interface – Maxim/ADI](https://www.analog.com/en/product-category/gmsl-serdes.html)
