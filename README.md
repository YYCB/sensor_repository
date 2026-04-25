# 传感器接入知识库（Sensor Integration Repository）

> 沉淀**接入流程、数据链路、编码传输、标定与同步、驱动与诊断、性能与稳定性**等可复用知识。

## 覆盖传感器类型

| 传感器 | 接口 |
|--------|------|
| 相机（Camera） | USB UVC、MIPI CSI-2、GMSL2、GigE Vision、Ethernet（RTSP/ONVIF） |
| 毫米波雷达（mmWave Radar） | CAN、Ethernet、UART |
| 激光雷达（LiDAR） | UDP/Ethernet |
| IMU / GNSS | SPI、I2C、UART、CAN |
| 超声波 / 麦克风阵列 | GPIO、I2C、USB |

---

## 目录结构

```
sensor_repository/
├── README.md                  ← 本文件（导航入口）
│
├── A_camera_pipeline/         ← 相机 → 编码 → 传输端到端流程
│   ├── A1_physical_link.md    ← 物理与链路层（Camera → Host）
│   ├── A2_image_pipeline.md   ← 图像链路（ISP / 预处理）
│   ├── A3_encoding.md         ← 编码（H.264 / H.265）
│   ├── A4_transport.md        ← 传输协议（RTP/RTSP/WebRTC/HTTP）
│   ├── A5_clock_sync.md       ← 时钟同步与触发（GPIO / PTP）
│   ├── A6_decode_render.md    ← 解码、渲染与下游算法
│   └── A7_diagnostics.md      ← 诊断与可观测性
│
├── B_sensor_catalog/          ← 传感器分类知识库
│   ├── B1_camera.md           ← 相机全流程
│   ├── B2_mmwave_radar.md     ← 毫米波雷达
│   ├── B3_lidar.md            ← 激光雷达
│   ├── B4_imu_gnss.md         ← IMU / GNSS
│   └── B5_ultrasonic.md       ← 超声波 / 麦克风阵列
│
└── C_template/
    └── article_template.md    ← 统一贡献模板
```

---

## 贡献方式

每篇内容建议按以下结构组织（详见 [C_template/article_template.md](C_template/article_template.md)）：

1. 这篇文章要解决什么问题？
2. 数据链路图（从物理线到应用）
3. 关键接口 / 协议 / 格式（列清单）
4. 关键参数与默认值（表格）
5. 性能指标与验收标准
6. 常见问题与排查步骤（Checklist）
7. 参考资料 / 链接 / 抓包样例 / 配置片段

---

## 统一术语表

| 术语 | 说明 |
|------|------|
| MIPI CSI-2 | Mobile Industry Processor Interface Camera Serial Interface 2 |
| GMSL2 | Gigabit Multimedia Serial Link 2（车载长距离相机接口） |
| GigE Vision | 基于千兆以太网的工业相机标准 |
| H.264 / AVC | Advanced Video Coding |
| H.265 / HEVC | High Efficiency Video Coding |
| RTP | Real-time Transport Protocol |
| RTSP | Real Time Streaming Protocol |
| WebRTC | Web Real-Time Communication |
| HLS | HTTP Live Streaming |
| HTTP-FLV | HTTP + FLV 格式的流媒体 |
| PTP / IEEE 1588 | Precision Time Protocol（精确时间协议） |
| GPIO / TTL | 硬触发同步信号 |
| CAN | Controller Area Network |
| V4L2 | Video4Linux2（Linux 视频子系统） |
| DMABUF | DMA Buffer（零拷贝内存共享） |
| NVENC | NVIDIA 硬件视频编码器 |
| V4L2 M2M | V4L2 Memory-to-Memory（平台硬编码器接口） |

---

## 快速导航

- **相机端到端流程** → [A_camera_pipeline/](A_camera_pipeline/)
- **各类传感器接入** → [B_sensor_catalog/](B_sensor_catalog/)
- **写文章用的模板** → [C_template/article_template.md](C_template/article_template.md)