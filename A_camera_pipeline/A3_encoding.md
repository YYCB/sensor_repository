# A3. 编码（Encode）

## 1. 这篇文章要解决什么问题？

将原始 YUV/RGB 帧压缩为 H.264 / H.265 码流，以降低存储/传输带宽，同时满足延迟、质量和 CPU/GPU 资源约束。

---

## 2. 数据链路图

```
[原始帧：NV12 / I420 / RGB]
        │
        ▼
[编码器选择]
  ┌──────────────────────────────────────────────┐
  │ 硬编（推荐）         │ 软编（备选）            │
  │  NVENC（NVIDIA）     │  x264 / x265（libav）   │
  │  V4L2 M2M（Jetson） │  OpenH264（Cisco）       │
  │  MediaCodec（Android）│ libopenh264             │
  │  Intel QuickSync    │                          │
  └──────────────────────────────────────────────┘
        │
        ▼
[码流输出]
  AnnexB（裸流）/ AVCC（MP4 风格）
        │
        ▼
[封装（Mux）]
  TS / FLV / MP4 / RTP Payload
        │
        ▼
[传输 / 存储]
```

---

## 3. 编码标准对比

| 特性 | H.264 (AVC) | H.265 (HEVC) | AV1 |
|------|-------------|--------------|-----|
| 压缩率 | 基准 | 同质量 ~40% 节省 | 同质量 ~50% 节省 |
| 硬件支持 | 极广 | 广（部分旧设备缺失） | 新一代（2022+） |
| 解码延迟 | 低 | 中 | 较高 |
| 专利费 | 有（H.264 专利池） | 有 | 免费 |
| 推荐用途 | 兼容性优先 | 带宽受限场景 | 未来趋势 |

---

## 4. 关键编码参数

### 4.1 GOP（Group of Pictures）

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `keyint` (IDR 间隔) | 关键帧间隔（帧数） | 30–60（1–2 秒@30fps） |
| `min-keyint` | 最小 IDR 间隔 | 同 `keyint`（固定场景）|
| `bframes` | B 帧数量 | 0（低延迟），2（高压缩） |
| `refs` | 参考帧数 | 1–3 |

**低延迟模式**：`keyint=1`（每帧都是 I 帧）或设置 `tune=zerolatency`（x264/x265）。

### 4.2 码率控制

| 模式 | 说明 | 适用 |
|------|------|------|
| CBR（固定码率） | 输出码率稳定，带宽可预测 | 实时传输（RTP/RTMP） |
| VBR（可变码率） | 质量更优，码率波动 | 本地录制 |
| CQP（固定量化） | 固定质量，码率不可控 | 离线转码 |
| CRF（恒定质量因子） | x264/x265 专有，质量稳定 | 录制存档 |

### 4.3 Profile / Level

| Profile | B帧 | CABAC | 常见设备 |
|---------|-----|-------|---------|
| Baseline | ✗ | ✗ | 旧移动设备、WebRTC |
| Main | ✓ | ✓ | 通用 |
| High | ✓ | ✓ | 高清录制 |

**Level** 决定最大分辨率×帧率（如 Level 4.1 支持 1080p60）。

---

## 5. 平台硬编代码示例

### 5.1 FFmpeg + NVENC
```bash
ffmpeg -f v4l2 -input_format nv12 -video_size 1920x1080 -framerate 30 \
       -i /dev/video0 \
       -c:v h264_nvenc \
       -preset llhq \          # low latency high quality
       -rc cbr \
       -b:v 4M -maxrate 4M -bufsize 4M \
       -g 30 \                 # keyint=30
       -bf 0 \                 # 无 B 帧
       -an \
       -f rtsp rtsp://localhost:8554/camera0
```

### 5.2 FFmpeg + V4L2 M2M（Jetson）
```bash
ffmpeg -f v4l2 -input_format nv12 -video_size 1920x1080 -framerate 30 \
       -i /dev/video0 \
       -c:v h264_v4l2m2m \
       -b:v 4M \
       -g 30 \
       -f rtp rtp://239.0.0.1:5004
```

### 5.3 GStreamer + NVENC（Jetson nvv4l2h264enc）
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  'video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1' ! \
  nvv4l2h264enc bitrate=4000000 iframeinterval=30 \
    preset-level=1 control-rate=1 ! \
  h264parse ! \
  rtph264pay config-interval=1 ! \
  udpsink host=192.168.1.100 port=5004
```

### 5.4 x264 软编（低延迟配置）
```bash
ffmpeg -f v4l2 -i /dev/video0 \
       -c:v libx264 \
       -preset ultrafast \
       -tune zerolatency \
       -x264opts "keyint=30:min-keyint=30:no-scenecut:bframes=0" \
       -b:v 2M \
       -f flv rtmp://localhost:1935/live/camera0
```

---

## 6. AnnexB vs AVCC 格式

| 格式 | Start Code | 用途 |
|------|-----------|------|
| AnnexB | `00 00 00 01` 前缀 | 裸流（TS / RTP / 文件流） |
| AVCC | 4 字节 NALU 长度前缀 | MP4 / MOV / ISO BMFF |

**切换方法（FFmpeg）**：
```bash
# AVCC → AnnexB
ffmpeg -i input.mp4 -c copy -bsf:v h264_mp4toannexb output.ts

# AnnexB → AVCC
ffmpeg -i input.ts -c copy -bsf:v h264_annexb_to_mp4 output.mp4
```

---

## 7. 端到端延迟分解

```
[采集时刻]
    │  曝光 + 读出（~1/fps）
    ▼
[ISP 处理]（~3–10 ms，硬件 ISP）
    │
    ▼
[编码排队]（取决于 B 帧和 lookahead）
    │  CBR 低延迟：< 1 frame
    ▼
[编码处理]（NVENC ~2–5 ms）
    │
    ▼
[封包/Mux]（< 1 ms）
    │
    ▼
[网络传输]（局域网 < 1 ms，广域网变化大）
    │
    ▼
[解码]（硬解 ~5 ms，软解 ~15 ms）
    │
    ▼
[渲染显示]（~16 ms @60Hz）

典型 LAN 端到端（硬编+硬解）：30–80 ms
```

---

## 8. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 编码格式 | H.264 | 兼容性最广 |
| Profile | High | 通用场景 |
| GOP / keyint | 30（1 秒@30fps） | 随机接入与压缩率平衡 |
| B 帧 | 0（低延迟），2（存档） | 低延迟必须为 0 |
| 码率（1080p30） | 4–8 Mbps CBR | 实时传输 |
| 编码器 | NVENC / V4L2 M2M | 优先硬编 |
| 像素格式 | NV12 | 硬编首选 |

---

## 9. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| 编码延迟 | < 2 帧（@30fps < 66 ms） | 硬件时间戳 |
| CPU 占用（硬编） | < 5%（单路 1080p30） | `top` |
| GPU 占用（NVENC） | < 30% | `nvidia-smi` / `tegrastats` |
| VMAF / PSNR | VMAF > 80 | `ffmpeg -filter_complex vmaf` |
| 码率稳定性（CBR） | 偏差 < ±10% | 抓包统计 |

---

## 10. 常见问题与排查步骤（Checklist）

- [ ] 输出花屏/绿屏 → 确认输入像素格式（NV12 stride 是否 64 对齐）
- [ ] 首帧延迟高 → 检查 IDR 帧是否立即生成（`force_key_frames=expr:gte(t,0)`）
- [ ] 码率严重不稳 → 检查 buffer size 配置（CBR 要求 `bufsize = 2×bitrate`）
- [ ] 编码器报错 `CUDA error` → GPU 资源不足或驱动版本不匹配
- [ ] V4L2 M2M 设备找不到 → `ls /dev/video*` 确认 M2M 节点，通常 `/dev/video1` 起
- [ ] x264 CPU 100% → 改 `preset=ultrafast` 或切换硬编
- [ ] SPS/PPS 丢失导致解码器无法解析 → 设置 `config-interval=-1`（GStreamer）或 `-vbsf dump_extra`

---

## 11. 参考资料

- [FFmpeg H.264 编码指南](https://trac.ffmpeg.org/wiki/Encode/H.264)
- [NVENC 编程指南](https://docs.nvidia.com/video-technologies/video-codec-sdk/nvenc-video-encoder-api-prog-guide/)
- [GStreamer nvv4l2h264enc 插件文档](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)
- [x264 参数参考](https://code.videolan.org/videolan/x264)
