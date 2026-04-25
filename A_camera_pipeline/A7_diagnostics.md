# A7. 诊断与可观测性

## 1. 这篇文章要解决什么问题？

建立贯穿采集→ISP→编码→传输→解码各阶段的可观测体系，快速定位延迟、丢帧、码率不稳等问题。

---

## 2. 可观测层次模型

```
┌─────────────────────────────────────────────────────┐
│ 应用层指标（fps / latency / drop rate / bitrate）    │  ← 业务告警
├─────────────────────────────────────────────────────┤
│ 中间件日志（ROS topic stats / GStreamer pipeline）    │  ← 调试
├─────────────────────────────────────────────────────┤
│ 内核 / 驱动事件（V4L2 seq / dmesg / perf）          │  ← 底层诊断
├─────────────────────────────────────────────────────┤
│ 硬件指标（温度 / 功耗 / 编解码器占用）               │  ← 稳定性评估
└─────────────────────────────────────────────────────┘
```

---

## 3. 各阶段关键监控指标

### 3.1 采集阶段

| 指标 | 说明 | 工具 |
|------|------|------|
| `fps_capture` | 实际采集帧率 | V4L2 `sequence` 字段差值 |
| `v4l2_sequence_gap` | V4L2 buffer sequence 跳变 = 丢帧 | 日志分析 |
| `queue_depth` | 缓冲队列深度 | 应用层统计 |
| `dma_timeout` | DMA 超时次数 | `dmesg` |

```bash
# 检查 V4L2 实际帧率和丢帧
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=300 \
         --stream-to=/dev/null 2>&1 | grep fps
```

### 3.2 编码阶段

| 指标 | 说明 | 工具 |
|------|------|------|
| `encode_fps` | 编码帧率 | 编码器 API 统计 |
| `encode_latency_ms` | 编码耗时 | 帧时间戳差值 |
| `bitrate_kbps` | 实际码率 | 抓包统计 / 编码器回调 |
| `keyframe_interval` | IDR 间隔 | 码流解析 |
| `gpu_enc_util` | GPU 编码引擎占用 | `nvidia-smi` / `tegrastats` |

```bash
# nvidia-smi 查看编码器占用
nvidia-smi dmon -s u -d 1 | grep -E "Enc|Dec"

# Jetson tegrastats
tegrastats --interval 500
```

### 3.3 传输阶段

| 指标 | 说明 | 工具 |
|------|------|------|
| `rtp_packet_loss` | RTP 丢包率 | RTCP RR 报告 |
| `rtp_jitter_ms` | RTP 抖动 | RTCP RR 报告 |
| `network_bitrate` | 实际网络带宽占用 | `iftop` / `nethogs` |
| `rtcp_rtt_ms` | 往返时延 | RTCP SR/RR |

```bash
# 抓取 RTP 包分析丢包
tcpdump -i eth0 -w /tmp/rtp_cap.pcap udp port 5004 &
# 用 Wireshark 分析：Statistics → RTP Streams

# 实时带宽监控
iftop -i eth0 -P
```

### 3.4 解码阶段

| 指标 | 说明 | 工具 |
|------|------|------|
| `decode_fps` | 解码帧率 | 播放器统计 |
| `decode_latency_ms` | 解码耗时 | 时间戳差值 |
| `gpu_dec_util` | GPU 解码引擎占用 | `nvidia-smi` |
| `frame_drops` | 解码器丢帧 | 播放器 API |

### 3.5 硬件指标

| 指标 | 告警阈值 | 工具 |
|------|----------|------|
| CPU 温度 | > 85°C | `sensors` / `cat /sys/class/thermal/thermal_zone*/temp` |
| GPU 温度 | > 90°C | `nvidia-smi` |
| 功耗 | 超 TDP | `tegrastats` / `nvidia-smi` |
| 内存占用 | > 80% | `free -h` |

---

## 4. 日志规范

### 4.1 关键日志事件

```
[CAPTURE ] frame_id=12345 seq=12345 ts_soe_ns=1700000123456789 queue_depth=2
[ENCODE  ] frame_id=12345 ts_in_ns=1700000123460000 ts_out_ns=1700000123464000 latency_ms=4 keyframe=0
[TRANSMIT] frame_id=12345 rtp_ts=3600000 seq=4567 bytes=18234
[DECODE  ] frame_id=12345 ts_in_ns=1700000123565000 ts_out_ns=1700000123569000 latency_ms=4
```

### 4.2 告警触发条件

| 事件 | 条件 | 级别 |
|------|------|------|
| 采集丢帧 | `v4l2_sequence` 跳变 ≥ 1 | WARN |
| 编码延迟高 | 单帧编码 > 2×帧周期 | WARN |
| RTP 丢包 | 连续丢包 > 3 帧 | ERROR |
| 码率偏差 | 实际码率偏离目标 > 20% | WARN |
| 温度过高 | CPU/GPU 温度超过阈值 | ERROR |

---

## 5. 抓包与抓帧

### 5.1 网络抓包
```bash
# 抓取指定摄像头的 RTP 流（端口 5004）
sudo tcpdump -i eth0 -w /tmp/camera0_$(date +%s).pcap \
  'udp port 5004 or udp port 5005' -c 10000

# 使用 tshark 实时统计 RTP 丢包
tshark -i eth0 -f "udp port 5004" \
  -T fields -e rtp.seq -e rtp.timestamp -e frame.time_epoch
```

### 5.2 保存关键帧（GStreamer）
```bash
# 触发保存 IDR 帧（每 100 帧存一次）
gst-launch-1.0 rtspsrc location=rtsp://host/test ! \
  rtph264depay ! h264parse ! tee name=t \
  t. ! queue ! avdec_h264 ! \
     videorate drop-only=true max-rate=1 ! \
     jpegenc ! \
     multifilesink location="/tmp/frame_%05d.jpg" \
  t. ! queue ! filesink location=/tmp/recording.ts
```

### 5.3 编码器统计（FFmpeg）
```bash
ffmpeg -i rtsp://host/test -vf "drawtext=text='%{pts\\:hms}':fontsize=24" \
  -c:v copy -f null - 2>&1 | grep -E "frame|fps|bitrate"
```

---

## 6. 故障注入（混沌测试）

| 注入类型 | 工具 | 命令示例 |
|----------|------|----------|
| 网络限速 | `tc netem` | `tc qdisc add dev eth0 root tbf rate 2mbit burst 32kbit latency 400ms` |
| 网络丢包 | `tc netem` | `tc qdisc add dev eth0 root netem loss 5%` |
| 网络抖动 | `tc netem` | `tc qdisc add dev eth0 root netem delay 100ms 20ms` |
| CPU 降频 | `cpufreq-set` | `cpufreq-set -g powersave` |
| GPU 限频 | `nvidia-smi` | `nvidia-smi -pl 50`（限制功耗上限） |
| 磁盘 I/O 限速 | `cgroup blkio` | 通过 cgroup v2 配置 |

```bash
# 模拟 5% 丢包 + 50ms 延迟（测试 WebRTC/RTSP 容错）
sudo tc qdisc add dev eth0 root netem loss 5% delay 50ms 10ms
# 恢复
sudo tc qdisc del dev eth0 root
```

---

## 7. 监控面板建议（Grafana + Prometheus）

```yaml
# prometheus.yml 采集目标（示意）
scrape_configs:
  - job_name: 'sensor_metrics'
    static_configs:
      - targets: ['localhost:9101']  # 自定义 exporter
    metrics_path: /metrics

# 关键 Gauge/Counter 指标
sensor_capture_fps{sensor="camera_front"}
sensor_encode_latency_ms{sensor="camera_front"}
sensor_rtp_packet_loss_ratio{sensor="camera_front"}
sensor_temperature_celsius{device="gpu"}
```

**推荐面板**：
- 实时 FPS 折线图（每路）
- 端到端延迟热力图
- 码率 vs 目标码率（时间序列）
- 温度/功耗趋势

---

## 8. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 日志级别（生产） | INFO | 关键事件必须记录 |
| 日志级别（调试） | DEBUG | 逐帧日志（性能影响大） |
| 监控采集间隔 | 1 s | Prometheus scrape |
| 丢帧告警窗口 | 10 s | 滑动窗口统计 |
| 抓包保留大小 | 500 MB（环形） | 避免磁盘溢出 |

---

## 9. 常见问题与排查步骤（Checklist）

- [ ] FPS 低于预期 → 先确认采集 FPS，再逐级检查编码/传输/解码
- [ ] 延迟忽高忽低 → 查 `queue_depth` 是否有积压；检查网络 jitter
- [ ] 码率远低于设置 → 检查场景是否变化很少（VBR 节省带宽是正常的）
- [ ] 码率远高于设置 → CBR `bufsize` 配置错误；检查编码器参数
- [ ] 日志中出现 sequence gap → 内核丢帧，检查采集线程实时性（降低 CPU 负载）
- [ ] Grafana 指标缺失 → 检查 exporter 是否正常；Prometheus 配置是否正确
- [ ] 故障注入后恢复慢 → 检查 jitter buffer 大小；NACK/FEC 配置是否合理

---

## 10. 参考资料

- [Linux tc netem 文档](https://www.linux.org/docs/man8/tc-netem.html)
- [Prometheus + Grafana 最佳实践](https://prometheus.io/docs/practices/naming/)
- [GStreamer 调试与性能分析](https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html)
- [FFmpeg 统计与分析](https://ffmpeg.org/ffprobe.html)
- [Wireshark RTP 分析](https://wiki.wireshark.org/RTP)
