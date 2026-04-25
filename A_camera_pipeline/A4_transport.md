# A4. 传输协议与工作流

## 1. 这篇文章要解决什么问题？

梳理从编码器输出到消费端的四种主流传输路径（RTP/RTSP、WebRTC、HTTP-FLV/HLS、HTTP 上传），明确各路径的工作流向、适用场景与关键注意事项。

---

## 2. 传输协议总览

```
[Encoder 码流输出]
        │
        ├──► RTP/RTSP ──────► 局域网监控 / 机器人内网预览
        │
        ├──► WebRTC ─────────► 远程遥操作 / 实时视频对讲
        │
        ├──► HTTP-FLV / HLS ─► 大规模分发 / 可回放
        │
        └──► HTTP Upload ────► 事件上报 / 离线取证 / 数据回传
```

---

## A4.1 RTP / RTSP

### 工作流向
```
Camera/Encoder
    │ 打包 RTP（H.264/H.265 RTP Payload）
    ▼
RTSP Server（会话控制：DESCRIBE/SETUP/PLAY/TEARDOWN）
    │ UDP（默认）或 TCP（穿防火墙）
    ▼
Client（VLC / GStreamer / ffplay / 算法订阅者）
    │ Jitter Buffer → 解封包 → 解码
    ▼
消费（显示 / 算法）
```

### 常用实现

| 工具 | 角色 | 命令示例 |
|------|------|----------|
| GStreamer `rtsp-server` | 服务端 | 见下方示例 |
| MediaMTX（旧称 rtsp-simple-server） | 服务端 | 配置文件驱动 |
| FFmpeg | 推流客户端 | `ffmpeg ... -f rtsp rtsp://host/path` |
| VLC | 播放客户端 | `vlc rtsp://host:8554/camera0` |
| GStreamer `rtspsrc` | 拉流客户端 | 见下方示例 |

```bash
# GStreamer RTSP 服务端（推荐 test-launch 快速验证）
./test-launch "( v4l2src device=/dev/video0 ! \
  video/x-raw,width=1920,height=1080,framerate=30/1 ! \
  nvv4l2h264enc bitrate=4000000 ! \
  rtph264pay name=pay0 pt=96 )"

# 客户端拉流
gst-launch-1.0 rtspsrc location=rtsp://192.168.1.10:8554/test latency=100 ! \
  rtph264depay ! h264parse ! avdec_h264 ! autovideosink
```

### 关注点

| 问题 | 说明 |
|------|------|
| UDP 丢包 | 使用 `rtspsrc latency=200`；严重时切 TCP（`protocols=tcp`） |
| Jitter Buffer | 值太小丢帧，太大增延迟；典型 100–200 ms |
| NAT 穿透差 | 内网可用，跨网需 TURN 或改 WebRTC |
| 客户端兼容性 | 部分浏览器不支持 RTSP，需转 WebRTC/HLS |

---

## A4.2 WebRTC

### 工作流向
```
Encoder（本地帧）
    │ 输入帧 / 编码帧
    ▼
WebRTC Stack
  ├─ SRTP（媒体加密）
  ├─ SCTP / DTLS（数据通道）
  ├─ GCC / REMB / TWCC（拥塞控制）
  └─ NACK / FEC（丢包恢复）
    │
    ▼ ICE（STUN/TURN 协商）
    │
    ▼
Browser / App / Peer（解码渲染）
```

### 信令流程
```
Peer A                     Signaling Server          Peer B
  │── createOffer ─────────────► │                     │
  │                              │──── offer ─────────► │
  │                              │◄─── answer ──────── │
  │◄─ answer ──────────────────── │                     │
  │─────────────────────── ICE Candidates ────────────► │
  │◄──────────────────────────────────────── ICE ────── │
  │═══════════════════ SRTP/DTLS Media ════════════════► │
```

### 关键库 / 框架

| 库 | 语言 | 特点 |
|----|------|------|
| [Pion WebRTC](https://github.com/pion/webrtc) | Go | 轻量，适合服务端 |
| [aiortc](https://github.com/aiortc/aiortc) | Python | 快速原型 |
| [GStreamer webrtcbin](https://gstreamer.freedesktop.org/documentation/webrtc/index.html) | C/GStreamer | 与 Pipeline 集成 |
| [libdatachannel](https://github.com/paullouisageneau/libdatachannel) | C++ | 嵌入式友好 |

```bash
# GStreamer WebRTC 推流示例（需要 signaling server）
gst-launch-1.0 v4l2src ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! vp8enc ! rtpvp8pay ! \
  webrtcbin name=sendonly bundle-policy=max-bundle \
    stun-server=stun://stun.l.google.com:19302
```

### 关注点

| 问题 | 说明 |
|------|------|
| 信令延迟 | WebSocket 信令尽量部署同区域 |
| STUN/TURN | 内网直连走 STUN，跨 NAT 必须 TURN；推荐 Coturn |
| 码率自适应 | GCC 算法在弱网下会主动降码率（可接受抖动） |
| 最低延迟 | 理想局域网 < 50 ms；广域网典型 100–300 ms |
| 浏览器兼容 | Chrome/Firefox/Safari 均支持 VP8/H.264 |

---

## A4.3 HTTP-FLV / HLS / DASH

### 工作流向
```
Encoder
    │ H.264/H.265 码流
    ▼
Muxer（FLV / TS segment / MP4 fragment）
    │ RTMP 推流 或 直接写文件
    ▼
Media Server（SRS / Nginx-RTMP / MediaMTX）
    │                 │
    ▼                 ▼
HTTP-FLV          HLS（.m3u8 + .ts 切片）
（< 3s 延迟）      （传统 3–10s；Low-Latency HLS < 1s）
    │                 │
    ▼                 ▼
Web Player          各类播放器（iOS / Android / PC）
```

### SRS 配置示例
```nginx
# srs.conf 片段
vhost __defaultVhost__ {
    hls {
        enabled     on;
        hls_fragment 1;   # 切片时长（秒）—— LL-HLS
        hls_window   5;   # 窗口（保留切片数）
    }
    http_remux {
        enabled  on;
        mount    [vhost]/[app]/[stream].flv;
    }
}
```

### 关注点

| 协议 | 典型延迟 | 主要缺点 |
|------|----------|----------|
| HTTP-FLV | 1–3 s | 依赖 Flash 时代协议（flv.js 可在浏览器播放） |
| HLS（传统） | 3–10 s | 切片时延高 |
| LL-HLS | < 1–2 s | 需服务端和客户端同时支持 |
| DASH | 2–8 s | 标准化好，CDN 支持广 |

---

## A4.4 HTTP 上传 / REST 数据回传

### 工作流向
```
设备端编码（关键帧 / 事件片段）
    │ HTTP POST（multipart/form-data 或 chunked）
    ▼
服务端（鉴权 → 存储 → 触发转码/审核）
    │
    ▼
下游消费（数据库 / 对象存储 / 流水线分析）
```

### 关键设计点

| 问题 | 推荐方案 |
|------|---------|
| 断点续传 | HTTP Range 请求 / TUS 协议 |
| 文件切片 | 每 10–30 s 一个 MP4 片段，避免单文件过大 |
| 鉴权 | JWT / API Key，HTTPS 强制 |
| 限流 | 设备端指数退避重试 |
| 存储成本 | 按事件上传（非全量），本地存储 + 周期同步 |

```python
# 简单 HTTP 上传示例（Python requests）
import requests, pathlib

def upload_clip(filepath: str, server_url: str, token: str):
    with open(filepath, "rb") as f:
        resp = requests.post(
            f"{server_url}/api/v1/clips",
            headers={"Authorization": f"Bearer {token}"},
            files={"file": (pathlib.Path(filepath).name, f, "video/mp4")},
            timeout=30,
        )
    resp.raise_for_status()
    return resp.json()
```

---

## 5. 协议选型矩阵

| 场景 | 推荐协议 | 次选 |
|------|---------|------|
| 局域网机器人实时预览 | RTP/RTSP | WebRTC |
| 远程遥操作（< 200ms） | WebRTC | — |
| 大规模直播分发 | HLS / HTTP-FLV | DASH |
| 离线事件取证/录制 | HTTP Upload | — |
| 多路录制回放 | HLS / MP4 | HTTP Upload |

---

## 6. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| RTSP UDP 端口 | 554 / 8554 | 8554 无需 root |
| RTP jitter buffer | 100–200 ms | 局域网取小值 |
| HLS 切片时长 | 1–2 s（LL-HLS） | 传统用 4–6 s |
| WebRTC STUN | `stun.l.google.com:19302` | 默认公共 STUN |
| HTTP 超时 | 30 s | 上传大文件适当增加 |

---

## 7. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| RTSP 端到端延迟（LAN） | < 300 ms | VLC 时间戳 |
| WebRTC 端到端延迟（LAN） | < 100 ms | `getStats()` |
| HLS 延迟（LL-HLS） | < 2 s | 播放器时间戳 |
| HTTP 上传成功率 | > 99.9% | 服务端日志 |

---

## 8. 常见问题与排查步骤（Checklist）

- [ ] RTSP 连不上 → 确认端口开放，`telnet host 8554`
- [ ] RTP 花屏 → 查 jitter buffer 是否太小；检查网络丢包 `ping -f`
- [ ] WebRTC 无法建立连接 → 检查 STUN/TURN；确认信令 WebSocket 正常
- [ ] HLS 起播慢 → 增加 Preload hint（LL-HLS）；检查 CDN 源站延迟
- [ ] HTTP 上传超时 → 减小切片大小；添加断点续传逻辑
- [ ] FLV 播放卡顿 → 检查服务端推流码率是否稳定；flv.js Worker 模式

---

## 9. 参考资料

- [RFC 7826 – RTSP 2.0](https://tools.ietf.org/html/rfc7826)
- [RFC 3550 – RTP](https://tools.ietf.org/html/rfc3550)
- [WebRTC 规范（W3C）](https://www.w3.org/TR/webrtc/)
- [SRS（Simple Real-time Server）](https://github.com/ossrs/srs)
- [MediaMTX（rtsp-simple-server）](https://github.com/bluenviron/mediamtx)
- [Apple HTTP Live Streaming（HLS）](https://developer.apple.com/documentation/http_live_streaming)
- [TUS 断点续传协议](https://tus.io/)
