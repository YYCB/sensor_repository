# A6. 解码、渲染与下游算法

## 1. 这篇文章要解决什么问题？

将编码码流（H.264/H.265）解码为图像帧，并高效分发给显示（OpenGL/Vulkan/Web 播放器）和算法（检测/SLAM）两类消费者，同时保证零拷贝和低延迟。

---

## 2. 数据链路图

```
[网络/存储 码流]
        │ RTP / RTSP / HLS / 文件
        ▼
[解复用（Demux）]
  TS / FLV / MP4 → NAL Units
        │
        ▼
[解码器（Decoder）]
  ┌────────────────────────────────┐
  │ 硬解：NVDEC / V4L2 M2M / VAAPI │
  │ 软解：FFmpeg avcodec / OpenH264 │
  └────────────────────────────────┘
        │ 解码帧：NV12 / YUV420 / RGBA
        │ (DMABUF / CUDA 内存 / 共享内存)
        │
        ├──────────────────────────────────────┐
        ▼                                      ▼
[显示（Render）]                      [算法（Vision）]
  OpenGL/EGL（纹理导入）               检测/跟踪/SLAM
  GStreamer videosink                  尽量零拷贝
  Web 播放器（Video Element）          定义清晰帧格式+元数据
```

---

## 3. 解码器选择

### 3.1 平台硬件解码

| 平台 | API | 格式 |
|------|-----|------|
| NVIDIA（x86 / Jetson） | NVDEC / CUVID | H.264 / H.265 / AV1（40系+） |
| Intel | VAAPI / QSV | H.264 / H.265 / AV1 |
| Jetson（ARM） | V4L2 M2M / NVJPEG | H.264 / H.265 |
| Android | MediaCodec | H.264 / H.265 / VP9 |
| RK（Rockchip） | MPP（媒体处理平台） | H.264 / H.265 |

### 3.2 FFmpeg 硬解命令
```bash
# NVDEC（CUVID）
ffplay -vcodec h264_cuvid -rtsp_transport tcp rtsp://192.168.1.10:8554/test

# VAAPI（Intel）
ffplay -hwaccel vaapi -hwaccel_output_format vaapi \
  -i rtsp://192.168.1.10:8554/test

# V4L2 M2M（Jetson / RPi）
ffplay -vcodec h264_v4l2m2m -i rtsp://192.168.1.10:8554/test
```

### 3.3 GStreamer 硬解管道
```bash
# Jetson nvv4l2decoder
gst-launch-1.0 rtspsrc location=rtsp://host:8554/test latency=100 ! \
  rtph264depay ! h264parse ! nvv4l2decoder ! \
  nvvidconv ! video/x-raw,format=BGRx ! \
  videoconvert ! video/x-raw,format=BGR ! \
  appsink name=sink

# NVDEC（使用 nvh264dec）
gst-launch-1.0 rtspsrc location=rtsp://host:8554/test ! \
  rtph264depay ! h264parse ! nvh264dec ! \
  glimagesink
```

---

## 4. 缓冲模型与零拷贝

### 4.1 零拷贝路径（推荐）
```
解码器 DMABUF fd
        │
        ├──► EGL Image（OpenGL 纹理）── 显示（无拷贝）
        │
        └──► CUDA cuImportExternalMemory── GPU 算法（无拷贝）
```

### 4.2 DMABUF → OpenGL 纹理（EGL）
```cpp
// 获取 DMABUF fd（来自 V4L2 或 GStreamer）
int dmabuf_fd = buf.m.fd;

// 创建 EGLImage
EGLAttrib attrs[] = {
    EGL_WIDTH,             width,
    EGL_HEIGHT,            height,
    EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_NV12,
    EGL_DMA_BUF_PLANE0_FD_EXT, dmabuf_fd,
    EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
    EGL_DMA_BUF_PLANE0_PITCH_EXT, stride,
    // UV plane...
    EGL_NONE
};
EGLImage egl_image = eglCreateImage(display, EGL_NO_CONTEXT,
    EGL_LINUX_DMA_BUF_EXT, nullptr, attrs);

// 绑定纹理
glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id);
glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, egl_image);
```

### 4.3 帧队列模型

| 场景 | 队列策略 |
|------|---------|
| 实时算法订阅 | 容量 1，覆盖旧帧（最新帧优先） |
| 显示 | 容量 2–3，允许短暂缓冲平滑抖动 |
| 录制 | 无限或大容量，丢帧时告警 |

---

## 5. 渲染

### 5.1 GStreamer 显示 Sink

```bash
# 本地 OpenGL 窗口
gst-launch-1.0 ... ! glimagesink

# Wayland
gst-launch-1.0 ... ! waylandsink

# 无头（算法消费，不显示）
gst-launch-1.0 ... ! appsink emit-signals=true max-buffers=1 drop=true
```

### 5.2 Web 播放器（WebRTC / HLS）
```html
<!-- WebRTC -->
<video id="remoteVideo" autoplay playsinline></video>
<script>
  const pc = new RTCPeerConnection({ iceServers: [...] });
  pc.ontrack = (e) => { remoteVideo.srcObject = e.streams[0]; };
</script>

<!-- HLS (hls.js) -->
<video id="player" controls></video>
<script src="hls.min.js"></script>
<script>
  const hls = new Hls();
  hls.loadSource("http://server/live/stream.m3u8");
  hls.attachMedia(document.getElementById("player"));
</script>
```

---

## 6. 下游算法接入

### 6.1 帧格式约定

```cpp
struct Frame {
    uint64_t frame_id;
    uint64_t timestamp_ns;       // PTP 时间（纳秒）
    uint32_t width, height;
    PixelFormat format;          // NV12 / BGR / RGBA
    uint8_t* data[3];            // plane 指针
    int      linesize[3];        // plane stride
    int      dmabuf_fd;          // -1 表示普通内存
    // 可选元数据
    uint32_t exposure_us;
    float    gain_db;
};
```

### 6.2 ROS 2 订阅示例
```cpp
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 使用 cv_bridge 转换（注意 encoding: "bgr8" / "nv12"）
    auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat frame = cv_img->image;
    // 调用检测/SLAM 算法...
}
```

### 6.3 算法解耦建议

- 算法线程只读帧数据，不拥有帧生命周期（使用共享指针）；
- 算法超时（> 2 帧周期）时，跳过当前帧，不阻塞采集线程；
- 通过 `frame_id` 和 `timestamp_ns` 与其他传感器数据对齐。

---

## 7. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 解码器 | NVDEC / V4L2 M2M | 优先硬解 |
| 解码缓冲数 | 4–8 | 依平台 API |
| 帧队列（算法） | 容量 1，覆盖旧帧 | 保证实时性 |
| 渲染格式 | RGBA8（OpenGL）/ NV12（算法） | 按消费者选择 |
| 零拷贝 | DMABUF 优先 | 减少 PCIe/内存带宽 |

---

## 8. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| 解码延迟（硬解） | < 5 ms（1080p） | 时间戳差值 |
| 显示帧率 | 与采集帧率一致（无丢帧） | `gst-launch` `dropped` 计数 |
| 算法帧率 | ≥ 算法设计帧率 | 算法内部计时 |
| 内存拷贝次数 | 0（零拷贝路径） | `perf mem` |
| GPU 内存占用 | < 200 MB（4路1080p） | `nvidia-smi` |

---

## 9. 常见问题与排查步骤（Checklist）

- [ ] 解码花屏 → 检查 SPS/PPS 是否在关键帧前送入；确认格式（AnnexB vs AVCC）
- [ ] 解码延迟高 → 关闭 B 帧（编码侧）；使用低延迟 decode profile
- [ ] OpenGL 纹理颜色偏差 → GLSL Shader 中 NV12→RGB 转换系数是否正确
- [ ] DMABUF 导入失败 → 检查 EGL 扩展是否支持 `EGL_EXT_image_dma_buf_import`
- [ ] 算法 CPU 占用高 → 确认是否绕过 DMABUF，发生了不必要的内存拷贝
- [ ] 多路解码 GPU OOM → 减少并发解码流数；降低解码分辨率

---

## 10. 参考资料

- [FFmpeg 硬件加速文档](https://trac.ffmpeg.org/wiki/HWAccelIntro)
- [GStreamer NVDEC 插件](https://gstreamer.freedesktop.org/documentation/nvcodec/index.html)
- [EGL DMA-BUF 扩展规范](https://registry.khronos.org/EGL/extensions/EXT/EGL_EXT_image_dma_buf_import.txt)
- [NVIDIA Video Codec SDK（NVDEC）](https://developer.nvidia.com/nvidia-video-codec-sdk)
- [ROS 2 Image Transport](https://github.com/ros-perception/image_transport_plugins)
