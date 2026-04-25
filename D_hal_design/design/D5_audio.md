# D5 – Audio HAL 设计

Audio HAL 与 `Sensor HAL.md V0.3.2 §5` 保持一致，有一项关键修正。

---

## 修正：AudioFrame 所有权模型（修正 #3）

### 原设计的问题

```cpp
// Sensor HAL.md V0.3.2 原版 ── 有安全漏洞
struct AudioFrame {
    const int16_t* data;          // 裸指针，指向 ALSA mmap 内部缓冲区
    size_t         sample_count;
    // ...
};
using AudioCallback = std::function<void(const AudioFrame&)>;
```

`captureLoop()` 调用 `snd_pcm_readi()` 后，PCM 数据存在于内核分配的 ALSA buffer 中。
`snd_pcm_readi()` 在下一次调用时可能立刻回收该 buffer。
如果 `AudioCallback` 异步持有 `AudioFrame`（例如投递到队列，或在另一线程消费），
则 `data` 指针将成为悬空引用，引发 UB。

### 修正方案

```cpp
// D_hal_design 修正版
struct AudioFrame {
    std::shared_ptr<const std::vector<int16_t>> data;  // 共享所有权
    size_t   frame_count  = 0;
    uint8_t  channels     = 0;
    uint32_t sample_rate  = 0;
    // ...
};
```

HAL 内部 `captureLoop()` 在调用 ALSA 后**立即将数据复制**到一个堆分配的
`vector<int16_t>`，包装为 `shared_ptr` 后填入 `AudioFrame`。
每帧开销：一次 `new` + 一次 memcpy（1024 frames × 4 channels × 2 B = **8 KB**，可忽略）。

**与 Camera 侧对齐**：`ImageFrame` 已用 `shared_ptr<const ImageFrame>` 传递，
`AudioFrame` 修正后遵循相同的所有权模型。

---

## IAudioHAL 方法表

| 方法 | 说明 |
|------|------|
| `configure(AudioConfig)` | 配置 ALSA 设备、采样率、通道数、DOA 开关 |
| `open() / close() / reset()` | 继承自 ISensorHAL |
| `startCapture()` | 启动 ALSA 采集线程 |
| `stopCapture()` | 停止采集线程 |
| `setAudioCallback(cb)` | 每个 ALSA period 触发（~64 ms at 16kHz/1024） |
| `setDOACallback(cb)` | 每次 DOA 估计更新触发（ReSpeaker HID 轮询） |
| `getLatestDOA()` | 同步获取最近 DOA（`std::optional<DOAResult>`） |

---

## ALSA PCM 映射

| `AudioConfig` 字段 | ALSA API |
|-------------------|----------|
| `device_name` | `snd_pcm_open(&handle, device_name, SND_PCM_STREAM_CAPTURE, 0)` |
| `sample_rate` | `snd_pcm_hw_params_set_rate_near()` |
| `channels` | `snd_pcm_hw_params_set_channels()` |
| `format` | `snd_pcm_hw_params_set_format()` |
| `period_frames` | `snd_pcm_hw_params_set_period_size_near()` |
| `buffer_frames` | `snd_pcm_hw_params_set_buffer_size_near()` |

---

## ReSpeaker DOA

ReSpeaker 4-Mic / 6-Mic 通过 USB HID 暴露 DOA 估计：

| HID 寄存器 | 名称 | 说明 |
|-----------|------|------|
| 21 | DOAANGLE | 方位角 0~359°（正北 = 0，顺时针正） |
| 19 | SPEECHDETECTED | 语音检测 0/1 |
| 20 | VOICEACTIVITY | VAD 置信度 |

`DOAResult.azimuth_deg` 在 HAL 内完成安装偏移校正后输出。

---

## 状态机

```
Closed ──configure()──► Configured ──open()──► Ready ──startCapture()──► Capturing
  ▲                                                                           │
  └──────────────────────── close() ─────────────────────────────────────────┘
任意状态 ──fault──► Faulted ──reset()──► Closed
```

Capturing 内部子状态：

| 子状态 | 条件 | HAL 行为 |
|--------|------|---------|
| NORMAL | 正常 | 持续调用 AudioCallback |
| OVERRUN | `snd_pcm_readi` 返回 -EPIPE | `snd_pcm_prepare()` 恢复，drop_count++ |
| ERROR | 无法恢复（-ENODEV 等） | 状态 → Faulted，停止采集线程 |
