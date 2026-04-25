# D2 – Camera HAL 设计

Camera HAL 的设计与 `Sensor HAL.md V0.3.2 §2` 高度一致。本文件只描述变更点。

---

## 变更点

| 字段 | 原值 | 修改后 |
|------|------|--------|
| `ImageFrame::timestamp_ns` | `uint64_t` | `rm::hal::SensorTimestamp` |
| `PointCloud::timestamp_ns` | （无） | 新增 `rm::hal::SensorTimestamp timestamp` |

其余接口（`ICameraHAL` / `ISyncManager` / `OptionInfo` / `StreamProfile` /
`CameraConfig` / `PixelEncoding` / 标定结构体）**完全保留原设计**。

---

## 头文件依赖顺序

```
stream_type.hpp         无依赖
pixel_encoding.hpp      无依赖
calibration_types.hpp → stream_type.hpp
camera_types.hpp      → pixel_encoding.hpp + stream_type.hpp + sensor_timestamp.hpp
sync_manager.hpp        无依赖
camera_hal.hpp        → calibration_types.hpp + camera_types.hpp + sync_manager.hpp
                          + sensor_hal_base.hpp + hal_factory.hpp
```

---

## ICameraHAL 完整方法表

| 类别 | 方法 | 说明 |
|------|------|------|
| 配置 | `configure(CameraConfig)` | 必须先于 open() 调用 |
| 生命周期 | `open / close / isOpen / reset` | 继承自 ISensorHAL |
| 流控 | `startStreaming / stopStreaming` | open 后调用 |
| 轮询 | `getColorFrame / getDepthFrame / getIRFrame / getPointCloud` | 注册回调后失效 |
| 回调 | `setColorCallback / setDepthCallback / setIRCallback / setFrameSetCallback` | HAL 内部线程调用 |
| Profile | `getSupportedProfiles()` | 枚举设备支持的分辨率/帧率 |
| 标定 | `getIntrinsics / getExtrinsics / getIMUCalibration / getDepthMetadata` | 读取出厂标定 |
| 标定 | `loadUserCalibration / exportCalibration` | 用户标定覆盖 |
| 硬件选项 | `getSupportedOptions / getOptionInfo / getOption / setOption` | 曝光/增益/白平衡等 |
| 同步 | `getSyncManager()` | 返回 ISyncManager（不支持则返回 nullptr） |

---

## ISyncManager 层次说明

| 接口 | 层次 | 职责 |
|------|------|------|
| `ISyncManager` | HAL 层 | 单台设备的硬件同步模式配置（Primary / Secondary / SoftwareTrigger） |
| `ISyncCoordinator` | Module 层 | 编排多台相机互相等待的业务逻辑 |

两者严格分离；`ISyncManager` 通过 `ICameraHAL::getSyncManager()` 获取。

---

## 线程安全合约

- `health()` / `isOpen()` / `deviceId()`：任意线程安全调用
- 生命周期方法（`configure / open / close / startStreaming` 等）：调用方不得并发
- 回调在 HAL 内部专用线程执行；**回调内禁止回调同一 HAL 实例方法**（避免重入死锁）

---

## 状态机

```
Closed ──configure()──► Configured ──open()──► Opened ──startStreaming()──► Streaming
  ▲                                                                              │
  └──────────────────────── close() ────────────────────────────────────────────┘
任意状态 ──fault──► Faulted ──reset()──► Closed
```
