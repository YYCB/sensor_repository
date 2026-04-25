# D1 – rm_hal_common 公共基础类型

`rm_hal_common` 包含所有传感器类型共享的基础定义，不依赖任何 SDK 或中间件。

---

## 新增：SensorTimestamp（修正 #1）

### 背景

`Sensor HAL.md V0.3.2` 中，`TimestampDomain`（Hardware / System / Global）仅定义在
Camera 侧的 `ImageFrame` 里。`LaserScanData`、`ImuData`、`AudioFrame` 均使用裸 `uint64_t`，
既无法区分时钟来源，也无法在融合时判断是否可以直接做时间差运算。

### 修正

提取 `SensorTimestamp{uint64_t ns, TimestampDomain domain}` 到 `rm_hal_common/sensor_timestamp.hpp`，
所有传感器帧类型统一使用该结构体替换原有的 `uint64_t timestamp_ns`。

| 帧类型 | 典型 domain 值 | 说明 |
|--------|---------------|------|
| `ImageFrame` | Hardware（Orbbec / RealSense 硬件时间戳） | SDK 提供设备级时钟 |
| `LaserScanData` | System（UDP 接收时刻） 或 Hardware（HDR2 帧头有时间戳） | BlueSea HDR2/HDR3 携带 16 位时间戳，可升级为 Hardware |
| `PointCloudXYZI` | System 或 Hardware | 同上（Velodyne/Livox 协议提供设备时间戳） |
| `ImuData` | System（串口接收时刻） 或 Hardware（DataID 0x80） | YESENSE 提供 sample_timestamp_ms |
| `AudioFrame` | System（snd_pcm_readi 返回时刻） | ALSA 无硬件时间戳 |

---

## 新增：ISensorHAL::reset()（修正 #2）

### 背景

所有传感器的状态机图（Camera §2.3、LiDAR §3.2、IMU §4.2、Audio §5.2）都有

```
Faulted ──reset()──► Closed
```

但任何接口类中都找不到 `reset()` 的声明，实现方容易遗漏。

### 修正

```cpp
class ISensorHAL : public IHardwareDevice {
public:
    // Recover from Faulted → Closed. Default: calls close().
    virtual bool reset() { close(); return true; }
};
```

具体实现可覆盖此方法做更深的清理（SDK 内部状态、内存池、环形缓冲区清空等）。

---

## HealthStatus

原文档多处引用 `health()` 返回类型，但未明确定义结构体字段。完整定义：

```cpp
struct HealthStatus {
    bool        alive        = false;  // 有数据输出 && 无持续故障
    double      data_rate_hz = 0.0;    // 实测输出频率 (Hz)
    std::string error_msg;             // 最后一次错误描述; 空 = 无错误
    uint32_t    drop_count   = 0;      // 累计丢帧/包（自 open() 起）
    uint32_t    error_count  = 0;      // 累计协议/CRC 错误
};
```

---

## ErrorCode

将 Sensor HAL.md §2.4 中分散描述的错误场景统一到 `ErrorCode` 枚举，
分为 5 组：General / DeviceLifecycle / Configuration / DataIO / SDK / Permissions。

当前阶段：驱动内部使用 `ErrorCode` 记录日志和设置 `health().error_msg`；
公开方法仍返回 `bool`（兼容现有调用方）。
未来可迁移为 `ErrorCode open()` 以便调用方区分不同失败原因。

---

## HALFactory\<T\>

类型化工厂，通过字符串 key 注册和创建驱动实例。

```cpp
// 驱动注册（通常在 .cpp 静态初始化阶段）：
REGISTER_HAL(CameraFactory, "orbbec",   OrbbecCameraHAL)
REGISTER_HAL(CameraFactory, "sim",      SimCameraHAL)
REGISTER_HAL(Lidar2DFactory,"bluesea",  BlueseaLidar2DHAL)
REGISTER_HAL(Lidar3DFactory,"velodyne", VelodyneLidar3DHAL)
REGISTER_HAL(ImuFactory,    "yesense",  YesenseImuHAL)
REGISTER_HAL(AudioFactory,  "respeaker",RespeakerAudioHAL)

// 使用：
auto cam  = CameraFactory::instance().create("orbbec");
auto lidar = Lidar3DFactory::instance().create("velodyne");
```

Factory 类型别名在各 HAL 头文件中定义（避免 hal_factory.hpp 循环依赖）：

```cpp
// camera_hal.hpp
using CameraFactory  = rm::hal::HALFactory<ICameraHAL>;
// lidar_2d_hal.hpp
using Lidar2DFactory = rm::hal::HALFactory<I2DLidarHAL>;
// lidar_3d_hal.hpp
using Lidar3DFactory = rm::hal::HALFactory<I3DLidarHAL>;
// imu_hal.hpp
using ImuFactory     = rm::hal::HALFactory<IImuHAL>;
// audio_hal.hpp
using AudioFactory   = rm::hal::HALFactory<IAudioHAL>;
```
