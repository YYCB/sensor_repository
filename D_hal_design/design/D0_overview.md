# D0 – Sensor HAL 架构总览

## 设计目标

Sensor HAL 为所有传感器设备提供统一的生命周期管理和类型安全的数据获取接口，同时保证：

- **零中间件依赖**：HAL 层头文件不得 `#include` 任何 ROS / ArcRT / Qt 头文件
- **按类型抽象**：不同物理特性的传感器使用独立接口与独立数据类型，不强制统一
- **统一生命周期**：所有 HAL 类型共享 `IHardwareDevice → ISensorHAL` 基类
- **统一时间域**：所有帧数据携带 `SensorTimestamp{ns, domain}`

---

## RMOS 层次关系

```
┌─ App / Motion Planning ────────────────────────────────────────┐
│ Module Library  (rm_sensor_module)                              │
│   ISyncCoordinator — 多相机跨设备帧对齐 (Module 层，非 HAL)       │
│   FilterPipeline — 后处理滤波管线 (Module 层)                    │
│   SensorTFPublisher — TF 坐标树发布 (Module 层)                  │
├─ ArcRT Channel ────────────────────────────────────────────────┤
│   message_filter — 跨品类时间戳对齐 (Channel 层)                  │
├─ rm_alg_foundation ────────────────────────────────────────────┤
│   depthToPointCloud / DecimationFilter / opticalToRos …        │
├─ Sensor HAL ← 本层 (D_hal_design/) ────────────────────────────┤
│   ICameraHAL / I2DLidarHAL / I3DLidarHAL / IImuHAL / IAudioHAL│
├─ BSP / SDK ────────────────────────────────────────────────────┤
│   OrbbecSDK / librealsense2 / V4L2 / BlueSea UDP / YESENSE TLV │
└────────────────────────────────────────────────────────────────┘
```

---

## 接口继承树

```
IHardwareDevice          (rm_hal_common/hardware_device.hpp)
  │  open() / close() / isOpen() / deviceId() / health()
  │
  └── ISensorHAL          (rm_hal_common/sensor_hal_base.hpp)
        │  + reset()  ← 本文件夹新增的正式声明
        │
        ├── ICameraHAL    (rm_hal_camera/camera_hal.hpp)
        ├── I2DLidarHAL   (rm_hal_lidar/lidar_2d_hal.hpp)
        ├── I3DLidarHAL   (rm_hal_lidar/lidar_3d_hal.hpp)   ★ 新增
        ├── IImuHAL       (rm_hal_imu/imu_hal.hpp)
        └── IAudioHAL     (rm_hal_audio/audio_hal.hpp)
```

---

## 四项修正（相对 Sensor HAL.md V0.3.2）

| # | 原问题 | 修正方案 | 影响文件 |
|---|--------|---------|---------|
| 1 | `TimestampDomain` 仅 Camera 有，LiDAR / IMU 用裸 `uint64_t` | 提取 `SensorTimestamp{ns, domain}` 到 `rm_hal_common`，所有帧类型统一使用 | `sensor_timestamp.hpp`；所有帧结构体 |
| 2 | `reset()` 仅在状态机图中出现，无接口声明 | 在 `ISensorHAL` 中声明 `virtual bool reset()`，默认调用 `close()` | `sensor_hal_base.hpp` |
| 3 | `AudioFrame::data` 为裸 `const int16_t*`，异步回调存在悬空风险 | 改为 `shared_ptr<const vector<int16_t>>` | `audio_types.hpp` |
| 4 | `ILidarHAL` 混用 2D 扫描与 3D 点云 | 拆分为 `I2DLidarHAL`（BlueSea）和 `I3DLidarHAL`（Velodyne/Livox） | `lidar_2d_*.hpp`、`lidar_3d_*.hpp` |

---

## 文件依赖图

```
rm_hal_common/
  sensor_timestamp.hpp     (无依赖)
  health_status.hpp        (无依赖)
  error_code.hpp           (无依赖)
  hardware_device.hpp   ─► health_status.hpp
  sensor_hal_base.hpp   ─► hardware_device.hpp
  hal_factory.hpp          (无 HAL 接口依赖; 纯模板)

rm_hal_camera/
  stream_type.hpp          (无依赖)
  pixel_encoding.hpp       (无依赖)
  calibration_types.hpp ─► stream_type.hpp
  camera_types.hpp      ─► pixel_encoding.hpp, stream_type.hpp, sensor_timestamp.hpp
  sync_manager.hpp         (无依赖)
  camera_hal.hpp        ─► calibration_types.hpp, camera_types.hpp, sync_manager.hpp
                           sensor_hal_base.hpp, hal_factory.hpp

rm_hal_lidar/
  lidar_2d_types.hpp    ─► sensor_timestamp.hpp
  lidar_2d_hal.hpp      ─► lidar_2d_types.hpp, sensor_hal_base.hpp, hal_factory.hpp
  lidar_3d_types.hpp    ─► sensor_timestamp.hpp
  lidar_3d_hal.hpp      ─► lidar_3d_types.hpp, sensor_hal_base.hpp, hal_factory.hpp

rm_hal_imu/
  imu_types.hpp         ─► sensor_timestamp.hpp
  imu_hal.hpp           ─► imu_types.hpp, sensor_hal_base.hpp, hal_factory.hpp

rm_hal_audio/
  audio_types.hpp       ─► sensor_timestamp.hpp
  audio_hal.hpp         ─► audio_types.hpp, sensor_hal_base.hpp, hal_factory.hpp
```

---

## CMake 使用方式

```cmake
# 接口库（无 .cpp，仅头文件）
add_library(rm_hal_sensor_interface INTERFACE)
target_include_directories(rm_hal_sensor_interface
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(rm_hal_sensor_interface INTERFACE cxx_std_17)

# 具体驱动示例
add_library(rm_hal_sensor_orbbec_usb STATIC
    orbbec_camera_hal.cpp)
target_link_libraries(rm_hal_sensor_orbbec_usb
    PUBLIC  rm_hal_sensor_interface
    PRIVATE OrbbecSDK::OrbbecSDK)
```
