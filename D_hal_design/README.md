# D – Sensor HAL 更新设计与接口实现

本文件夹为 Sensor HAL 抽象层的**更新版详细设计文档**及 **C++ 接口头文件**实现。

## 与 `Sensor HAL.md` 的关系

`Sensor HAL.md`（V0.3.2）是本设计的输入基线。本文件夹在此基础上修正了 4 项缺口，
其余接口（ICameraHAL / ISyncManager / OptionInfo / BlueSea 协议 / YESENSE TLV 等）
**完全保留原设计**，不重复描述。

| # | 问题 | 修正 |
|---|------|------|
| 1 | `TimestampDomain` 仅限 Camera，LiDAR / IMU 时钟域缺失 | 提取为 `SensorTimestamp{ns, domain}` 置于 `rm_hal_common`，所有帧类型统一使用 |
| 2 | `ISensorHAL` 无 `reset()` 正式声明（仅在状态机图中出现） | 在 `ISensorHAL` 中声明 `virtual bool reset()`，默认实现调用 `close()` |
| 3 | `AudioFrame::data` 为裸指针 `const int16_t*`，异步回调存在悬空风险 | 改为 `shared_ptr<const vector<int16_t>>`，与 Camera 侧所有权模型一致 |
| 4 | `ILidarHAL` 混用 2D 扫描与 3D 点云语义 | 拆分为 `I2DLidarHAL`（BlueSea 2D）和 `I3DLidarHAL`（Velodyne / Livox 3D） |

## 文件夹结构

```
D_hal_design/
├── README.md              ← 本文件
├── design/                ← 更新版详细设计文档
│   ├── D0_overview.md     ← 架构总览、层次划分、设计原则
│   ├── D1_common.md       ← rm_hal_common 公共类型设计
│   ├── D2_camera.md       ← Camera HAL 设计
│   ├── D3_lidar.md        ← LiDAR HAL 设计（2D/3D 拆分说明）
│   ├── D4_imu.md          ← IMU HAL 设计
│   └── D5_audio.md        ← Audio HAL 设计（AudioFrame 修复说明）
└── include/               ← C++ 接口头文件（纯头文件，无 .cpp）
    ├── rm_hal_common/     ← 跨传感器公共基础类型
    │   ├── sensor_timestamp.hpp   ← SensorTimestamp + TimestampDomain ★
    │   ├── health_status.hpp      ← HealthStatus
    │   ├── error_code.hpp         ← ErrorCode enum + ErrorInfo
    │   ├── hardware_device.hpp    ← IHardwareDevice 基类
    │   ├── sensor_hal_base.hpp    ← ISensorHAL（含 reset()）★
    │   └── hal_factory.hpp        ← HALFactory<T> + DeviceInfo + REGISTER_HAL
    ├── rm_hal_camera/     ← Camera HAL 接口
    │   ├── stream_type.hpp        ← StreamType enum + StreamIndex
    │   ├── pixel_encoding.hpp     ← PixelEncoding enum
    │   ├── calibration_types.hpp  ← DistortionModel / Intrinsics / Extrinsics / IMUCalibration
    │   ├── camera_types.hpp       ← ImageFrame / PointCloud / StreamProfile / OptionInfo / FrameSet / CameraConfig
    │   ├── sync_manager.hpp       ← SyncMode / SyncConfig / ISyncManager
    │   └── camera_hal.hpp         ← ICameraHAL（汇总入口）
    ├── rm_hal_lidar/      ← LiDAR HAL 接口
    │   ├── lidar_2d_types.hpp     ← Lidar2DConfig + LaserScanData
    │   ├── lidar_2d_hal.hpp       ← I2DLidarHAL（BlueSea 2D）
    │   ├── lidar_3d_types.hpp     ← Lidar3DConfig + PointXYZI + PointCloudXYZI ★
    │   └── lidar_3d_hal.hpp       ← I3DLidarHAL（Velodyne / Livox 3D）★
    ├── rm_hal_imu/        ← IMU HAL 接口
    │   ├── imu_types.hpp          ← AccelRange / GyroRange / ImuConfig / ImuData / ImuDeviceInfo
    │   └── imu_hal.hpp            ← IImuHAL
    └── rm_hal_audio/      ← Audio HAL 接口
        ├── audio_types.hpp        ← AudioSampleFormat / AudioFrame（已修复）/ DOAResult / AudioConfig
        └── audio_hal.hpp          ← IAudioHAL
```

★ = 相对 Sensor HAL.md V0.3.2 的新增 / 修改项

## 构建说明

所有文件为**纯头文件接口库**，不含实现代码（.cpp）。构建时只需将 `include/` 加入头文件搜索路径：

```cmake
# CMakeLists.txt 片段
add_library(rm_hal_sensor_interface INTERFACE)
target_include_directories(rm_hal_sensor_interface
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(rm_hal_sensor_interface INTERFACE cxx_std_17)
```

具体驱动（OrbbecCameraHAL、BlueseaLidar2DHAL 等）链接此 INTERFACE 库后即可实现对应接口。

## 设计原则

- **按同类传感器抽象**：Camera / LiDAR-2D / LiDAR-3D / IMU / Audio 各自独立接口，数据类型不强制统一
- **统一生命周期**：所有 HAL 继承 `IHardwareDevice → ISensorHAL`，共享 `open/close/reset/health/deviceId`
- **统一时间域**：所有帧数据携带 `SensorTimestamp{ns, domain}`，上层融合时可判断时钟来源
- **HAL 不依赖中间件**：禁止 `#include <rclcpp/...>` 或任何 ROS / ArcRT 头文件
