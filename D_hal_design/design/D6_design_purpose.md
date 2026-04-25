# D6 – Sensor HAL 设计目的、作用、收益与能力说明

> **适读对象**：对本仓库 `D_hal_design/` 不熟悉的新成员；
> 在评估是否引入本接口层的团队负责人；
> 需要向上汇报时的参考材料。

---

## 一、设计目的

### 核心问题

RMOS 机器人系统同时接入 **五类异构传感器**：

| 传感器 | 典型硬件 | 接入接口 | SDK / 协议 |
|--------|---------|---------|-----------|
| 相机（Camera） | Orbbec Gemini 330 / RealSense D435i / USB UVC | USB / GMSL2 | OrbbecSDK / librealsense2 / V4L2 |
| 2D 激光雷达 | BlueSea LDS-U50C-S | Ethernet UDP | BlueSea 私有帧协议 |
| 3D 激光雷达 | Velodyne VLP-16/32C、Livox Mid-360 | Ethernet UDP | Velodyne PCAP / Livox SDK |
| IMU | YESENSE 系列 | UART 串口 | YESENSE TLV 二进制协议 |
| 麦克风阵列 | ReSpeaker 4-Mic / 6-Mic | USB | ALSA / USB HID |

如果各业务模块（感知、导航、语音）直接依赖厂商 SDK，会产生以下问题：

1. **厂商绑定**：更换传感器型号 = 全局改代码，迁移成本高
2. **时间戳不可信**：每个 SDK 用不同时钟域，跨传感器融合需额外适配
3. **生命周期不一致**：open/close/reset 在各 SDK 里风格各异，上层必须写胶水代码
4. **悬空指针风险**：异步回调直接传出 SDK 内部缓冲区的裸指针（如 ALSA mmap），回调延迟时 UB
5. **难以单元测试**：业务逻辑和硬件 I/O 耦合在一起，无法用 Sim 驱动替换

**本设计的目的就是用一个薄而稳定的接口层（HAL）解决上述所有问题。**

---

## 二、在 RMOS 中的作用（层次定位）

```
┌─ 应用层 / Motion Planning ─────────────────────────────────────────────────────┐
│                                                                                 │
│  Module Library (rm_sensor_module)                                              │
│  ┌────────────────────────────────────────────────────────────────────────────┐ │
│  │  ISyncCoordinator   FilterPipeline   SensorTFPublisher   SpeechManager     │ │
│  └────────────────────────────────────────────────────────────────────────────┘ │
│                                    ▲ 只依赖 HAL 接口                            │
├─ ArcRT Channel（message_filter 跨品类时间戳对齐）────────────────────────────────┤
├─ rm_alg_foundation（depthToPointCloud / DecimationFilter）──────────────────────┤
│                                                                                 │
│  ┌── Sensor HAL 接口层（本文件夹，D_hal_design/） ────────────────────────────┐  │
│  │  ICameraHAL  I2DLidarHAL  I3DLidarHAL  IImuHAL  IAudioHAL               │  │
│  │  统一：open/close/reset/health/deviceId + SensorTimestamp                │  │
│  └─────────────────────────────────────────────────────────────────────────┘  │
│                                    ▲ 实现层（驱动，不在本仓库）                  │
├─ BSP / Vendor SDK ──────────────────────────────────────────────────────────────┤
│  OrbbecSDK   librealsense2   V4L2   BlueSea UDP   YESENSE TLV   ALSA           │
└────────────────────────────────────────────────────────────────────────────────┘
```

**HAL 层的精确定位**：

- **向下**：为各厂商 SDK / BSP 定义统一的实现合约（纯虚接口），驱动开发者只需继承并实现
- **向上**：为业务模块提供稳定 API，模块代码 `#include` 的是 HAL 头文件，**永远不会看到 OrbbecSDK、librealsense2 或 ALSA 的类型**
- **横向**：提供跨传感器一致的时间戳（`SensorTimestamp`），使 ArcRT `message_filter` 可以对齐任意两路传感器的数据

---

## 三、带来的收益

### 3.1 对业务开发者

| 场景 | 无 HAL | 有 HAL |
|------|--------|--------|
| 更换相机型号（Orbbec → RealSense） | 改所有调用 `OBFrame::getTimestamp()` 的地方 | 只换驱动注册 key，业务代码不动 |
| 单元测试感知算法 | 必须接真实相机 | `CameraFactory::instance().create("sim")` 注入仿真帧 |
| 多传感器时间戳对齐 | 各类型时钟来源不同，自行判断 | 统一读 `frame.timestamp.ns` 与 `frame.timestamp.domain` |
| 传感器故障恢复 | 各 SDK 恢复流程不同 | 统一调 `hal->reset()`，内部细节透明 |
| 热插拔 | 轮询 SDK-specific 事件 | `HALFactory::setDeviceChangedCallback()` 统一注册 |

### 3.2 对驱动开发者

- **明确合约**：接口头文件即设计文档，不需要口口相传哪些方法必须实现
- **默认实现**：`reset()` 已提供 `close()` 降级默认，驱动可选择覆盖也可不管
- **工厂注册一行搞定**：`REGISTER_CAMERA_HAL("orbbec", OrbbecCameraHAL)` 即可接入工厂体系

### 3.3 对系统整体

| 收益维度 | 具体描述 |
|---------|---------|
| **安全性** | `AudioFrame::data` 从裸指针改为 `shared_ptr`，消除了异步回调的悬空引用 UB |
| **可测试性** | 接口与 SDK 解耦，CI 可在无硬件环境运行全量感知单测 |
| **可维护性** | 单一变更点：传感器型号升级只影响对应驱动，不扩散到上层 |
| **时间精度** | 统一 `TimestampDomain`（Hardware/System/Global），融合算法可按需选择精度 |
| **扩展性** | 新增传感器类型（如 GNSS）只需新增 `IGnssHAL : ISensorHAL`，不影响已有接口 |

---

## 四、提供的能力

### 4.1 统一生命周期管理

所有传感器共享相同的状态机骨架，调用方无需记忆每个 SDK 的差异：

```
Closed ──configure()──► Configured ──open()──► [Opened/Streaming]
  ▲                                                     │
  └──────────────────── close() ───────────────────────┘
任意状态 ──fault──► Faulted ──reset()──► Closed
```

| 方法 | 来自 | 所有传感器通用 |
|------|------|:---:|
| `configure(Config)` | 各 HAL 接口 | ✓ |
| `open()` | `IHardwareDevice` | ✓ |
| `close()` | `IHardwareDevice` | ✓ |
| `reset()` | `ISensorHAL` ★ | ✓ |
| `isOpen()` | `IHardwareDevice` | ✓ |
| `deviceId()` | `IHardwareDevice` | ✓ |
| `health()` | `IHardwareDevice` | ✓ |

### 4.2 统一健康监控

`HealthStatus` 结构体提供任意时刻非阻塞的健康快照：

```cpp
HealthStatus h = hal->health();
// h.alive         — 设备是否正常出数据
// h.data_rate_hz  — 实测帧率
// h.error_msg     — 最新错误描述
// h.drop_count    — 累计丢帧数
// h.error_count   — 累计协议错误数
```

诊断系统 / 运维看板可统一读取所有传感器的 `health()`，无需了解各 SDK 细节。

### 4.3 统一时间域（跨传感器数据融合的基础）

```cpp
struct SensorTimestamp {
    uint64_t        ns;      // 纳秒
    TimestampDomain domain;  // Hardware / System / Global
};
```

| 能力 | 描述 |
|------|------|
| 时钟域区分 | 上层 `message_filter` 可判断两帧是否来自同一时钟，决定是否直接做差 |
| 精度升级路径 | 驱动优先填 `Hardware`（设备自带时戳），回退时填 `System`，行为自描述 |
| 统一 API | 无论 Camera / LiDAR / IMU / Audio，读时间都是 `frame.timestamp.ns` |

### 4.4 类型化传感器数据

每类传感器输出语义精确的独立数据类型，不强制统一：

| 传感器 | 输出类型 | 关键字段 |
|--------|---------|---------|
| Camera | `ImageFrame` | 编码格式、分辨率、曝光、帧号、`SensorTimestamp` |
| Camera（对齐帧组） | `FrameSet` | color + depth + ir 三路对齐 |
| Camera（点云） | `PointCloud`（SDK 派生） | xyz + rgb，适用于深度+彩色融合 |
| 2D LiDAR | `LaserScanData` | 极坐标 ranges/intensities，等效 ROS LaserScan |
| 3D LiDAR | `PointCloudXYZI` | xyz + intensity + **time_offset**（运动补偿必需）+ ring |
| IMU | `ImuData` | 加速度、角速度、四元数、欧拉角、磁场、温度 |
| Audio | `AudioFrame` | shared_ptr PCM 数据（安全所有权） |
| Audio DOA | `DOAResult` | 方位角、仰角、置信度 |

### 4.5 双模数据获取（轮询 + 回调）

每个 HAL 接口同时支持两种获取模式，调用方按需选择：

```cpp
// 方式 A：轮询（适合主循环同步消费）
LaserScanData scan;
lidar2d->getScan(scan);

// 方式 B：回调（适合独立线程异步消费）
lidar2d->setScanCallback([](shared_ptr<const LaserScanData> s) {
    // HAL 内部线程调用；s 的生命周期由 shared_ptr 管理
});
```

### 4.6 硬件配置与标定（Camera）

`ICameraHAL` 提供完整的运行时硬件控制与标定能力：

- **流配置**：`getSupportedProfiles()` 枚举设备支持的分辨率/帧率/格式组合
- **运行时选项**：`getOption / setOption`（曝光、增益、白平衡、深度后处理等）
- **标定读写**：`getIntrinsics / getExtrinsics / getIMUCalibration / loadUserCalibration`
- **硬件同步**：`getSyncManager()` → `ISyncManager`（Primary / Secondary / SoftwareTrigger）

### 4.7 插件式驱动工厂（热插拔支持）

```cpp
// 注册（驱动 .cpp 静态初始化，零侵入）
REGISTER_CAMERA_HAL("orbbec",    OrbbecCameraHAL)
REGISTER_CAMERA_HAL("sim",       SimCameraHAL)
REGISTER_LIDAR3D_HAL("velodyne", VelodyneLidar3DHAL)

// 使用（配置驱动，业务代码无需 if/else）
auto hal = CameraFactory::instance().create(config["type"]);

// 设备枚举
auto devices = CameraFactory::instance().enumerateDevices();

// 热插拔回调
CameraFactory::instance().setDeviceChangedCallback(
    [](auto added, auto removed) { /* 处理插拔 */ });
```

### 4.8 麦克风阵列 DOA 能力

`IAudioHAL` 不仅输出 PCM 音频流，还集成 ReSpeaker USB HID 的方向估计：

```cpp
audio->setDOACallback([](const DOAResult& r) {
    // r.azimuth_deg  — 声源方位角（已校正安装偏移）
    // r.confidence   — 估计置信度 [0,1]
});
auto latest = audio->getLatestDOA();  // 同步查询
```

---

## 五、一句话总结

> **Sensor HAL（D_hal_design/）是 RMOS 传感器子系统的「标准插槽」——**
> 它用 20 个 C++17 纯头文件定义了相机、激光雷达、IMU、麦克风四类传感器的统一接入合约，
> 让业务代码与厂商 SDK 彻底解耦，让多传感器数据融合拥有可信的统一时间基准，
> 让传感器从「接一个改一次代码」变成「注册一行字符串、业务代码零改动」。

---

## 六、快速索引

| 想了解… | 看哪里 |
|---------|--------|
| 架构图 & 层次关系 | [D0_overview.md](D0_overview.md) |
| 时间戳统一方案 | [D1_common.md](D1_common.md) §SensorTimestamp |
| 相机接口完整方法 | [D2_camera.md](D2_camera.md) |
| 2D vs 3D LiDAR 拆分理由 | [D3_lidar.md](D3_lidar.md) |
| IMU TLV 字段映射 | [D4_imu.md](D4_imu.md) |
| AudioFrame 安全所有权说明 | [D5_audio.md](D5_audio.md) |
| C++ 头文件 | [../include/](../include/) |
| 传感器型号 & 接入规格 | [../../B_sensor_catalog/](../../B_sensor_catalog/) |
| 相机端到端流程 | [../../A_camera_pipeline/](../../A_camera_pipeline/) |
