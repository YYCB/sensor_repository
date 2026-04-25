# Sensor HAL 详细设计文档

> 版本: V0.3.2 | 日期: 2026-04-28 | 状态: 草稿
> 基线: V0.3.1 + 双厂商对比分析 (OrbbecSDK_ROS2 v2.7.6 vs realsense-ros 4.57.7)
> 版本序列: Notion V0.1 → V0.2 → V0.3 → V0.3.1 → **V0.3.2 (本版)**
>
> **V0.3.2 变更记录 (相对 V0.3.1):**
> - §2.1 新增: `StreamIndex{StreamType, int}` — 统一流标识符 (双厂共识)
> - §2.1 OptionRange → `OptionInfo` 升级: 新增 `OptionType` 枚举 + `enum_values` + `description`
> - §2.1 `ImageFrame` 新增 `TimestampDomain` 字段 + 帧元数据 (frame_number/actual_exposure/actual_gain)
> - §2.1 `ICameraHAL` 新增 `getSyncManager()` — 单设备硬件同步配置入口
> - §2.1A `CameraIntrinsics` 新增 `DistortionModel` 枚举 (BrownConrady/KannalaBrandt4 鱼眼)
> - §2.1A 新增 `IMUCalibration` 结构体 (scale_bias矩阵, 噪声参数)
> - **§2.11 新增: `ISyncManager` 接口** — 单设备 7 种硬件同步模式 + SyncConfig 延迟参数
> - §2.7 新增: `IDecoder/IDecoderFactory` — 平台解码器抽象 (libjpeg/Jetson NV/RK MPP)
> - §6.1 HALFactory 新增: `enableProcessLock()` — POSIX 共享内存互斥锁 (多进程安全)
> - §7.4 新增条目 15-19
> - **§7.5 新增**: ParameterProvider 自动映射 / TF 光学帧四元数约定 / IMU 软件对齐 / 图像后处理归属
>
> **V0.3.1 变更记录 (相对 V0.3):**
> - ICameraHAL 新增: getSupportedProfiles(), getExtrinsics(from,to), getIntrinsics(stream)
> - ICameraHAL 新增: 运行时硬件选项 (OptionRange/getOption/setOption)
> - ICameraHAL 新增: FrameSet 回调 (单设备内 Color+Depth 对齐帧组)
> - ICameraHAL 新增: StreamProfile / StreamType 枚举
> - HALFactory 新增: DeviceInfo 结构 + enumerateDevices() + setDeviceChangedCallback()
> - **§2.10 ISyncCoordinator 层级修正: HAL → Module 层** (基于 RMOS V5 架构复核)
> - **§7 新增 上层需求清单**: Module / ArcRT Channel / rm_alg_foundation 14 项需求
>
> **V0.3 变更记录 (相对 V0.2):**
> - 新增 §2.1A PixelEncoding 独立枚举 (24 种)
> - §2.6.1 OrbbecCameraHAL 映射升级为 API 代码级精度
> - §2.6.2 GMSL 方案完全重写: 基于 OrbbecSDK + /dev/camsync 硬件触发 (非独立 V4L2)
> - §2.6.2 新增 IGmslTrigger 平台抽象层 (Orin NX / S100 / Sim)
> - §2.6.4 新增 UsbCameraHAL (V4L2) 独立映射
> - §2.6.5 新增 SimCameraHAL 实现规范
> - §2.6.6 新增 Adapter 对比总表
> - §2.10 新增 多相机同步架构 (ISyncCoordinator + SyncPolicy)
> - CameraConfig 补充 depth_width/height, align_mode, ring_buffer_depth, color_encoding 等字段
> - 修正 ImuCallback 签名不一致 (shared_ptr → const ref)
> - §3.4 修正 BlueSea CRC16 → STM32 CRC32
> - §3 补充完整 LidarConfig 字段、BlueSea 6 种帧头格式、FanAssembler 算法
> - §4 补充 YESENSE TLV 完整字节级协议、12+ DataID 缩放因子、Fletcher CRC
> - §5 从骨架升级为完整设计 (IAudioHAL 接口/数据结构/ALSA 映射/线程模型/CMake)

---

## 目录

- §1 概述
- §2 Camera HAL 详细设计
  - §2.1 ICameraHAL 接口
  - §2.1A PixelEncoding 独立枚举 (**V0.3 新增**)
  - §2.2 数据结构 (**V0.3 补充**)
  - §2.3 状态机
  - §2.4 ErrorCode 定义
  - §2.5 线程与并发模型
  - §2.6 Adapter→SDK 实现映射
    - §2.6.1 OrbbecCameraHAL (USB) — 代码级 API 映射
    - §2.6.2 OrbbecGmslCameraHAL (GMSL2) — OrbbecSDK + /dev/camsync + **IGmslTrigger 平台抽象**
    - §2.6.3 RealsenseCameraHAL
    - §2.6.4 UsbCameraHAL (V4L2 通用) (**V0.3 新增**)
    - §2.6.5 SimCameraHAL (**V0.3 新增**)
    - §2.6.6 Adapter 对比总表 (**V0.3 新增**)
  - §2.7 CMake target 与文件布局
  - §2.8 异常与错误场景
  - §2.9 单测要点
  - **§2.10 多相机同步架构 (V0.3 新增)**
    - **§2.11 ISyncManager — 单设备硬件同步接口 (V0.3.2 新增)**
- §3 LiDAR HAL 详细设计
- §4 IMU HAL 详细设计
- §5 Audio HAL 详细设计
- §6 横切面: 工厂 / Sim / 性能 / Profile / 测试
- **§7 上层需求清单 — 基于 realsense-ros 对标分析 (V0.3.1 新增)**
  - §7.1 Module Library 层需求 (ISyncCoordinator / FilterPipeline / TF / 参数 / Lifecycle / Diagnostics)
  - §7.2 ArcRT Channel 层需求 (跨品类 message_filter 时间对齐)
  - §7.3 rm_alg_foundation 层需求 (深度滤波 / 点云生成 / 坐标变换)
  - §7.4 层次归属总表 (14 项需求 × 优先级)
    - **§7.5 新增 Module 层最佳实践 (V0.3.2 新增)**: ParameterProvider 自动映射 / TF 四元数约定 / IMU 软件对齐 / 图像后处理

---

## §1 概述

Sensor HAL 负责所有传感器设备的数据采集抽象。本文档为 Notion V0.3 版本，补全了 V0.2 中所有 TODO 章节，新增 GMSL 平台抽象层和多相机同步架构。

**接口继承体系:**
```
IHardwareDevice (§4.0.5)
  └── ISensorHAL (§4.1)
        ├── ICameraHAL
        ├── IImuHAL
        ├── ILidarHAL
        └── IAudioHAL
```

**子页与主文档关系:**
- Camera §2.1-§2.3: 以 Notion 子页 V0.2 为真源（本文摘要引用）
- Camera §2.4-§2.9: 本文为补充草稿，待同步回 Notion
- LiDAR §3 / IMU §4: 本文为首次详细设计，待同步回 Notion

---

## §2 Camera HAL 详细设计

### §2.1-§2.3 接口/数据结构/状态机 (摘要)

> 完整内容见 Notion Sensor HAL详设 V0.2 §2.1-§2.3。以下仅列关键要素供本地参考。

**ICameraHAL 核心方法:**
```cpp
class ICameraHAL : public ISensorHAL {
public:
    virtual bool configure(const CameraConfig& config) = 0;
    // open() / close() / isOpen() / health() 继承自 IHardwareDevice
    virtual bool startStreaming() = 0;
    virtual bool stopStreaming() = 0;
    virtual bool getColorFrame(ImageFrame& out) = 0;
    virtual bool getDepthFrame(ImageFrame& out) = 0;
    virtual bool getIRFrame(ImageFrame& out) = 0;
    virtual bool getPointCloud(PointCloud& out) = 0;
    virtual CameraIntrinsics getColorIntrinsics() const = 0;
    virtual CameraIntrinsics getDepthIntrinsics() const = 0;
    virtual CameraExtrinsics getExtrinsics() const = 0;  // [补充]
    using FrameCallback = std::function<void(std::shared_ptr<const ImageFrame>)>;
    virtual void setColorCallback(FrameCallback cb) = 0;
    virtual void setDepthCallback(FrameCallback cb) = 0;
    virtual void setIRCallback(FrameCallback cb) = 0;

    // ==== V0.3.1 新增: 基于 realsense-ros 对标分析 ====

    // --- Profile 查询 ---
    /// 查询设备支持的所有流配置 (分辨率/帧率/格式 组合)
    virtual std::vector<StreamProfile> getSupportedProfiles() const = 0;

    // --- 扩展标定参数查询 ---
    /// 获取任意两流之间的外参 (旋转+平移)
    virtual CameraExtrinsics getExtrinsics(StreamType from, StreamType to) const = 0;
    /// 获取指定流的内参
    virtual CameraIntrinsics getIntrinsics(StreamType stream) const = 0;

    // --- 运行时硬件选项 (曝光/增益/白平衡等) ---
    // V0.3.2: OptionRange 升级为 OptionInfo (见 §2.1A), 接口签名同步更新
    /// 获取所有支持选项的完整描述列表 (含类型/范围/默认值/枚举表)
    virtual std::vector<OptionInfo> getSupportedOptions() const = 0;
    /// 获取单个选项完整描述 (V0.3.2 替代旧 getOptionRange)
    virtual OptionInfo getOptionInfo(const std::string& name) const = 0;
    virtual float getOption(const std::string& name) const = 0;
    virtual bool  setOption(const std::string& name, float value) = 0;

    // --- FrameSet 回调 (单设备内的 Color+Depth 对齐帧组) ---
    struct FrameSet {
        std::shared_ptr<const ImageFrame> color;
        std::shared_ptr<const ImageFrame> depth;
        std::shared_ptr<const ImageFrame> ir;       // nullable
        uint64_t timestamp_ns;
    };
    using FrameSetCallback = std::function<void(const FrameSet&)>;
    virtual void setFrameSetCallback(FrameSetCallback cb) = 0;

    // ==== V0.3.2 新增: 基于双厂商对比分析 ====

    // --- 单设备硬件同步配置入口 (单设备内 PRIMARY/SECONDARY/SOFTWARE_TRIGGER 等) ---
    /// 获取单设备硬件同步管理器 (若设备不支持则返回 nullptr)
    virtual std::shared_ptr<class ISyncManager> getSyncManager() = 0;
};
```

**StreamIndex / StreamProfile / StreamType (V0.3.1/V0.3.2):**
```cpp
/// 流类型枚举 (V0.3.2 扩展)
enum class StreamType : uint8_t {
    COLOR, DEPTH, IR_LEFT, IR_RIGHT, FISHEYE,
    GYRO, ACCEL, MOTION,   // IMU 流
    LIDAR, LASER_SCAN,     // LiDAR 流
    UNKNOWN = 0xFF,
};

/// 统一流标识符 — 区分同类型多路流 (V0.3.2 新增)
/// 双厂商均使用 stream_index_pair = pair<StreamType, int> 作为全局 key
/// index 用于区分: IR_LEFT(0)/IR_RIGHT(0), COLOR_LEFT(0)/COLOR_RIGHT(1) 等
struct StreamIndex {
    StreamType  type  = StreamType::UNKNOWN;
    int         index = 0;    // 0 = 默认 (绝大多数单路流)

    bool operator==(const StreamIndex& o) const { return type == o.type && index == o.index; }
    bool operator<(const StreamIndex& o)  const {
        return type < o.type || (type == o.type && index < o.index);
    }
};

/// 时间戳域 (V0.3.2 新增) — 避免不同时钟域混用
/// Orbbec 有 3 种: device(硬件时钟) / system(主机时钟) / global(SDK 对齐后)
enum class TimestampDomain : uint8_t {
    Hardware,   // 设备硬件时钟 (最准, 但需与主机对时)
    System,     // 主机系统时钟 (接收时打戳)
    Global,     // SDK 对齐后的统一时钟 (若 SDK 支持 PTP)
};

/// 单条流配置描述 (V0.3.2: stream 字段换为 StreamIndex)
struct StreamProfile {
    StreamIndex     stream;       // 替换原 StreamType stream; index 默认 0
    int             width;
    int             height;
    int             fps;
    PixelEncoding   format;

    bool exactMatch(const StreamProfile& o) const;
    bool partialMatch(const StreamProfile& o) const; // 0 表示通配
};

/// 硬件选项类型 (V0.3.2 新增)
enum class OptionType : uint8_t { Bool, Int, Float, Enum };

/// 硬件选项描述 (V0.3.2: OptionRange 升级为 OptionInfo)
/// 新增: OptionType / description / enum_values (离散选项用)
struct OptionInfo {
    std::string name;
    std::string description;           // 人类可读说明 (来自 SDK option description)
    OptionType  type = OptionType::Float;
    float min          = 0.0f;
    float max          = 0.0f;
    float step         = 0.0f;
    float default_value = 0.0f;
    bool  is_readonly  = false;
    std::map<std::string, float> enum_values;  // 仅 Enum 类型有效 (如 sync_mode→{FreeRun:0, Primary:1,...})
};

/// ImageFrame 时间戳域字段 (V0.3.2 新增到原 ImageFrame 结构)
/// 在现有 ImageFrame 中补充以下字段:
// uint64_t       timestamp_ns;           // 已有
// TimestampDomain timestamp_domain;      // V0.3.2 新增: Hardware/System/Global
// uint64_t       frame_number;           // V0.3.2 新增: 设备帧序号 (单调递增)
// float          actual_exposure_us;     // V0.3.2 新增: 实际曝光时间 (μs, 0=未知)
// float          actual_gain;            // V0.3.2 新增: 实际增益 (dB, 0=未知)
// bool           auto_exposure_enabled;  // V0.3.2 新增: 当前 AE 状态
```

**5 态状态机:**
```
Closed ──configure()──→ Configured ──open()──→ Opened ──startStreaming()──→ Streaming
  ↑                                                                           │
  └──────────────────────close()──────────────────────────────────────────────┘
                                              任意状态 ──fault──→ Faulted ──reset()──→ Closed
```

---

### §2.1A PixelEncoding 独立枚举 (V0.3 新增)

> **问题**: 主文档 V3.0 的 `ImageFrame::Encoding` 仅 8~12 值且嵌套在 struct 内。
> Notion V0.2 定义了独立 `enum class PixelEncoding` 共 24 种。应统一采用 Notion 版本。

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/pixel_encoding.hpp
#pragma once
#include <cstdint>

namespace rm::hal::sensor {

enum class PixelEncoding : uint8_t {
    // === 彩色 (0x00~0x1F) ===
    RGB8        = 0x00,
    BGR8        = 0x01,
    RGBA8       = 0x02,
    BGRA8       = 0x03,
    YUYV        = 0x04,   // YUV422 packed
    UYVY        = 0x05,   // YUV422 packed
    NV12        = 0x06,   // YUV420 semi-planar
    NV21        = 0x07,   // YUV420 semi-planar (Android)
    I420        = 0x08,   // YUV420 planar
    M420        = 0x09,   // YUV420 variant

    // === 灰度 (0x20~0x2F) ===
    MONO8       = 0x20,   // 8-bit 灰度 (IR)
    MONO16      = 0x21,   // 16-bit 灰度

    // === 深度 (0x30~0x3F) ===
    Z16         = 0x30,   // 16-bit 深度 (mm)
    Z32F        = 0x31,   // 32-bit float 深度 (m)

    // === 压缩 (0x40~0x4F) ===
    MJPEG       = 0x40,
    H264        = 0x41,
    H265        = 0x42,
    HEVC        = H265,

    // === 红外 (0x50~0x5F) ===
    Y8          = 0x50,   // 等同 MONO8，SDK 偏好名
    Y16         = 0x51,   // 等同 MONO16

    // === 特殊 (0xF0~0xFF) ===
    RAW16       = 0xF0,   // Bayer 原始 16-bit
    CUSTOM      = 0xFF,
};

/// PixelEncoding → 可读字符串
const char* pixelEncodingToString(PixelEncoding enc);
/// 每像素字节数 (压缩格式返回 0)
int bytesPerPixel(PixelEncoding enc);
/// 判断是否为压缩格式
inline bool isCompressed(PixelEncoding enc) {
    return enc >= PixelEncoding::MJPEG && enc <= PixelEncoding::H265;
}

} // namespace rm::hal::sensor
```

**ImageFrame 相应修改**: 删除内嵌 `Encoding` enum，改用顶层 `PixelEncoding`:
```cpp
struct ImageFrame {
    PixelEncoding encoding = PixelEncoding::BGR8;  // 替换原嵌套 enum
    // ... 其余字段不变
};
```

**CameraConfig 补充字段 （V0.3）**:

**CameraIntrinsics 扩展 / DistortionModel / IMUCalibration (V0.3.2 新增):**
```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/calibration_types.hpp
namespace rm::hal::sensor {

/// 畸变模型枚举 (V0.3.2 新增)
/// 两家厂商使用的畸变模型各有不同, HAL 统一标注
enum class DistortionModel : uint8_t {
    None,
    BrownConrady,           // 5/8 参数 plumb_bob / opencv (径向+切向), 两家均支持
    InverseBrownConrady,    // RealSense D4xx 深度流使用
    KannalaBrandt4,         // 鱼眼镜头 (4 参数), fisheye 模组
};

/// 相机内参 (V0.3.2: 增加 distortion_model 字段)
struct CameraIntrinsics {
    float fx = 0, fy = 0;   // 焦距 (像素)
    float cx = 0, cy = 0;   // 主点
    int   width = 0, height = 0;
    DistortionModel distortion_model = DistortionModel::BrownConrady;
    float distortion_coeffs[8] = {0};  // k1,k2,p1,p2,k3[,k4,k5,k6] or fisheye k1~k4
};

/// IMU 标定参数 (V0.3.2 新增)
/// 来源: realsense-ros IMU calibration / Orbbec IMU params
/// 由 ICameraHAL::getIMUCalibration(stream_type) 返回
struct IMUCalibration {
    StreamType stream = StreamType::GYRO;   // GYRO 或 ACCEL
    float scale_bias[12] = {0};            // 3×4 矩阵 (3×3 scale + 3×1 bias), 行主序
    float noise_variances[3] = {0};        // 量测噪声方差 [x,y,z]  (用于 EKF/UKF)
    float bias_variances[3]  = {0};        // 偏置随机游走方差 [x,y,z]
    bool valid = false;                    // SDK 是否提供了有效标定
};

/// 深度元数据 (V0.3.2 新增)
/// 由 ICameraHAL::getDepthMetadata() 返回
struct DepthMetadata {
    float depth_scale;        // 1 LSB 对应的物理距离 (米), Orbbec 通常 0.001
    float depth_min_meters;   // 有效量程下界 (米)
    float depth_max_meters;   // 有效量程上界 (米)
};

} // namespace rm::hal::sensor
```

**ICameraHAL 标定查询方法 (V0.3.2 增补, 在 getIntrinsics/getExtrinsics 之后):**
```cpp
    // --- 扩展标定查询 (V0.3.2 增补) ---
    /// 获取 IMU 传感器标定参数 (设备有内置 IMU 时有效)
    virtual IMUCalibration getIMUCalibration(StreamType imu_stream) const = 0;
    /// 获取深度量程元数据 (depth_scale / depth_min / depth_max)
    virtual DepthMetadata getDepthMetadata() const = 0;
    /// 加载用户自定义标定文件 (YAML, 覆盖出厂标定)
    virtual bool loadUserCalibration(const std::string& yaml_path) = 0;
    /// 导出当前标定到 YAML 字符串
    virtual std::string exportCalibration() const = 0;
```

```cpp
struct CameraConfig {
    // ==== 原有字段 ====
    std::string device_id;
    int width = 1280, height = 720;
    int fps = 30;
    bool enable_color = true;
    bool enable_depth = true;
    bool enable_ir = false;
    std::string serial_number;
    std::unordered_map<std::string, std::string> extra_params;

    // ==== V0.3 新增字段 ====
    PixelEncoding color_encoding = PixelEncoding::BGR8;  // 彩色流像素格式
    int depth_width = 640;       // 深度流分辨率 (可与彩色不同)
    int depth_height = 480;
    int depth_fps = 30;
    int ring_buffer_depth = 4;   // 内部帧队列深度
    std::string align_mode;      // "none" / "depth_to_color" / "color_to_depth"
    std::string frame_aggregate_mode = "ANY"; // "full_frame"/"color_frame"/"ANY"/"disable"

    // GMSL 专用
    std::string usb_port;         // GMSL 通道标识 "gmsl2-1" / "gmsl2-3" 等
    bool enable_gmsl_trigger = false;
    int gmsl_trigger_fps = 3000;  // GMSL 硬件触发频率
    std::string sync_mode = "free_run";  // "free_run"/"primary"/"secondary"/"hardware_triggering"

    // V4L2 通用
    std::string v4l2_node;        // "/dev/video0" (仅 UsbCameraHAL 使用)
};
```

---

### §2.4 ErrorCode 定义

> **补全 Notion V0.2 §2.4 TODO**

HAL 层统一错误码。V3.4 各接口暂仍返回 `bool`，V3.5 规划切换为 `ErrorCode`。本节提前定义枚举，Driver 开发时可内部使用，待 V3.5 正式切换。

```cpp
// rm_hal_common/include/rm_hal_common/error_code.hpp
#pragma once
#include <string>

namespace rm::hal {

enum class ErrorCode : int32_t {
    // === 通用 (0~99) ===
    OK                  = 0,
    UNKNOWN             = 1,
    NOT_IMPLEMENTED     = 2,

    // === 设备生命周期 (100~199) ===
    DEVICE_NOT_FOUND    = 100,  // 设备未发现（枚举/序列号不匹配）
    DEVICE_BUSY         = 101,  // 设备被其他进程占用
    DEVICE_DISCONNECTED = 102,  // 运行中设备断开（热拔）
    INVALID_STATE       = 103,  // 当前状态不允许该操作（如 Closed 态调 startStreaming）
    ALREADY_OPEN        = 104,  // 重复 open()
    NOT_OPEN            = 105,  // 未 open() 就调用数据方法

    // === 配置 (200~299) ===
    INVALID_CONFIG      = 200,  // 配置参数校验失败
    UNSUPPORTED_FORMAT  = 201,  // 请求的 PixelEncoding 设备不支持
    UNSUPPORTED_RESOLUTION = 202,
    UNSUPPORTED_FPS     = 203,

    // === 数据/IO (300~399) ===
    TIMEOUT             = 300,  // 数据获取/指令发送超时
    IO_ERROR            = 301,  // 底层 IO 错误（串口/USB/网络/CAN）
    FRAME_DROPPED       = 302,  // 帧丢失（ring buffer 溢出或 SDK 丢帧）
    CRC_ERROR           = 303,  // 协议 CRC 校验失败
    BUFFER_OVERFLOW     = 304,  // 内部缓冲区溢出

    // === SDK/驱动 (400~499) ===
    SDK_ERROR           = 400,  // 厂商 SDK 返回错误（详情在 error_msg）
    SDK_NOT_INITIALIZED = 401,  // SDK 未初始化
    FIRMWARE_MISMATCH   = 402,  // 固件版本不兼容

    // === 权限/资源 (500~599) ===
    PERMISSION_DENIED   = 500,  // 设备权限不足（如 /dev/video* 无权限）
    RESOURCE_EXHAUSTED  = 501,  // 系统资源耗尽（fd/memory/GPU）
};

/// 错误码转可读字符串
const char* errorCodeToString(ErrorCode code);

/// 扩展错误信息
struct ErrorInfo {
    ErrorCode code = ErrorCode::OK;
    std::string message;         // 人类可读描述
    std::string sdk_error_detail; // 厂商 SDK 原始错误（可选）
};

} // namespace rm::hal
```

**使用约定:**
- M1 阶段：Driver 内部使用 ErrorCode 做日志和诊断，对外接口仍返回 `bool`
- V3.5 切换：`bool open()` → `ErrorCode open()` 或 `ErrorInfo open()`
- Driver 实现需在 `health().error_msg` 中填充最后一次错误的文本描述

---

### §2.5 线程与并发模型

> **补全 Notion V0.2 §2.5 TODO**

#### 2.5.1 线程角色

Camera HAL 实现内部涉及以下线程角色：

| 线程 | 归属 | 职责 | 生命周期 |
|------|------|------|---------|
| **SDK 回调线程** | 厂商 SDK (OrbbecSDK/librealsense2) | 帧数据到达时触发回调 | SDK Pipeline start ~ stop |
| **解码线程** (可选) | HAL Driver 内部 | MJPEG/H264 → RGB 解码 | startStreaming ~ stopStreaming |
| **用户回调线程** | HAL Driver 内部 | 执行用户注册的 FrameCallback | startStreaming ~ stopStreaming |
| **调用者线程** | Module 层 | 调用 getColorFrame 等轮询方法 | 由调用者控制 |

#### 2.5.2 数据流线程模型

```
SDK 回调线程 ──push──→ [内部帧队列 (bounded)] ──pop──→ 用户回调线程 ──调用──→ FrameCallback
                                                        │
                                                        └── 同时更新 ring buffer (供轮询)
```

**关键设计决策:**

1. **SDK 回调线程不直接执行用户回调**: SDK 线程持有内部锁，直接回调可能导致死锁（用户回调内调 HAL 方法）。必须转移到独立的用户回调线程。

2. **内部帧队列有界**: 队列深度 = `CameraConfig::ring_buffer_depth`（默认 4）。队列满时丢弃最旧帧（非阻塞生产者），防止 SDK 回调线程被阻塞。

3. **回调与轮询互斥**: 同一流（color/depth/IR）上设置了 callback 时，`getXxxFrame()` 轮询方法返回 false（INVALID_STATE）。未设置 callback 时使用 ring buffer 轮询模式。

4. **shared_ptr 帧生命周期**: 回调传递 `shared_ptr<const ImageFrame>`，接收方持有即可延长生命期。

#### 2.5.3 锁策略

| 锁 | 类型 | 保护对象 | 持有时机 |
|---|------|---------|---------|
| `state_mutex_` | `std::mutex` | 状态机转移 (configure/open/close/start/stop) | 生命周期方法调用期间 |
| `frame_queue_mutex_` | `std::mutex` | 内部帧队列 push/pop | 帧入队/出队时短暂持有 |
| `config_mutex_` | `std::mutex` | CameraConfig 读写 | configure() 和运行时参数查询 |

**不需要锁的场景:**
- `health()`: 返回原子读取的 HealthStatus，无需加锁
- `deviceId()`: 返回 configure() 时确定的不可变字符串
- `isOpen()`: 读取 atomic<bool>

#### 2.5.4 线程安全合同

- **ICameraHAL 实例不是线程安全的**: 调用者（Module 层）需确保不并发调用同一实例的方法
- **例外**: `health()` / `deviceId()` / `isOpen()` 可从任意线程安全调用
- **回调函数在专用线程执行**: 用户的 FrameCallback 在 HAL 内部的用户回调线程中被调用，不在 SDK 线程中
- **回调函数内不得调用同一 HAL 实例的方法**: 避免重入死锁

#### 2.5.5 与 OrbbecSDK 线程模型的对齐

OrbbecSDK 的实际线程模型：
```
SDK 内部 Pipeline 线程 ──onNewFrameSetCallback──→ SDK 回调线程
     │                                                │
     │   (颜色帧需要解码时)                            │
     └── colorFrameThread_(消费者线程) ── 独立线程做解码
```

HAL 封装策略：
- OrbbecCameraHAL 在 `onNewFrameSetCallback` 中将 `shared_ptr<ob::FrameSet>` 转换为 `ImageFrame`
- 如果帧格式为 MJPEG 需要解码，在解码线程中完成后再入队
- 转换后的 `ImageFrame` 入队到 HAL 内部队列

---

### §2.6 Adapter→SDK 实现映射

> **补全 Notion V0.2 §2.6 TODO**

#### 2.6.1 OrbbecCameraHAL (USB) — Adapter 映射表

| ICameraHAL 方法 | OrbbecSDK API | 说明 |
|----------------|--------------|------|
| `configure(CameraConfig)` | `ob::Pipeline()` + `ob::Config::enableStream()` + `Config::setAlignMode()` + `Config::setFrameAggregateOutputMode()` | 创建 Pipeline，按 CameraConfig 中的分辨率/帧率/格式配置各流 |
| `open()` | `context->queryDeviceList()` + `deviceList->getDevice(idx)` | 按 device_id (序列号) 枚举并打开设备 |
| `close()` | `pipeline->stop()` + 释放 device/pipeline | 停止数据流并释放资源 |
| `startStreaming()` | `pipeline->start(config, callback)` | 启动 Pipeline，注册 `onNewFrameSetCallback` |
| `stopStreaming()` | `pipeline->stop()` | 停止 Pipeline |
| `getColorFrame()` | 从内部 ring buffer 取最新帧 | SDK 回调→转换→ring buffer，轮询方式 |
| `getDepthFrame()` | 同上 | depth 流 |
| `getIRFrame()` | 同上 | IR/IR_LEFT/IR_RIGHT 流 |
| `getPointCloud()` | `ob::PointCloudFilter::process(frameset)` | 使用 SDK 内置 PointCloudFilter |
| `getColorIntrinsics()` | `device->getCameraIntrinsics(OB_SENSOR_COLOR)` | 返回 fx/fy/cx/cy |
| `getDepthIntrinsics()` | `device->getCameraIntrinsics(OB_SENSOR_DEPTH)` | 同上 |
| `getExtrinsics()` | `device->getCalibrationCameraToCamera(src, dst)` | Rotation[9] + Translation[3] |
| `setColorCallback()` | 内部路由：帧入队后通知用户回调线程 | 见 §2.5 线程模型 |
| `health()` | 内部维护 `alive` / `data_rate_hz` / `error_msg` | alive = Pipeline 运行中 && 最近 2s 内有帧 |
| `deviceId()` | `device->getDeviceInfo()->serialNumber()` | 设备序列号 |

**PixelEncoding 映射:**

| PixelEncoding (HAL) | ob::OBFormat (SDK) | 说明 |
|---------------------|-------------------|------|
| RGB8 | OB_FORMAT_RGB | RGB 24bit |
| BGR8 | OB_FORMAT_BGR | OpenCV 默认 |
| YUYV | OB_FORMAT_YUYV | YUV422 |
| MJPEG | OB_FORMAT_MJPG | 压缩格式，需解码 |
| Z16 | OB_FORMAT_Y16 | 16bit 深度 |
| Y8 | OB_FORMAT_Y8 | 8bit 灰度 (IR) |
| NV21 | OB_FORMAT_NV21 | Android 常见 |
| H264 | OB_FORMAT_H264 | 压缩视频流 |

#### 2.6.2 OrbbecGmslCameraHAL (Orin GMSL2) — Adapter 映射表

> **V0.3 关键修正**: 经 OrbbecSDK_ROS2 源码分析，Realman 使用的 Orbbec Gemini 330 GMSL 版本
> **并非**独立 V4L2 驱动。GMSL 仅是物理传输层，SDK 抽象与 USB 版完全一致。
> 唯一差异是需额外管理 `/dev/camsync` 硬件触发同步。因此 GmslCameraHAL 应
> **继承/组合 OrbbecCameraHAL**，仅扩展 GMSL 触发逻辑。

##### 架构关系

```
OrbbecCameraHAL (USB)         ← §2.6.1 完整 SDK API 映射
    ↑ 继承
OrbbecGmslCameraHAL (GMSL2)   ← 本节: 仅扩展 trigger + 多机同步
    └── /dev/camsync 硬件触发
    └── 设备寻址: usb_port = "gmsl2-X" (替代 serial_number)
```

##### GMSL 相对于 USB 的差异项

| 差异项 | USB (§2.6.1) | GMSL2 (本节) |
|--------|-------------|-------------|
| 物理链路 | USB 3.0 | GMSL2 (Maxim SerDes) |
| 设备寻址 | `serial_number` | `usb_port: "gmsl2-{1,3,5,7}"` |
| SDK 接口 | OrbbecSDK `ob::Pipeline` | **同左** (完全一致) |
| 多机同步 | `sync_mode` 软件/主从 | `hardware_triggering` + `/dev/camsync` |
| 触发管理 | 无 | `openSocSyncPwmTrigger()` / `closeSocSyncPwmTrigger()` |
| 帧聚合 | 按需 | 建议 `full_frame` (4 机硬同步) |
| 时间域 | device / system | 建议 `global` (PTP) |

##### /dev/camsync 硬件触发协议

```cpp
// 内核驱动 camsync: SoC PWM → GMSL2 SerDes → 各相机同步曝光
#define DEVICE_PATH "/dev/camsync"

struct cs_param_t {
    uint8_t  mode;   // 1 = enable trigger, 0 = disable
    uint16_t fps;    // 触发频率，单位 0.01Hz (3000 = 30.00 Hz)
} __attribute__((packed));

// 启动触发
int openSocSyncPwmTrigger(uint16_t fps) {
    cs_param_t param = {1, fps};
    int fd = open(DEVICE_PATH, O_RDWR);
    write(fd, &param, sizeof(param));     // 写入模式+频率
    cs_param_t rd;
    read(fd, &rd, sizeof(rd));            // 回读确认
    return fd;                            // 保持 fd 打开 = 持续触发
}

// 停止触发
void closeSocSyncPwmTrigger(int fd) {
    close(fd);  // 关闭 fd 即停止 PWM 触发
}
```

##### ICameraHAL 方法映射 (增量)

| ICameraHAL 方法 | OrbbecGmslCameraHAL 实现 | 说明 |
|----------------|------------------------|------|
| `configure()` | 父类 `OrbbecCameraHAL::configure()` + 保存 GMSL 参数 | `enable_gmsl_trigger` / `gmsl_trigger_fps` / `sync_mode` |
| `open()` | 按 `usb_port` (而非 serial_number) 查找设备 | `ctx->queryDeviceList()` 后按 port 匹配 |
| `startStreaming()` | 父类 `pipeline->start()` **之后** 调用 `startGmslTrigger()` | 先启动 SDK pipeline 再开 PWM |
| `stopStreaming()` | 先调用 `stopGmslTrigger()` **再** `pipeline->stop()` | 先停 PWM 再停 SDK |
| 其余方法 | 完全委托父类 | getColorFrame/getDepthFrame/getPointCloud 等无差异 |

##### GMSL 专用参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_gmsl_trigger` | bool | false | 是否启用 SoC PWM 硬件触发 |
| `gmsl_trigger_fps` | int | 3000 | 触发频率 (单位 0.01 Hz，3000=30Hz) |
| `usb_port` | string | "" | GMSL 通道标识，如 `"gmsl2-1"` |
| `sync_mode` | string | "hardware_triggering" | GMSL 多机场景应设为硬件触发 |
| `device_num` | int | 1 | 同步设备数量 (4 机时设 4) |

##### Realman 平台 GMSL 实际配置 (验证参考)

```yaml
# 4× Gemini 330 GMSL 相机 — 真机部署配置
cameras:
  head_depth_cam:
    usb_port: "gmsl2-1"
    sync_mode: "hardware_triggering"
    device_num: 4
    enable_gmsl_trigger: true
    gmsl_trigger_fps: 3000   # 30Hz
  chest_depth_cam:
    usb_port: "gmsl2-3"
    sync_mode: "hardware_triggering"
    device_num: 4
  left_arm_depth_cam:
    usb_port: "gmsl2-5"
    sync_mode: "hardware_triggering"
    device_num: 4
  right_arm_depth_cam:
    usb_port: "gmsl2-7"
    sync_mode: "hardware_triggering"
    device_num: 4
# 注: 仅一台相机设 enable_gmsl_trigger=true (主触发源)
# 其余相机通过 GMSL2 硬件同步线跟随触发
```

##### 多相机启动顺序

```
1. 创建共享进程容器 (component_container_mt)
2. 依次加载 4 个 OrbbecGmslCameraHAL 实例，间隔 2s (避免 USB 枚举冲突)
3. 所有 pipeline->start() 完成后
4. 主相机 (head) 执行 openSocSyncPwmTrigger(3000)
5. 所有相机开始同步出帧 (硬件触发 PWM)
```

##### IGmslTrigger 平台抽象层 (V0.3 新增)

> **背景**: `/dev/camsync` 是 NVIDIA Jetson (Orin NX) 特有的内核驱动，负责 SoC PWM → GMSL2
> SerDes 触发。未来 S100 平台 (不同 SoC) 的 GMSL 触发机制可能不同。为避免重复开发，
> 将触发逻辑抽象为 `IGmslTrigger` 接口，平台相关实现单独封装。

```
┌──────────────────────────────────────┐
│     OrbbecGmslCameraHAL              │ ← OrbbecSDK 层 (平台无关)
│  ┌────────────────────────────────┐  │
│  │     IGmslTrigger (interface)   │  │ ← 触发抽象层
│  └────────┬───────────┬───────────┘  │
│           │           │              │
│  ┌────────▼──┐ ┌──────▼──────┐ ┌────▼────────┐
│  │ OrinNX    │ │ S100        │ │ Sim         │
│  │ Trigger   │ │ Trigger     │ │ Trigger     │
│  │/dev/      │ │ (TBD:       │ │ (software   │
│  │camsync    │ │ vendor SDK  │ │  timer)     │
│  │+cs_param_t│ │ / sysfs)    │ │             │
│  └───────────┘ └─────────────┘ └─────────────┘
└──────────────────────────────────────┘
```

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/gmsl_trigger.hpp
#pragma once
#include <cstdint>
#include <memory>

namespace rm::hal {

/// GMSL 硬件触发抽象接口 — 隔离平台差异
class IGmslTrigger {
public:
    virtual ~IGmslTrigger() = default;

    /// 启动 PWM 同步触发
    /// @param fps_hundredths 触发频率, 单位 0.01Hz (3000 = 30.00 Hz)
    /// @return 成功返回 true
    virtual bool startTrigger(uint16_t fps_hundredths) = 0;

    /// 停止触发
    virtual void stopTrigger() = 0;

    /// 查询当前触发状态
    virtual bool isTriggering() const = 0;
};

/// Orin NX 实现: /dev/camsync + cs_param_t
class GmslTriggerOrinNX final : public IGmslTrigger {
public:
    bool startTrigger(uint16_t fps_hundredths) override;
    void stopTrigger() override;
    bool isTriggering() const override;
private:
    int fd_ = -1;   // /dev/camsync 文件描述符
};

/// S100 实现: 待定 (vendor SDK / sysfs / 其他触发路径)
class GmslTriggerS100 final : public IGmslTrigger {
public:
    bool startTrigger(uint16_t fps_hundredths) override;  // TODO: S100 触发协议
    void stopTrigger() override;
    bool isTriggering() const override;
private:
    // S100 平台特定资源
};

/// Sim 实现: 纯软件定时器, 无硬件依赖
class GmslTriggerSim final : public IGmslTrigger {
public:
    bool startTrigger(uint16_t fps_hundredths) override;  // 启动软件定时器
    void stopTrigger() override;
    bool isTriggering() const override;
private:
    bool running_ = false;
};

} // namespace rm::hal
```

> **设计决策**: OrbbecSDK 自身是平台无关的 (USB 和 GMSL 均通过同一 SDK 接口)。
> 只有**触发机制**是平台相关的。因此:
> - 换平台 (Orin NX → S100): 只需实现新的 `IGmslTrigger` 子类，OrbbecSDK 调用层**零修改**
> - OrbbecGmslCameraHAL 通过构造函数注入 `std::unique_ptr<IGmslTrigger>`
> - HALFactory 根据 profile YAML 中的 `platform` 字段选择对应 Trigger 实现

##### 原设计 vs 实际 (勘误)

| 原设计 (V0.1~V0.2) | 实际 (V0.3) |
|-------------------|------------|
| GMSL 走独立 V4L2 驱动 + MAX9296 deserializer | GMSL 走 OrbbecSDK (透明传输) |
| 需 libargus / NvMedia ISP | 不需要 (SDK 内部处理) |
| 每个 GMSL 相机对应 `/dev/video*` | 每个 GMSL 相机按 `usb_port` 寻址 |
| 帧同步依赖 V4L2 多路 DMA | 帧同步依赖 `/dev/camsync` PWM |
| GmslCameraHAL 完全独立实现 | OrbbecGmslCameraHAL **继承** OrbbecCameraHAL |
| GMSL 触发逻辑硬编码 Orin NX | 通过 IGmslTrigger 接口抽象, 支持多平台 |

#### 2.6.3 RealsenseCameraHAL — Adapter 映射表

| ICameraHAL 方法 | librealsense2 API | 说明 |
|----------------|------------------|------|
| `configure()` | `rs2::config::enable_stream()` | 配置各流 |
| `open()` | `rs2::pipeline::start(config)` | RealSense 的 start = open + streaming |
| `startStreaming()` | 已在 open() 中完成，或重新 start() | |
| `getColorFrame()` | `pipeline.wait_for_frames().get_color_frame()` | |
| `getDepthFrame()` | `frameset.get_depth_frame()` | |
| `getPointCloud()` | `rs2::pointcloud::calculate(depth)` | |
| `getColorIntrinsics()` | `stream_profile.get_intrinsics()` | |
| `getExtrinsics()` | `stream_profile.get_extrinsics_to(other)` | |

#### 2.6.4 UsbCameraHAL (V4L2 通用) — Adapter 映射表 (**V0.3 新增**)

> 用于通用 USB UVC 摄像头 (无深度/IR 能力)，基于 V4L2 直接驱动。

| ICameraHAL 方法 | V4L2 / usb_cam API | 说明 |
|----------------|-------------------|------|
| `configure()` | 保存分辨率/帧率/像素格式参数 | 不打开设备 |
| `open()` | `::open(dev_path, O_RDWR)` + `VIDIOC_S_FMT` + `VIDIOC_REQBUFS` + `mmap` | V4L2 初始化 |
| `startStreaming()` | `VIDIOC_STREAMON` + `VIDIOC_QBUF` (mmap buffers) | 开始捕获 |
| `stopStreaming()` | `VIDIOC_STREAMOFF` | |
| `getColorFrame()` | `VIDIOC_DQBUF` → 读取 buffer → `VIDIOC_QBUF` 回收 | 轮询模式 |
| `getDepthFrame()` | 返回 false | USB 通用摄像头无深度流 |
| `getIRFrame()` | 返回 false | 同上 |
| `getPointCloud()` | 返回 false | 无深度数据 |
| `getColorIntrinsics()` | 返回 `valid=false` 的零值默认内参，或从配置文件加载 | V4L2 无标定数据接口; 调用方需检查 `CameraIntrinsics::valid` 字段 |
| `close()` | `munmap` + `::close(fd)` | |
| `health()` | fd 有效 + 帧率统计 | alive = fd≥0 && 最近 2s 有帧 |

**V4L2 像素格式映射:**

| PixelEncoding (HAL) | V4L2 FourCC | 说明 |
|---------------------|-------------|------|
| YUYV | `V4L2_PIX_FMT_YUYV` | 最常见硬件输出 |
| MJPEG | `V4L2_PIX_FMT_MJPEG` | 压缩模式 (省带宽) |
| RGB8 | `V4L2_PIX_FMT_RGB24` | 部分设备不支持 |
| BGR8 | `V4L2_PIX_FMT_BGR24` | |
| MONO8 | `V4L2_PIX_FMT_GREY` | 灰度摄像头 |
| H264 | `V4L2_PIX_FMT_H264` | UVC 1.5+ |

**格式转换能力 (可复用 usb_cam 存量代码):**
- YUYV → RGB8 (`yuyv2rgb`)
- MJPEG → RGB8 (`mjpeg2rgb`，依赖 libjpeg/libav)
- UYVY → RGB8
- M420 → RGB8

#### 2.6.5 SimCameraHAL — 实现规范 (**V0.3 新增**)

> 纯软件模拟，无硬件依赖。用于 CI 测试、算法开发、离线调试。

##### 行为规范

| 参数 | 规范 |
|------|------|
| 帧生成频率 | 按 `CameraConfig.fps` 生成，默认 30Hz |
| Color 帧内容 | 配置分辨率 BGR8，合成渐变图 + 帧号水印 |
| Depth 帧内容 | 同分辨率 Z16，值域 [500, 5000] mm 正弦波平面 |
| IR 帧内容 | 同分辨率 MONO8，固定灰度 128 |
| 时间戳 | `rm::hal::now_ns()` (系统单调时钟) |
| Intrinsics | 理想针孔: fx=fy=525, cx=w/2, cy=h/2, 畸变=0 |
| Extrinsics | 单位变换: R=I₃, T=0 |
| PointCloud | 根据合成深度 + 理想内参反投影生成 XYZ |
| 延迟模拟 | 可配置 0~5ms 随机抖动 |
| 丢帧模拟 | 可配置丢帧率 (0~1) |

##### Sim 专用配置 (通过 `extra_params` 传入)

```cpp
// CameraConfig.extra_params 中的 Sim 专用 key:
// "sim_jitter_ms"  = "3"          // 随机延迟上限 (ms)
// "sim_drop_rate"  = "0.01"       // 1% 丢帧率
// "sim_pattern"    = "gradient"   // gradient / checkerboard / noise
```

#### 2.6.6 Camera Adapter 对比总表 (V0.3 新增)

| 维度 | OrbbecCameraHAL (USB) | OrbbecGmslCameraHAL | RealsenseCameraHAL | UsbCameraHAL (V4L2) | SimCameraHAL |
|------|----------------------|--------------------|--------------------|---------------------|-------------|
| **SDK/API** | OrbbecSDK | OrbbecSDK + IGmslTrigger | librealsense2 | V4L2 ioctl | 无 (纯软件) |
| **物理链路** | USB 3.0 | GMSL2 (Maxim SerDes) | USB 3.0 | USB 2.0/3.0 | N/A |
| **同步方式** | SDK 软件同步 | /dev/camsync 硬件触发 | HW sync (GPIO) | 无 | 软件定时器 |
| **流类型** | Color + Depth + IR + PointCloud | 同左 | Color + Depth + IR + PointCloud | Color 仅 | Color + 合成 Depth |
| **设备寻址** | serial_number | usb_port (gmsl2-X) | serial_number | /dev/videoN | N/A |
| **平台依赖** | 无 | Orin NX (可扩展 S100) | 无 | Linux | 无 |
| **格式转换** | SDK 内部 | SDK 内部 | SDK 内部 | 需自行 (YUYV→RGB) | 合成 |
| **CMake target** | rm_hal_sensor_orbbec_usb | rm_hal_sensor_gmsl_orbbec_orin | rm_hal_sensor_realsense | rm_hal_sensor_usb_cam | rm_hal_sensor_camera_sim |
| **AI 开发就绪度** | 90% | 85% (需实现 IGmslTrigger) | 30% | 85% | 80% |

---

### §2.7 CMake target 与文件布局

> **补全 Notion V0.2 §2.7 TODO**

#### 2.7.1 Camera HAL CMake 结构

```
src/hal/rm_hal_sensor/
├── CMakeLists.txt                    # 顶层：add_subdirectory 各子目录
├── interface/
│   ├── CMakeLists.txt                # INTERFACE library (纯头文件)
│   └── include/rm_hal_sensor/
│       ├── sensor_hal_base.hpp
│       ├── camera_hal.hpp
│       ├── sensor_factory.hpp
│       └── ...
├── common/camera/
│   ├── orbbec_usb/
│   │   ├── CMakeLists.txt            # target: rm_hal_sensor_orbbec_usb
│   │   ├── orbbec_camera_hal.hpp
│   │   └── orbbec_camera_hal.cpp
│   ├── realsense/
│   │   ├── CMakeLists.txt            # target: rm_hal_sensor_realsense
│   │   ├── realsense_camera_hal.hpp
│   │   └── realsense_camera_hal.cpp
│   └── usb_cam/
│       ├── CMakeLists.txt            # target: rm_hal_sensor_usb_cam
│       ├── usb_camera_hal.hpp
│       └── usb_camera_hal.cpp
├── orin/camera/
│   └── gmsl_orbbec/
│       ├── CMakeLists.txt            # target: rm_hal_sensor_gmsl_orbbec_orin
│       ├── gmsl_camera_hal.hpp
│       └── gmsl_camera_hal.cpp
└── sim/camera/
    └── camera_sim/
        ├── CMakeLists.txt            # target: rm_hal_sensor_camera_sim
        └── sim_camera_hal.cpp
```

#### 2.7.2 CMakeLists.txt 模板 (叶子 Driver)

```cmake
# src/hal/rm_hal_sensor/common/camera/orbbec_usb/CMakeLists.txt
project(rm_hal_sensor_orbbec_usb)

find_package(OrbbecSDK REQUIRED)

add_library(${PROJECT_NAME} STATIC
    orbbec_camera_hal.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${OrbbecSDK_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC  rm_hal_sensor_interface    # HAL 接口头文件
    PRIVATE rm_hal_common              # hal_clock / hal_types
    PRIVATE OrbbecSDK::OrbbecSDK       # 厂商 SDK
)

# 禁止依赖中间件
# target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp)  # ❌ 禁止
```

#### 2.7.3 interface CMakeLists.txt

```cmake
# src/hal/rm_hal_sensor/interface/CMakeLists.txt
project(rm_hal_sensor_interface)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE rm_hal_common  # IHardwareDevice + HealthStatus

#### 2.7.4 IDecoder/IDecoderFactory — 平台解码器抽象 (**V0.3.2 新增**)

> **来源**: Orbbec 在三个平台使用不同 MJPEG/H264 解码器，通过 CMake 编译选项选择。
> 此为 HAL 内部实现细节（不暴露到 Module 层），但需在 CMake 中显式管理。

```
MJPEG / H264 帧
    │
    ├── [Jetson 平台] USE_NV_HW_DECODER
    │   └── JetsonNVDecoder (NvJPEGDecoder + NvBufferTransform)
    │
    ├── [Rockchip 平台] USE_RK_HW_DECODER
    │   └── RkMppDecoder (MPP + librga/libyuv)
    │
    └── [通用 x86/ARM] 默认
        └── SoftwareDecoder (libjpeg-turbo tjDecompress2)
```

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/decoder.hpp
namespace rm::hal::sensor {

/// 压缩帧解码器接口 (HAL 内部使用, 不暴露到 Module 层)
class IDecoder {
public:
    virtual ~IDecoder() = default;
    /// 解码压缩数据 → 原始 BGR/RGB 数据
    virtual bool decode(const uint8_t* compressed, size_t size,
                        uint8_t* output_rgb, int width, int height) = 0;
    virtual PixelEncoding outputFormat() const = 0;  // BGR8 or RGB8
};

/// 解码器工厂 (编译期平台选择)
class IDecoderFactory {
public:
    static std::unique_ptr<IDecoder> create();
    // 内部实现依据 CMake define 选择:
    // #ifdef USE_NV_HW_DECODER  → JetsonNVDecoder
    // #elif USE_RK_HW_DECODER   → RkMppDecoder
    // #else                     → SoftwareDecoder (libjpeg-turbo)
};

} // namespace
```

```cmake
# rm_hal_sensor/common/camera/orbbec_usb/CMakeLists.txt (节选)
option(USE_RK_HW_DECODER "Enable Rockchip MPP hardware decoder" OFF)
option(USE_NV_HW_DECODER "Enable NVIDIA Jetson hardware decoder" OFF)

if(USE_NV_HW_DECODER)
    find_package(CUDA REQUIRED)
    target_link_libraries(rm_hal_sensor_orbbec_usb PRIVATE nvjpeg nvbufsurface)
    target_compile_definitions(rm_hal_sensor_orbbec_usb PRIVATE USE_NV_HW_DECODER)
elseif(USE_RK_HW_DECODER)
    find_library(RKMPP_LIBRARY rockchip_mpp REQUIRED)
    find_library(RKRGA_LIBRARY rga REQUIRED)
    target_link_libraries(rm_hal_sensor_orbbec_usb PRIVATE ${RKMPP_LIBRARY} ${RKRGA_LIBRARY})
    target_compile_definitions(rm_hal_sensor_orbbec_usb PRIVATE USE_RK_HW_DECODER)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
    target_link_libraries(rm_hal_sensor_orbbec_usb PRIVATE ${TURBOJPEG_LIBRARIES})
endif()
```

> **RMOS 平台对应**: Orin NX (机器人主板) → `USE_NV_HW_DECODER=ON`; S100 平台 → 待确认。
> 此设计确保同一套 HAL 接口代码可跨平台编译, 仅底层解码实现不同。
)
```

---

### §2.8 异常与错误场景

> **补全 Notion V0.2 §2.8 TODO**

#### 2.8.1 场景清单

| 场景 | 触发条件 | HAL 行为 | Module 层响应建议 |
|------|---------|---------|------------------|
| **设备未找到** | open() 时序列号不匹配 | 返回 false + DEVICE_NOT_FOUND | 重试枚举或切换 sim |
| **设备被占用** | 另一个进程已 open 同一设备 | 返回 false + DEVICE_BUSY | 等待或终止冲突进程 |
| **USB 热拔** | 运行中 USB 断开 | 状态→Faulted，回调停止 | 监听 health().alive，触发重连 |
| **GMSL 链路故障** | deserializer 丢失同步 | error_msg 报告，帧率降零 | reset() 重试 |
| **SDK 崩溃** | 厂商 SDK 内部异常 | 捕获异常→Faulted + SDK_ERROR | reset()，必要时重启进程 |
| **帧超时** | 持续 N 秒无帧到达 | health().alive = false | Module 决定 reset() 或告警 |
| **格式不支持** | configure() 请求不支持的格式 | 返回 false + UNSUPPORTED_FORMAT | 降级到设备默认格式 |
| **Ring buffer 溢出** | 消费者慢于生产者 | 丢弃最旧帧，日志 WARN | 优化消费者或降帧率 |
| **内存不足** | 分配帧 buffer 失败 | 返回 false + RESOURCE_EXHAUSTED | 减少同时打开的相机数 |

#### 2.8.2 故障恢复策略

```
Module 检测 health().alive == false
    ├── 第 1 次: reset() (= close + open)
    ├── 第 2 次: 等待 3s + reset()
    ├── 第 3 次: 销毁实例 + 重新 createCameraHAL + configure + open
    └── 第 4 次: 放弃，上报系统级告警
```

#### 2.8.3 超时参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| frame_timeout_ms | 3000 | 轮询 getXxxFrame 等待超时 |
| open_timeout_ms | 5000 | open() SDK 初始化超时 |
| health_alive_threshold_s | 2.0 | 超过此时间无帧则 alive=false |

---

### §2.9 单测要点

> **补全 Notion V0.2 §2.9 TODO**

#### 2.9.1 测试分层

| 层级 | 测试对象 | 方法 | 依赖 |
|------|---------|------|------|
| **接口合同测试** | ICameraHAL 接口语义 | SimCameraHAL 作为被测对象 | 无 SDK 依赖 |
| **Driver 单测** | OrbbecCameraHAL 等 | Mock SDK (gmock) | MockOrbbecSDK |
| **集成测试** | 真实设备 + Driver | 物理设备连接 | 真实 SDK + 设备 |

#### 2.9.2 接口合同测试用例

```cpp
// test/test_camera_hal_contract.cpp
// 使用 SimCameraHAL 验证状态机语义

TEST(CameraHALContract, LifecycleHappyPath) {
    auto cam = createCameraHAL("sim");
    CameraConfig cfg{.device_id = "sim_cam_0", .width = 640, .height = 480};
    ASSERT_TRUE(cam->configure(cfg));      // Closed → Configured
    ASSERT_TRUE(cam->open());              // Configured → Opened
    ASSERT_TRUE(cam->startStreaming());     // Opened → Streaming
    ASSERT_TRUE(cam->stopStreaming());      // Streaming → Opened
    ASSERT_TRUE(cam->close());             // Opened → Closed
}

TEST(CameraHALContract, InvalidStateTransitions) {
    auto cam = createCameraHAL("sim");
    ASSERT_FALSE(cam->open());             // Closed 态不能直接 open (未 configure)
    ASSERT_FALSE(cam->startStreaming());    // Closed 态不能 startStreaming
}

TEST(CameraHALContract, DoubleClose) {
    auto cam = createCameraHAL("sim");
    // ... configure + open ...
    ASSERT_TRUE(cam->close());
    ASSERT_TRUE(cam->close());             // 幂等: 对已关闭设备 close() 返回 true
}

TEST(CameraHALContract, HealthWhenStreaming) {
    auto cam = createCameraHAL("sim");
    // ... configure + open + startStreaming ...
    auto h = cam->health();
    ASSERT_TRUE(h.alive);
    ASSERT_GT(h.data_rate_hz, 0.0);
}

TEST(CameraHALContract, CallbackMode) {
    auto cam = createCameraHAL("sim");
    // ... configure + open ...
    std::atomic<int> count{0};
    cam->setColorCallback([&](auto frame) { count++; });
    cam->startStreaming();
    std::this_thread::sleep_for(200ms);
    ASSERT_GT(count.load(), 0);
}
```

#### 2.9.3 Mock SDK 策略

```cpp
// 对 OrbbecSDK 的 Mock 核心:
class MockObPipeline {
    void start(config, callback) { /* 启动模拟帧生成线程 */ }
    void stop() { /* 停止线程 */ }
};
// 通过编译期注入 (模板参数) 或链接期替换 (弱符号)
```

---

### §2.10 多相机同步架构 (V0.3 新增)

> **⚠ 层级归属修正 (V0.3.1)**:
> 经 RMOS V5 架构文档复核，**ISyncCoordinator 属于 Module 层（HAL 编排层），而非 HAL 层本身**。
> 理由: ISyncCoordinator 协调多个 ICameraHAL 实例的帧对齐，涉及跨设备编排逻辑，违反
> HAL "单设备抽象" 原则。RMOS V5 架构明确指出 "多传感器时间戳对齐由 Channel 层
> message_filter 支持"，ISyncCoordinator 是此机制在 Module 层的具象化实现。
>
> **保留本节原因**: 同步需求与 Camera HAL 设计强相关 (硬件触发、PTP 时钟、帧时间戳)，
> 保留于此供参考。实际代码应放在 Module Library 层的 `rm_sensor_module` 包中。
> 完整的上层需求清单见 **§7 上层需求清单**。

> **背景**: Realman 机器人当前使用 4× Orbbec Gemini 330 GMSL 相机，未来可能接入深云 (ShenYun)
> 等第三方深度相机。需要设计跨厂商的相机同步方案。

#### 2.10.1 同步层次模型

```
┌───────────────────────────────────────────────────────────┐
│                    ISyncCoordinator                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │ SyncGroup 0  │  │ SyncGroup 1  │  │ SyncGroup 2  │    │
│  │ (Orbbec×4)   │  │ (ShenYun×2)  │  │ (Mixed)      │    │
│  │ HARDWARE_ONLY│  │ HARDWARE_ONLY│  │ PTP_ALIGNED  │    │
│  └──────────────┘  └──────────────┘  └──────────────┘    │
│                                                           │
│  Time Base: PTP (IEEE 1588) or system monotonic clock     │
└───────────────────────────────────────────────────────────┘
```

#### 2.10.2 SyncPolicy 枚举

```cpp
enum class SyncPolicy : uint8_t {
    HARDWARE_ONLY,   // 同厂商硬件触发 (Orbbec /dev/camsync, RealSense GPIO)
    PTP_ALIGNED,     // 跨厂商: PTP 时间基准 + 软件对齐
    BEST_EFFORT,     // 尽力而为: 基于系统时钟对齐, 容忍 ±10ms 抖动
    NONE             // 无同步要求 (各相机独立运行)
};
```

#### 2.10.3 ISyncCoordinator 接口

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/sync_coordinator.hpp
#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <cstdint>

namespace rm::hal {

class ICameraHAL;  // forward declaration

/// 同步帧组: 同一时刻所有相机的帧集合
struct SyncFrameSet {
    uint64_t                    timestamp_ns;   // 对齐后的统一时间戳
    std::vector<CameraFrame>    color_frames;   // 按相机索引排列
    std::vector<CameraFrame>    depth_frames;
    uint32_t                    missing_mask;   // 位掩码: bit=1 表示该相机帧缺失
};

using SyncCallback = std::function<void(const SyncFrameSet&)>;

/// 多相机同步协调器接口
class ISyncCoordinator {
public:
    virtual ~ISyncCoordinator() = default;

    /// 添加相机到同步组
    virtual void addCamera(std::shared_ptr<ICameraHAL> camera, int group_id = 0) = 0;

    /// 设置同步策略
    virtual void setSyncPolicy(int group_id, SyncPolicy policy) = 0;

    /// 设置跨组对齐容忍度 (ns)
    virtual void setAlignTolerance(uint64_t tolerance_ns) = 0;

    /// 注册同步帧回调
    virtual void setSyncCallback(SyncCallback cb) = 0;

    /// 启动同步采集
    virtual bool start() = 0;

    /// 停止
    virtual void stop() = 0;
};

} // namespace rm::hal
```

#### 2.10.4 同步策略说明

| 场景 | SyncPolicy | 实现方式 | 精度 | 限制 |
|------|-----------|---------|------|------|
| 4× Orbbec GMSL (当前) | HARDWARE_ONLY | `/dev/camsync` PWM 硬件触发 | < 100μs | 仅同厂商 GMSL |
| 2× RealSense | HARDWARE_ONLY | GPIO sync (inter_cam_sync_mode) | < 1ms | 需物理 GPIO 连接 |
| Orbbec + 深云 (跨厂商) | PTP_ALIGNED | PTP 时间基准 + 帧时间戳匹配 | 1~5ms | 需 PTP 硬件支持 |
| 低要求场景 | BEST_EFFORT | 系统时钟 + 滑动窗口匹配 | 5~15ms | 无硬件要求 |
| 单台相机 / 录制回放 | NONE | 各自独立 | N/A | 无同步 |

#### 2.10.5 帧对齐算法

```
对齐窗口: tolerance_ns (默认 5ms = 5,000,000 ns)

每当任一相机回调新帧:
  1. 将帧存入环形缓冲 (按相机 ID)
  2. 取各缓冲中最新帧的时间戳
  3. max_ts - min_ts ≤ tolerance_ns ?
     → YES: 取出各缓冲最新帧, 组成 SyncFrameSet, 触发回调
     → NO:  丢弃最旧的帧, 设 missing_mask 对应位
  4. 缓冲超时 (2× tolerance): 强制输出 partial SyncFrameSet
```

#### 2.10.6 PTP 时间同步 (跨厂商方案)

> 当 `SyncPolicy == PTP_ALIGNED` 时，要求:

1. **网络层**: 所有相机和主机在同一 PTP 域 (IEEE 1588v2)
2. **相机端**: 支持 PTP slave 模式 (Orbbec Gemini 330 支持 `global` 时间域)
3. **主机端**: `ptp4l` + `phc2sys` 服务将 PTP 时钟同步到系统时钟
4. **对齐**: ISyncCoordinator 使用相机帧自带的 PTP 时间戳做帧对齐

> **深云相机集成**: 深云 SR 系列是否支持 PTP 需确认。若不支持，退回 `BEST_EFFORT` 策略
> (基于系统时钟接收时间做粗对齐)。

#### 2.10.7 与现有设计的关系

- ISyncCoordinator **属于 Module 层**，不属于 HAL 层 (V0.3.1 修正)
- 实际代码位置: `rm_sensor_module/sync/` (Module Library 层)
- ISyncCoordinator 依赖 ICameraHAL 的帧回调 + 时间戳，但不侵入 HAL 接口
- 单厂商 GMSL 场景可**直接走 `/dev/camsync` 硬件触发**，无需 ISyncCoordinator
- 跨厂商/跨品类同步 (Camera + LiDAR + IMU) 应使用 ArcRT Channel 层的 `message_filter`
  (参见 §7.2 ArcRT Channel 层需求)

---

### §2.11 ISyncManager — 单设备硬件同步接口 (**V0.3.2 新增**)

> **层次归属**: ISyncManager **属于 HAL 层** (单设备内部硬件同步配置)。
> 与 §2.10 的 ISyncCoordinator (Module 层, 跨设备编排) 严格区分:
> - **ISyncManager**: 单台相机内的同步模式配置 (PRIMARY/SECONDARY/SOFTWARE_TRIGGER 等),
>   直接映射 SDK API (`ob::Device::setMultiDeviceSyncConfig()` /
>   `rs2::device::set_options()`), 属于 HAL。
> - **ISyncCoordinator**: 编排多台相机互相等待的业务逻辑, 属于 Module 层。
>
> 通过 `ICameraHAL::getSyncManager()` 访问; 不支持同步的设备返回 `nullptr`。

#### 2.11.1 SyncMode 枚举 (来自 Orbbec 7 种模式)

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/sync_manager.hpp
namespace rm::hal::sensor {

/// 硬件同步模式 (V0.3.2 新增)
/// 对标 Orbbec OBMultiDeviceSyncMode; RealSense 有 inter_cam_sync_mode 类似概念
enum class SyncMode : uint8_t {
        FreeRun,          // 自由运行 (默认, 无外部触发)
        Standalone,       // 独立模式 (忽略外部触发信号, 不向外输出)
        Primary,          // 主设备: 产生触发信号, 供从设备使用
        Secondary,        // 从设备: 被动接收触发, 可能有掉帧
        SecondarySynced,  // 从设备 (同步保证版): 接收触发后保证同步输出
        SoftwareTrigger,  // 软件触发: 由代码调用 triggerOnce() 触发单次拍摄
        HardwareTrigger,  // 硬件触发: 外部 GPIO/GMSL 信号 (与 /dev/camsync 配合)
};

/// 硬件同步精细配置参数 (V0.3.2 新增)
/// 对标 Orbbec 的细粒度时序参数; RealSense 仅部分支持
struct SyncConfig {
        SyncMode mode                = SyncMode::FreeRun;
        int depth_delay_us           = 0;   // 深度帧触发延迟 (μs)
        int color_delay_us           = 0;   // 彩色帧触发延迟 (μs)
        int trigger2image_delay_us   = 0;   // 触发信号到出图延迟 (μs)
        int trigger_out_delay_us     = 0;   // 触发输出引脚延迟 (μs, Primary 模式)
        bool trigger_out_enabled     = false; // 是否向外输出触发信号 (Primary 模式)
        int frames_per_trigger       = 1;   // 每次触发产生多少帧 (通常 1)
};

/// 单设备硬件同步管理器接口 (V0.3.2 新增)
/// 由 ICameraHAL::getSyncManager() 获取
class ISyncManager {
public:
        virtual ~ISyncManager() = default;

        /// 获取设备支持的同步模式列表
        virtual std::vector<SyncMode> getSupportedSyncModes() const = 0;

        /// 设置同步配置 (运行时可调, 部分设备需重启流)
        virtual bool setSyncConfig(const SyncConfig& config) = 0;

        /// 读取当前同步配置
        virtual SyncConfig getSyncConfig() const = 0;

        /// 软件触发一次 (仅 SyncMode::SoftwareTrigger 时有效)
        /// 对应 Orbbec /camera/trigger_capture Service 的 HAL 侧实现
        virtual bool triggerOnce() = 0;

        /// 查询当前是否已完成同步 (Slave 模式下: 有没有收到主设备触发)
        virtual bool isSynced() const = 0;
};

} // namespace rm::hal::sensor
```

#### 2.11.2 与 CameraConfig::sync_mode 的关系

V0.3 的 `CameraConfig.sync_mode` 是字符串 (`"free_run"` / `"primary"` / `"secondary"` /
`"hardware_triggering"`)。V0.3.2 推荐:
- **启动时**: 在 `configure()` 中将字符串映射为 `SyncMode` 枚举并调用 `getSyncManager()->setSyncConfig()`
- **运行时**: 通过 `getSyncManager()->setSyncConfig()` 动态修改 (如触发延迟调整)
- **软件触发**: Module 层将 ROS Service 调用转发为 `getSyncManager()->triggerOnce()`

#### 2.11.3 OrbbecCameraHAL 实现映射

| ISyncManager 方法 | OrbbecSDK API | 说明 |
|------------------|--------------|------|
| `getSupportedSyncModes()` | `device->isPropertySupported(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG)` | 检查支持, 返回全部 7 种 |
| `setSyncConfig(cfg)` | `device->setStructuredData(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, ...)` | 写入多设备同步配置结构体 |
| `getSyncConfig()` | `device->getStructuredData(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG)` | 读取当前配置 |
| `triggerOnce()` | `device->triggerCapture()` | 软件触发一次 |
| `isSynced()` | 检查帧 frameset `timeStampUs()` 有效且与主设备时间对齐 | HAL 内部实现 |

#### 2.11.4 RMOS 机器人典型配置

```yaml
# Realman 4× GMSL 相机场景
head_depth_cam:
    sync_mode: "primary"          # Primary: 产生触发信号
    trigger_out_enabled: true
    trigger_out_delay_us: 0

chest_depth_cam:
    sync_mode: "secondary"        # Secondary: 接收 GMSL 硬件触发
    depth_delay_us: 0
    trigger2image_delay_us: 0

# 单机软件触发场景 (工业视觉 / 标定)
calibration_cam:
    sync_mode: "software_trigger" # 调用 triggerOnce() 触发
    frames_per_trigger: 1
```

---

## §3 LiDAR HAL 详细设计

> **补全 Notion V0.2 §3 TODO — 首次详细设计**

### §3.1 ILidarHAL 完整接口

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/lidar_hal.hpp
#pragma once
#include "rm_hal_sensor/sensor_hal_base.hpp"
#include <cmath>
#include <vector>
#include <functional>
#include <memory>

namespace rm::hal::sensor {

/// LiDAR 扫描模式
enum class LidarScanMode : int {
    STANDARD = 0,    // 标准扫描
    EXPRESS  = 1,    // 高速扫描 (降精度换帧率)
    BOOST    = 2,    // 增强扫描
};

struct LidarConfig {
    std::string device_id;      // "front_lidar" / "rear_lidar"

    // 连接参数 (BlueSea UDP)
    std::string host;           // LiDAR IP 地址
    int port = 6543;            // UDP 数据端口

    // 扫描范围
    double angle_min = -M_PI;   // rad
    double angle_max =  M_PI;
    double range_min = 0.1;     // m
    double range_max = 30.0;

    // 扫描参数
    int scan_frequency_hz = 10; // 期望扫描频率（部分 LiDAR 可调）
    LidarScanMode scan_mode = LidarScanMode::STANDARD;

    // 滤波
    bool enable_intensity_filter = false;
    float min_intensity = 0.0f;  // 低于此强度的点被过滤

    // ==== V0.3 新增: 协议参数 (来自 bluesea-ladar 存量) ====
    bool   raw_bytes = true;         // 使用原始字节解析
    bool   with_checksum = false;    // 启用 CRC32 校验 (STM32 变体)
    bool   with_intensity = false;   // 数据带强度值
    bool   output_360 = true;        // 拼装 360° 后输出 (否则按扇形)

    int    angle_resolution = 100;   // 角度分辨率 (0.01° 单位), 100=1°
    int    rpm = 600;                // 电机转速 (RPM)

    // 网络参数
    int    recv_buf_size = 1024 * 1024;  // UDP 接收缓冲区 (1MB)
    int    udp_timeout_ms = 5000;        // 接收超时

    // 去拖影 / 角度掩码
    int    mask = 0;                 // 角度掩码 bitmask (扇区屏蔽)
    int    error_circle = 3;         // 容错圈数
    bool   with_deshadow = false;    // 去拖影滤波
};

struct LaserScanData {
    uint64_t timestamp_ns = 0;  // steady_clock, 整圈扫描起始时间

    // 扫描几何
    double angle_min = 0;       // rad
    double angle_max = 0;
    double angle_increment = 0; // rad/点
    double time_increment = 0;  // 相邻点时间差 (s)
    double scan_time = 0;       // 整圈时间 (s)

    // 量程
    double range_min = 0;
    double range_max = 0;

    // 数据
    std::vector<float> ranges;       // m, inf = 无效
    std::vector<float> intensities;  // 可选, 反射强度
};

class ILidarHAL : public ISensorHAL {
public:
    virtual bool configure(const LidarConfig& config) = 0;

    /// 轮询获取最新一圈扫描数据
    virtual bool getScan(LaserScanData& out) = 0;

    /// 回调模式: 每完成一圈触发
    using ScanCallback = std::function<void(std::shared_ptr<const LaserScanData>)>;
    virtual void setScanCallback(ScanCallback cb) = 0;

    /// 运行时调整扫描频率 (如果设备支持)
    virtual bool setScanFrequency(int hz) { return false; }
};

} // namespace rm::hal::sensor
```

### §3.2 LiDAR 状态机

```
Closed ──configure()──→ Configured ──open()──→ Opened ──startReceiving*──→ Streaming
  ↑                                                                          │
  └──────────────────────close()─────────────────────────────────────────────┘
                                        任意状态 ──fault──→ Faulted ──reset()──→ Closed
```

*注: LiDAR 无独立的 `startStreaming()`，`open()` 成功后 UDP 数据即开始到达。Streaming 态由内部从首帧到达时自动切换。*

**状态转移表:**

| 当前状态 | 事件 | 下一状态 | 动作 |
|---------|------|---------|------|
| Closed | configure() | Configured | 保存配置 |
| Configured | open() | Opened | 创建 UDP socket，绑定端口 |
| Opened | 首帧到达 | Streaming | 更新 alive=true |
| Streaming | close() | Closed | 关闭 socket，释放缓冲区 |
| Streaming | UDP 超时 3s | Faulted | alive=false |
| Faulted | reset() | Closed | close() + 清理 |

### §3.3 线程模型

```
UDP 接收线程 (select/epoll) ──解析协议──→ [LaserScanData ring buffer]
                                              │
                                              ├── getScan() 轮询：取最新
                                              └── ScanCallback：通知用户
```

- **UDP 接收线程**: HAL 内部创建，`open()` 启动，`close()` 终止
- **协议解析**: BlueSea 私有 UDP 协议 → 角度+距离+强度 → LaserScanData
- **Ring buffer**: 深度 2（LiDAR 帧率低，无需太深）

### §3.4 BlueSea LiDAR Adapter→协议映射 (**V0.3 大幅扩展**)

| ILidarHAL 方法 | BlueSea 协议操作 | 说明 |
|----------------|-----------------|------|
| `configure()` | 保存 host:port + 扫描参数 | |
| `open()` | `socket(AF_INET, SOCK_DGRAM)` + `bind()` + 启动接收线程 | |
| `close()` | 关闭 socket + join 接收线程 | |
| `getScan()` | 从 ring buffer 取最新 LaserScanData | FanAssembler 聚合后 |
| `health()` | alive = 最近 3s 内有完整扫描 | data_rate_hz = 实际扫描频率 |

#### 3.4.1 BlueSea UDP 六种帧头格式 (V0.3 新增)

BlueSea LiDAR 存在 **6 种帧头格式**，对应不同硬件版本和数据类型：

##### HDR — 基础格式 (18B, magic = "LSXX")

```c
struct HdrInfo {
    unsigned char  header[6];     // "LSXX" 前缀
    unsigned short angle;          // 起始角度 (0.01°单位)
    unsigned short span;           // 角度范围
    unsigned short fbase;          // 基础距离偏移
    unsigned short first;          // 第一个数据点索引
    unsigned short last;           // 最后一个数据点索引
    unsigned short fend;           // 保留
};  // sizeof = 18
```

##### HDR2 — 扩展头 (24B, 含 CRC + 时间戳)

```c
struct HdrInfo2 {
    // HDR 基本字段 (18B)
    unsigned char  header[6];
    unsigned short angle, span, fbase, first, last, fend;
    // 扩展字段
    unsigned short CRC;            // CRC 校验 (由 STM32 CRC32 截短)
    unsigned short timestamp_lo;   // 时间戳低16位
    unsigned short timestamp_hi;   // 时间戳高16位
};  // sizeof = 24
```

##### HDR3 — 含圈号 (26B)

```c
struct HdrInfo3 {
    HdrInfo2       base;           // 24B
    uint16_t       circle_no;     // 当前扫描圈序号
};  // sizeof = 26
```

##### HDR7 — 含强度标记 (28B)

```c
struct HdrInfo7 {
    // HDR2 字段 (24B)
    unsigned char  header[6];
    unsigned short angle, span, fbase, first, last, fend;
    unsigned short CRC;
    unsigned short timestamp_lo, timestamp_hi;
    // 扩展
    unsigned short circle_no;
    unsigned char  with_intensity; // 1=每点附加强度字节
    unsigned char  reserved;
};  // sizeof = 28
// 数据区: with_intensity=1 每点=[distance(2B)+intensity(1B)]=3B; 否=2B
```

##### HDRAA / HDR99 — 特殊标记帧 (少见)

```c
// 0xAA 或 0x99 作为首字节标识, 后续字节依固件版本而定
// HAL 应能识别并跳过
```

##### 帧头版本判别

```cpp
int identifyHeaderSize(const uint8_t* buf, bool with_checksum, bool with_intensity) {
    if (buf[0] == 0xAA || buf[0] == 0x99) return -1; // 特殊帧, 跳过
    // "LS" prefix check
    if (!(buf[0]=='L' && buf[1]=='S')) return -1;
    // 按配置判定:
    if (!with_checksum) return 18;   // HDR
    if (!with_intensity) return 24;  // HDR2 (可能 26=HDR3)
    return 28;                        // HDR7
}
```

#### 3.4.2 数据点结构映射

```c
// BlueSea 原始数据点
struct DataPoint {
    uint16_t distance;   // 单位 mm (或 0.25mm，取决于 fbase)
    uint8_t  intensity;  // 0~255, 仅 with_intensity=1 时有效
};
```

**LaserScanData 字段映射:**

| LaserScanData 字段 | 源值 | 计算方式 |
|-------------------|------|---------|
| `angle_min` | HDR.angle | `angle * 0.01 * π / 180` |
| `angle_max` | HDR.angle + HDR.span | `(angle + span) * 0.01 * π / 180` |
| `angle_increment` | N 点均分 span | `span * 0.01 * π / 180 / N` |
| `ranges[i]` | DataPoint.distance | `distance * 0.001` (mm→m) |
| `intensities[i]` | DataPoint.intensity | `intensity / 255.0f` |
| `timestamp_ns` | HDR2.timestamp_lo/hi | `((hi << 16) \| lo) * 1000` (μs→ns) |

#### 3.4.3 FanAssembler — 360° 扇形聚合 (V0.3 新增)

每个 UDP 包仅覆盖部分角度 (扇形)。需要 `FanAssembler` 拼装完整 360° 扫描：

```
UDP 包 (30°~45°)  ──┐
UDP 包 (45°~60°)  ──┤──→ FanAssembler
UDP 包 (60°~75°)  ──┤      ↓ span 累计 ≥ 360°
...                 ──┤      ↓
UDP 包 (15°~30°)  ──┘  → 发布一帧完整 LaserScanData
```

```cpp
class FanAssembler {
    int total_points_;             // 一圈总点数 (= 36000 / angle_resolution)
    float resolution_deg_;         // 角度分辨率 (°)
    std::vector<float> ranges_;
    std::vector<float> intensities_;
    int total_span_ = 0;          // 累计角度 (0.01° 单位)

public:
    /// 每收到一个 UDP 包调用。返回 true = 一帧就绪
    bool addFan(const HdrInfo& hdr, const DataPoint* pts, int count) {
        int start_idx = hdr.angle / (resolution_deg_ * 100);
        for (int i = 0; i < count; i++) {
            int idx = (start_idx + i) % total_points_;
            ranges_[idx] = pts[i].distance * 0.001f;
            if (has_intensity_)
                intensities_[idx] = pts[i].intensity / 255.0f;
        }
        total_span_ += hdr.span;
        if (total_span_ >= 36000) {  // ≥360.00°
            total_span_ = 0;
            return true;             // 发布
        }
        return false;
    }
    LaserScanData popScan();        // 构建 LaserScanData + reset buffers
};
```

#### 3.4.4 CRC32 校验 (STM32 变体) (V0.3 勘误+新增)

> **V0.3 勘误**: 原文档写"CRC16 校验"为 **错误**。BlueSea 实际使用 **STM32 CRC32**。

```cpp
// 多项式 0x04C11DB7, 无反转 (与 zlib CRC32 不同!)
uint32_t stm32_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    size_t words = len / 4;
    for (size_t i = 0; i < words; i++) {
        uint32_t w = (data[i*4]<<24) | (data[i*4+1]<<16) |
                     (data[i*4+2]<<8) | data[i*4+3];
        crc ^= w;
        for (int j = 0; j < 32; j++)
            crc = (crc & 0x80000000) ? (crc<<1) ^ 0x04C11DB7 : crc<<1;
    }
    return crc;
}
// 校验流程: UDP 接收 → 解析帧头 → CRC32 比对 → 通过则解析数据点, 否则丢弃+计数
```

#### 3.4.5 硬件型号配置预设

| 型号 | range_max | with_intensity | rpm | angle_resolution | 说明 |
|------|-----------|---------------|-----|-----------------|------|
| LDS-U50C-S | 50m | true | 600 | 36 (0.36°) | 标准 |
| LDS-U80C-S | 80m | true | 1200 | 18 (0.18°) | 高精度 |

### §3.5 CMake target

```
src/hal/rm_hal_sensor/common/lidar/bluesea/
├── CMakeLists.txt          # target: rm_hal_sensor_bluesea
├── bluesea_lidar_hal.hpp
├── bluesea_lidar_hal.cpp
└── bluesea_protocol.hpp    # BlueSea UDP 协议解析
```

### §3.6 错误场景

| 场景 | HAL 行为 |
|------|---------|
| UDP 端口被占用 | open() 返回 false + IO_ERROR |
| 网络不通 | 持续无数据 → alive=false |
| 数据包 CRC 失败 | 丢弃该包，error_msg 计数 |
| 扫描频率异常 | health().data_rate_hz 偏差 >50% 时 WARN |

### §3.7 单测要点

```cpp
TEST(LidarHALContract, LifecycleHappyPath) { /* sim 模式 */ }
TEST(LidarHALContract, ScanDataValid) {
    // getScan 返回数据: ranges.size() > 0, angle_min < angle_max
}
TEST(LidarHALContract, CallbackReceivesData) { /* 类似 Camera */ }
```

---

## §4 IMU HAL 详细设计

> **补全 Notion V0.2 §4 TODO — 首次详细设计**

### §4.1 IImuHAL 完整接口

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/imu_hal.hpp
#pragma once
#include "rm_hal_sensor/sensor_hal_base.hpp"
#include <functional>
#include <memory>
#include <array>

namespace rm::hal::sensor {

/// IMU 加速度计量程
enum class AccelRange : int {
    G2  = 2,    // ±2g
    G4  = 4,    // ±4g
    G8  = 8,    // ±8g
    G16 = 16,   // ±16g
};

/// IMU 陀螺仪量程
enum class GyroRange : int {
    DPS250  = 250,    // ±250 °/s
    DPS500  = 500,
    DPS1000 = 1000,
    DPS2000 = 2000,
};

/// IMU 融合策略（多源数据对齐方式）
enum class ImuFusionMode : int {
    NONE          = 0,  // 不融合，accel/gyro 独立输出
    COPY          = 1,  // 直接复制最近值（适合硬件已同步）
    INTERPOLATION = 2,  // 线性插值对齐时间戳（推荐）
};

struct ImuConfig {
    std::string device_id;          // "body_imu" / "chassis_imu"

    // 连接参数 (YESENSE 串口)
    std::string port;               // "/dev/yesenseIMU"
    int baudrate = 460800;

    // 采样参数
    int output_rate_hz = 200;       // 输出频率
    AccelRange accel_range = AccelRange::G8;
    GyroRange gyro_range = GyroRange::DPS2000;

    // 融合策略
    ImuFusionMode fusion_mode = ImuFusionMode::INTERPOLATION;

    // 校正开关
    bool enable_accel_correction = true;    // 加速度计数据校正
    bool enable_gyro_correction = true;     // 陀螺仪数据校正
    bool enable_mag_correction = false;     // 磁力计校正（如有）

    // 噪声参数（用于上层滤波器配置，如 EKF）
    double accel_noise_density = 0.0001;    // m/s²/√Hz
    double gyro_noise_density = 0.0001;     // rad/s/√Hz
    double accel_random_walk = 0.0001;      // m/s³/√Hz
    double gyro_random_walk = 0.0001;       // rad/s²/√Hz
};

struct ImuData {
    uint64_t timestamp_ns = 0;      // steady_clock

    // 加速度 (m/s²)
    double accel_x = 0, accel_y = 0, accel_z = 0;

    // 角速度 (rad/s)
    double gyro_x = 0, gyro_y = 0, gyro_z = 0;

    // 姿态四元数（如果设备 AHRS 输出）
    double quat_w = 1, quat_x = 0, quat_y = 0, quat_z = 0;
    bool has_orientation = false;

    // 磁力计 (μT，可选)
    double mag_x = 0, mag_y = 0, mag_z = 0;
    bool has_magnetometer = false;

    // 温度 (℃，可选)
    double temperature = 0;
    bool has_temperature = false;

    // 数据质量标志
    bool is_calibrated = true;      // false = 原始未校正数据
    uint32_t sequence = 0;          // 单调递增序列号
};

/// IMU 设备信息（用于上层配置滤波器参数）
struct ImuDeviceInfo {
    AccelRange accel_range;
    GyroRange gyro_range;
    double accel_noise_density;
    double gyro_noise_density;
    double accel_random_walk;
    double gyro_random_walk;
    double reference_temperature;   // 校准参考温度
};

class IImuHAL : public ISensorHAL {
public:
    virtual bool configure(const ImuConfig& config) = 0;

    /// 轮询获取最新 IMU 数据
    virtual bool getData(ImuData& out) = 0;

    /// 回调模式: 每次新数据到达时触发
    using ImuCallback = std::function<void(const ImuData&)>;  // V0.3 修正: const ref 替代 shared_ptr
    virtual void setDataCallback(ImuCallback cb) = 0;

    /// 获取设备噪声/校准信息
    virtual ImuDeviceInfo getDeviceInfo() const = 0;

    /// 重置姿态估计（如果设备支持 AHRS reset）
    virtual bool resetOrientation() { return false; }
};

} // namespace rm::hal::sensor
```

### §4.2 IMU 状态机

```
Closed ──configure()──→ Configured ──open()──→ Opened(=Streaming)
  ↑                                                    │
  └──────────────────────close()───────────────────────┘
                                任意状态 ──fault──→ Faulted ──reset()──→ Closed
```

*注: IMU 设备无独立 start/stop，open() 后串口即持续输出数据。*

### §4.3 线程模型

```
串口读取线程 (read + 协议解析) ──→ [ImuData ring buffer (depth=4)]
                                        │
                                        ├── getData() 轮询
                                        └── ImuCallback 通知
```

- **串口读取线程**: 以 `output_rate_hz` 频率 (200Hz) 读取
- **协议解析**: YESENSE 私有串口协议 → 二进制解帧 → ImuData
- **融合处理**: 如果 `fusion_mode == INTERPOLATION`，在解析后对 accel/gyro 做线性插值时间对齐

### §4.4 YESENSE IMU Adapter→串口协议映射 (**V0.3 大幅扩展**)

| IImuHAL 方法 | YESENSE 串口操作 | 说明 |
|-------------|-----------------|------|
| `configure()` | 保存串口参数 + 采样率 | |
| `open()` | `::open(port, O_RDWR)` + termios 460800/8N1 + 启动读取线程 | |
| `close()` | 关闭 fd + join 读取线程 | |
| `getData()` | ring buffer 取最新 | |
| `health()` | alive = 最近 1s 内有数据 | data_rate_hz = 实际输出频率 |
| `getDeviceInfo()` | 返回 configure 时设置的噪声参数 | |

#### 4.4.1 YESENSE TLV 帧结构 (V0.3 新增)

```
┌──────────┬──────────┬────────┬────────┬─────────┬─────────┬────────┐
│ HEADER_L │ HEADER_H │ TID_L  │ TID_H  │ LENGTH  │ PAYLOAD │  CRC   │
│  0x59    │  0x53    │ 2 Byte │        │ 1 Byte  │ N Byte  │ 2 Byte │
└──────────┴──────────┴────────┴────────┴─────────┴─────────┴────────┘
```
- **HEADER**: 固定 0x59 0x53 ("YS")
- **TID**: Transaction ID, 2 字节 LE
- **LENGTH**: Payload 长度 (0~255)
- **PAYLOAD**: 多个 TLV 三元组 `[DataID(1B) + Len(1B) + Value(Len B)]` 顺序排列
- **CRC**: 2 字节 Fletcher checksum (ck1, ck2)
- **总帧长**: N + 7 字节

#### 4.4.2 DataID 与缩放因子 (V0.3 新增, 关键!)

| DataID | 名称 | 字节 | 类型 | 缩放因子 | 输出单位 | HAL 目标 (SI) |
|--------|------|------|------|---------|---------|-------------|
| 0x01 | CHIP_TEMPERATURE | 2 | int16 | ×0.01 | °C | °C |
| 0x10 | ACCEL_RAW | 12 | int32×3 | ×0.000001 | g | → m/s² (×9.80665) |
| 0x11 | LINEAR_ACCEL | 12 | int32×3 | ×0.000001 | g | → m/s² (×9.80665) |
| 0x20 | ANGULAR_VELOCITY | 12 | int32×3 | ×0.000001 | dps | → rad/s (×π/180) |
| 0x30 | MAG_FIELD | 12 | int32×3 | ×0.001 | μT | → T (×1e-6) |
| 0x40 | EULER_ANGLES | 12 | int32×3 | ×0.000001 | deg | → rad (×π/180) |
| 0x41 | QUATERNION | 16 | int32×4 | ×0.000001 | 无量纲 | 无量纲 |
| 0x50 | UTC_TIME | 11 | struct | 见下方 | — | — |
| 0x60 | LOCATION | 12 | int32×3 | 经纬×1e-7, 高度×0.001 | deg, m | deg, m |
| 0x61 | SPEED_OVER_GROUND | 12 | int32×3 | ×0.001 | m/s | m/s |
| 0x70 | STATUS_WORD | 2 | uint16 | 位域 | — | — |
| 0x80 | SAMPLE_TIMESTAMP | 4 | uint32 | ×1 | ms | ms |

**缩放常量 (HAL 内部使用):**

```cpp
constexpr double ACCEL_SCALE     = 0.000001;    // int32 × 0.000001 = g
constexpr double GYRO_SCALE      = 0.000001;    // int32 × 0.000001 = dps
constexpr double EULER_SCALE     = 0.000001;    // int32 × 0.000001 = degree
constexpr double QUAT_SCALE      = 0.000001;    // int32 × 0.000001
constexpr double MAG_SCALE       = 0.001;       // int32 × 0.001 = μT
constexpr double TEMP_SCALE      = 0.01;        // int16 × 0.01 = °C

// g → m/s², dps → rad/s
constexpr double G_TO_MS2        = 9.80665;
constexpr double DPS_TO_RADS     = M_PI / 180.0;
```

#### 4.4.3 Fletcher Checksum (V0.3 勘误+新增)

> **V0.3 勘误**: 原文档写"CRC"不够精确。YESENSE 使用 **Fletcher checksum**。

```cpp
void fletcher_checksum(const uint8_t* data, size_t len,
                       uint8_t& ck1, uint8_t& ck2) {
    ck1 = 0;  ck2 = 0;
    for (size_t i = 0; i < len; i++) {
        ck1 += data[i];
        ck2 += ck1;
    }  // uint8_t 自然溢出截断
}
// 校验范围: TID_L 到 PAYLOAD 末尾 (不含 HEADER 和 CRC 本身)
```

#### 4.4.4 帧同步状态机 (V0.3 新增)

```
SEEK_H1 ──0x59──→ SEEK_H2 ──0x53──→ READ_TID(2B) → READ_LEN(1B)
   ↑ 非0x59          ↑ 非0x53                         │
   └──────────────────┘                               ↓
                                   READ_PAYLOAD(LEN B) → READ_CRC(2B)
                                                          │
                                              CRC OK → DISPATCH (解析 TLV)
                                              CRC FAIL → 回退 SEEK_H1
```

#### 4.4.5 ImuData 字段补全 (V0.3 新增)

```cpp
struct ImuData {
    // ==== 已有 ====
    uint64_t timestamp_ns;
    double accel_x, accel_y, accel_z;     // m/s²
    double gyro_x, gyro_y, gyro_z;        // rad/s
    double quat_w, quat_x, quat_y, quat_z;
    bool has_orientation;
    double mag_x, mag_y, mag_z;
    bool has_magnetometer;
    double temperature;
    bool has_temperature;
    bool is_calibrated;
    uint32_t sequence;

    // ==== V0.3 新增 ====
    double linear_accel_x, linear_accel_y, linear_accel_z;  // 去重力 (DataID 0x11)
    bool has_linear_accel = false;
    double euler_roll, euler_pitch, euler_yaw;  // rad (DataID 0x40, 已转换)
    bool has_euler = false;
    uint16_t status_word = 0;                   // DataID 0x70
    uint32_t sample_timestamp_ms = 0;           // DataID 0x80
};
```

### §4.5 多 IMU 实例管理

系统中存在两个 IMU:
- `body_imu`: 安装在躯干，用于姿态估计
- `chassis_imu`: 安装在底盘，用于里程计融合

**管理方式:**
- 工厂函数创建两个独立的 IImuHAL 实例
- 各实例独占自己的串口 (`/dev/yesenseIMU_body` / `/dev/yesenseIMU_chassis`)
- 通过 `device_id` 区分

```yaml
# profiles/real_full.yaml
sensors:
    body_imu:    { type: yesense, port: /dev/yesenseIMU_body }
    chassis_imu: { type: yesense, port: /dev/yesenseIMU_chassis }
```

### §4.6 错误场景

| 场景 | HAL 行为 |
|------|---------|
| 串口不存在 | open() 返回 false + DEVICE_NOT_FOUND |
| 串口权限不足 | open() 返回 false + PERMISSION_DENIED |
| 数据超时 | alive=false, error_msg="no data for 1s" |
| CRC 校验失败 | 丢弃该帧，统计错误率 |
| 波特率不匹配 | 持续 CRC 失败 → 自动尝试其他波特率（可选） |

### §4.7 单测要点

```cpp
TEST(ImuHALContract, LifecycleHappyPath) { /* sim 模式 */ }
TEST(ImuHALContract, DataValid) {
    // getData: accel 在合理范围 (±16g), gyro 在合理范围 (±2000°/s)
}
TEST(ImuHALContract, DataRate) {
    // 200Hz ± 10% 容差
}
TEST(ImuHALContract, FusionInterpolation) {
    // 验证融合模式下 accel/gyro 时间戳一致
}
TEST(ImuHALContract, MultiInstance) {
    // 同时创建 body_imu + chassis_imu，互不干扰
}
```

---

## §5 Audio HAL 详细设计 (**V0.3 从骨架升级为完整设计**)

> 原 V0.2 仅预留骨架。V0.3 补全完整接口、数据结构、ALSA 映射、DOA 及线程模型。
> 存量代码: `wheeltec_mic` (ReSpeaker 麦克风阵列) + `audio_encode` (ALSA 采集)

### §5.1 IAudioHAL 完整接口

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/audio_hal.hpp
#pragma once
#include "rm_hal_sensor/sensor_hal_base.hpp"
#include <functional>
#include <optional>
#include <vector>
#include <cstdint>

namespace rm::hal::sensor {

/// PCM 采样格式
enum class AudioSampleFormat : uint8_t {
    S16_LE = 0,   // 16-bit signed LE (最常用, 语音识别)
    S24_LE,       // 24-bit signed
    S32_LE,       // 32-bit signed
    F32_LE,       // 32-bit float
};

/// 音频帧 — 一个 period 内的 PCM 采样
struct AudioFrame {
    const int16_t* data;          // PCM 数据 (interleaved channels)
    size_t         sample_count;  // 总采样 = frame_count × channels
    size_t         frame_count;   // period 内帧数
    uint8_t        channels;
    uint32_t       sample_rate;
    AudioSampleFormat format;
    uint64_t       timestamp_ns;
    uint32_t       sequence;
};

/// DOA (Direction of Arrival) 声源定位结果
struct DOAResult {
    float  azimuth_deg;           // 水平方位角 (°), 0=正前方, 顺时针正
    float  elevation_deg;         // 仰角 (°), 0=水平
    float  confidence;            // 置信度 [0, 1]
    uint64_t timestamp_ns;
};

/// 音频配置
struct AudioConfig {
    std::string device_type;      // "respeaker_4mic" / "respeaker_6mic" / "alsa_generic"
    std::string device_name;      // ALSA 设备名 "plughw:2,0" / "default"
    uint32_t    sample_rate = 16000;  // Hz (语音: 16000, 音乐: 48000)
    uint8_t     channels = 4;
    AudioSampleFormat format = AudioSampleFormat::S16_LE;
    size_t      period_frames = 1024; // ALSA period
    size_t      buffer_frames = 4096; // ALSA buffer
    bool        enable_doa = true;
};

using AudioCallback = std::function<void(const AudioFrame&)>;
using DOACallback   = std::function<void(const DOAResult&)>;

class IAudioHAL : public ISensorHAL {
public:
    ~IAudioHAL() override = default;

    virtual bool configure(const AudioConfig& config) = 0;

    /// 开始采集 (ALSA 读取线程启动)
    virtual bool startCapture() = 0;
    virtual void stopCapture() = 0;

    /// 注册回调
    virtual void setAudioCallback(AudioCallback cb) = 0;
    virtual void setDOACallback(DOACallback cb) = 0;

    /// 同步获取最近 DOA
    virtual std::optional<DOAResult> getLatestDOA() const = 0;
};

} // namespace rm::hal::sensor
```

### §5.2 Audio 状态机

```
CREATED ──configure()──→ CONFIGURED ──open()──→ READY
                                                    ↓
                                startCapture() → CAPTURING
                                                    ↓ ↑
                                 ← stopCapture() ←
                                                    ↓
                                    close() → CLOSED

CAPTURING 内部子状态:
  NORMAL   — 正常采集
  OVERRUN  — ALSA buffer overrun (自动恢复)
  ERROR    — 无法恢复
```

### §5.3 ALSA PCM 映射

| AudioConfig 字段 | ALSA API | 说明 |
|-----------------|----------|------|
| `device_name` | `snd_pcm_open(&handle, name, SND_PCM_STREAM_CAPTURE, 0)` | |
| `sample_rate` | `snd_pcm_hw_params_set_rate_near()` | 可能被修正 |
| `channels` | `snd_pcm_hw_params_set_channels()` | |
| `format` | `snd_pcm_hw_params_set_format()` | |
| `period_frames` | `snd_pcm_hw_params_set_period_size_near()` | |
| `buffer_frames` | `snd_pcm_hw_params_set_buffer_size_near()` | |

**采样格式映射:**

| AudioSampleFormat | ALSA 常量 | 字节/样本 |
|-------------------|----------|----------|
| S16_LE | `SND_PCM_FORMAT_S16_LE` | 2 |
| S24_LE | `SND_PCM_FORMAT_S24_LE` | 3 |
| S32_LE | `SND_PCM_FORMAT_S32_LE` | 4 |
| F32_LE | `SND_PCM_FORMAT_FLOAT_LE` | 4 |

### §5.4 线程模型

```
┌────────────────────────┐
│   ALSA 采集线程         │ ← snd_pcm_readi() 阻塞读取
│   (captureLoop)        │    → AudioFrame → audio_cb_
│                        │    → 若 enable_doa → DOA 估计 → doa_cb_
└────────────────────────┘

多通道 interleaved 排列:
  [ch0_s0, ch1_s0, ch2_s0, ch3_s0, ch0_s1, ch1_s1, ...]
```

**ALSA 采集循环核心:**

```cpp
void captureLoop() {
    std::vector<int16_t> buf(config_.period_frames * config_.channels);
    while (running_) {
        auto frames = snd_pcm_readi(pcm_, buf.data(), config_.period_frames);
        if (frames < 0) {
            frames = snd_pcm_recover(pcm_, frames, 0);  // EPIPE=overrun 自动恢复
            if (frames < 0) { reportError(); break; }
            continue;
        }
        AudioFrame af{buf.data(), size_t(frames)*config_.channels,
                      size_t(frames), config_.channels,
                      config_.sample_rate, config_.format, now_ns(), seq_++};
        if (audio_cb_) audio_cb_(af);
    }
}
```

### §5.5 ReSpeaker DOA 声源定位

ReSpeaker 4-Mic / 6-Mic 通过 **USB HID** 暴露 DOA:

| HID Register | 名称 | 说明 |
|-------------|------|------|
| 21 | DOAANGLE | 方向角 0~359° |
| 19 | SPEECHDETECTED | 语音检测 0/1 |
| 20 | VOICEACTIVITY | VAD 置信度 |

```
ReSpeaker DOA (0~359°, 正北=0 顺时针)
    → HAL DOAResult.azimuth_deg (安装偏移校正后)
```

### §5.6 错误场景

| 场景 | ALSA 错误 | HAL 行为 |
|------|----------|---------|
| Buffer overrun | -EPIPE | `snd_pcm_prepare()` 恢复, 计数器++ |
| 设备断开 | -ENODEV | 状态→Faulted |
| 格式不支持 | configure 失败 | INVALID_CONFIG |
| 权限不足 | -EACCES | PERMISSION_DENIED |

### §5.7 CMake target

```cmake
# rm_hal_sensor/common/audio/CMakeLists.txt
find_package(ALSA REQUIRED)

add_library(rm_hal_sensor_audio STATIC
    alsa_audio_hal.cpp
    respeaker_audio_hal.cpp
    sim_audio_hal.cpp
)
target_link_libraries(rm_hal_sensor_audio
    PUBLIC  rm_hal_sensor_interface
    PRIVATE ALSA::ALSA
)

# ReSpeaker HID (可选)
find_package(PkgConfig)
pkg_check_modules(HIDAPI hidapi-libusb)
if(HIDAPI_FOUND)
    target_sources(rm_hal_sensor_audio PRIVATE respeaker_hid.cpp)
    target_link_libraries(rm_hal_sensor_audio PRIVATE ${HIDAPI_LIBRARIES})
    target_compile_definitions(rm_hal_sensor_audio PRIVATE HAS_HIDAPI)
endif()
```

### §5.8 单测要点

```cpp
TEST(AudioHALContract, LifecycleHappyPath) { /* SimAudioHAL */ }
TEST(AudioHALContract, CaptureCallbackReceivesData) {
    // startCapture → audio_cb_ 被调用, frame_count > 0, channels == 4
}
TEST(AudioHALContract, DOACallback) {
    // enable_doa=true → doa_cb_ 被调用, azimuth 在 [0,360)
}
```

---

## §6 横切面: 工厂 / Sim / 性能 / Profile / 测试 (**V0.3 新增**)

> 本节整合跨传感器类型的共性设计。

### §6.1 HALFactory 注册机制

**DeviceInfo 与设备发现 (V0.3.1 新增):**

> 参考 realsense-ros 的设备枚举/热插拔机制。设备发现是 HAL 的基础能力，属于 HAL Interface 层。

```cpp
// rm_hal_sensor/interface/include/rm_hal_sensor/device_info.hpp
namespace rm::hal::sensor {

struct DeviceInfo {
    std::string type;           // "orbbec" / "realsense" / "usb_cam" 等 (对应 Factory 注册名)
    std::string serial_number;  // 设备序列号
    std::string name;           // 设备型号名 (如 "Gemini 330", "D435i")
    std::string connection;     // "usb" / "gmsl2" / "ethernet" / "sim"
    std::string port;           // 物理端口标识 ("gmsl2-1", "/dev/video0", ...)
    std::string firmware_version;
};

/// 设备变更回调: added/removed 列表
using DeviceChangedCallback = std::function<void(
    const std::vector<DeviceInfo>& added,
    const std::vector<DeviceInfo>& removed)>;

} // namespace rm::hal::sensor
```

**HALFactory (含设备枚举):**

```cpp
// rm_hal_sensor/common/include/rm_hal_sensor/factory.hpp
namespace rm::hal::sensor {

template<typename Interface>
class HALFactory {
public:
    using Creator = std::function<std::unique_ptr<Interface>()>;
    using Enumerator = std::function<std::vector<DeviceInfo>()>;

    static HALFactory& instance() {
        static HALFactory inst;
        return inst;
    }

    void registerType(const std::string& type_name, Creator creator) {
        creators_[type_name] = std::move(creator);
    }

    /// 注册设备枚举器 (各 Driver 提供静态发现函数)
    void registerEnumerator(const std::string& type_name, Enumerator enumerator) {
        enumerators_[type_name] = std::move(enumerator);
    }

    std::unique_ptr<Interface> create(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        return it != creators_.end() ? it->second() : nullptr;
    }

    /// 枚举所有已注册类型的可用设备 (V0.3.1 新增)
    std::vector<DeviceInfo> enumerateDevices() const {
        std::vector<DeviceInfo> all;
        for (const auto& [name, enumer] : enumerators_) {
            auto devs = enumer();
            all.insert(all.end(), devs.begin(), devs.end());
        }
        return all;
    }

    /// 设置热插拔回调 (由后台监视线程驱动)
    void setDeviceChangedCallback(DeviceChangedCallback cb) {
        device_changed_cb_ = std::move(cb);
    }

    /// 启用多进程设备互斥锁 (V0.3.2 新增, 来自 Orbbec orb_device_lock 实践)
    /// 使用 POSIX 共享内存 pthread_mutex (PROCESS_SHARED) 防止多进程同时抢占同一设备
    /// 典型场景: ROS2 组合容器内多个 HAL 实例 + 外部调试工具同时运行
    void enableProcessLock(const std::string& lock_name = "rmos_camera_hal") {
        // 内部实现: shm_open(lock_name) → mmap → pthread_mutexattr_setpshared(SHARED)
        process_lock_name_    = lock_name;
        process_lock_enabled_ = true;
    }

private:
    HALFactory() = default;
    std::unordered_map<std::string, Creator>    creators_;
    std::unordered_map<std::string, Enumerator> enumerators_;
    DeviceChangedCallback device_changed_cb_;
    bool        process_lock_enabled_ = false;  // V0.3.2 新增
    std::string process_lock_name_;             // V0.3.2 新增
};

using CameraFactory = HALFactory<ICameraHAL>;
using LidarFactory  = HALFactory<ILidarHAL>;
using ImuFactory    = HALFactory<IImuHAL>;
using AudioFactory  = HALFactory<IAudioHAL>;

// 便捷注册宏
#define REGISTER_SENSOR_HAL(Factory, name, Impl)                         \
    static bool _reg_##Impl = []() {                                     \
        Factory::instance().registerType(name,                           \
            []() { return std::make_unique<Impl>(); });                  \
        return true;                                                     \
    }();

} // namespace rm::hal::sensor
```

**注册示例:**
```cpp
REGISTER_SENSOR_HAL(CameraFactory, "orbbec",   OrbbecCameraHAL)
REGISTER_SENSOR_HAL(CameraFactory, "usb_cam",  UsbCameraHAL)
REGISTER_SENSOR_HAL(CameraFactory, "sim",       SimCameraHAL)
REGISTER_SENSOR_HAL(LidarFactory,  "bluesea",   BlueseaLidarHAL)
REGISTER_SENSOR_HAL(ImuFactory,    "yesense",   YesenseImuHAL)
REGISTER_SENSOR_HAL(AudioFactory,  "respeaker", RespeakerAudioHAL)
```

### §6.2 SimDriver 统一基类

```cpp
// rm_hal_sensor/sim/sim_base.hpp
class SimSensorBase {
protected:
    std::mt19937 rng_;

    void initRng(const std::map<std::string,std::string>& extra) {
        auto it = extra.find("sim_seed");
        rng_.seed(it != extra.end() ? std::stoul(it->second) : std::random_device{}());
    }

    double gaussianNoise(double sigma) {
        return std::normal_distribution<double>(0.0, sigma)(rng_);
    }

    bool shouldDrop(double rate) {
        return rate > 0 && std::uniform_real_distribution<>(0,1)(rng_) < rate;
    }
};
```

**各 Sim 实现数据特征:**

| 类型 | 频率 | 数据 | 噪声 |
|------|------|------|------|
| SimCamera | 30Hz | BGR8 渐变 + Z16 正弦深度 | 无 |
| SimLidar | 10Hz | 360° 正弦距离 2~8m | σ=0.01m |
| SimImu | 100Hz | 静止 (0,0,g) | accel σ=0.01, gyro σ=0.001 |
| SimAudio | 按配置 | 440Hz 正弦 + 噪声 | σ=100 (S16) |

### §6.3 性能预算

**延迟目标:**

| 接口 | 操作 | P99 延迟 | 吞吐 |
|------|------|---------|------|
| ICameraHAL | configure() | < 500ms | 一次性 |
| ICameraHAL | getColorFrame() | < 33ms | 30fps |
| ICameraHAL | getPointCloud() | < 50ms | 15fps |
| ILidarHAL | getScan() | < 100ms | 10~20Hz |
| IImuHAL | callback | < 1ms | 100~400Hz |
| IAudioHAL | callback | < 64ms | 连续流 |

**内存预算:**

| 传感器 | 单帧 | Ring Buffer | 总计 |
|--------|------|-------------|------|
| Camera Color 1280×720 | 2.76MB | ×4 | 11MB |
| Camera Depth 640×480 | 0.6MB | ×4 | 2.4MB |
| PointCloud XYZRGB | 7.37MB | ×2 | 15MB |
| LiDAR (3600点) | 29KB | ×2 | 58KB |
| IMU | 120B | 实时 | ~0 |
| Audio (1024×4ch×S16) | 8KB | ALSA 管理 | 32KB |

### §6.4 Profile YAML 示例

**real_full.yaml (全传感器):**
```yaml
platform: orin
sensors:
  camera:
    type: "orbbec"
    serial_number: "AY3A12100F7"
    width: 1280
    height: 720
    fps: 30
    enable_color: true
    enable_depth: true
    align_mode: "depth_to_color"
  lidar:
    type: "bluesea"
    ip: "192.168.1.200"
    port: 6543
    with_checksum: true
    with_intensity: true
  imu:
    type: "yesense"
    serial_port: "/dev/ttyUSB0"
    baud_rate: 460800
  audio:
    type: "respeaker"
    device_name: "plughw:2,0"
    channels: 4
    enable_doa: true
```

**sim.yaml (全仿真):**
```yaml
platform: sim
sensors:
  camera: { type: "sim", fps: 30, extra_params: { sim_seed: "42" } }
  lidar:  { type: "sim", extra_params: { sim_seed: "42" } }
  imu:    { type: "sim", sample_rate: 100 }
  audio:  { type: "sim", channels: 4 }
```

### §6.5 测试策略

**三层测试金字塔:**

| 层级 | 范围 | 实现 |
|------|------|------|
| 单元测试 | 协议解析 / CRC / 缩放 / 格式转换 | GTest, CI 必过 |
| 契约测试 | Sim 实现行为验证 / 状态机 / 回调 | GTest + SimHAL, CI 必过 |
| 集成测试 | 真实硬件 | 手动 / nightly |

**关键单元测试清单:**

| 目标 | 用例 |
|------|------|
| PixelEncoding | 枚举唯一, bytesPerPixel, isCompressed |
| Fletcher CRC | 已知输入匹配, 空数据, 单字节 |
| STM32 CRC32 | 已知输入, 与硬件比对 |
| YESENSE TLV 解析 | 正常帧 / CRC 错误 / 不完整 / 连续帧 |
| BlueSea 帧头 | 6 种格式解析, CRC32 校验 |
| IMU 缩放 | DataID→SI 单位 (g→m/s², dps→rad/s) |
| FanAssembler | 单扇 / 多扇 / 跨圈 |
| 工厂注册 | 注册 / 创建 / 未注册返回 nullptr |

---

## §7 上层需求清单 — 基于 realsense-ros 对标分析 (**V0.3.1 新增**)

> **来源**: 对标 realsense-ros 4.57.7 分析后识别的 7 项差距中，3 项经 RMOS V5 架构复核
> 后确认不属于 HAL 层，应在上层实现。本节将这些需求按 RMOS 架构层次归集，为后续
> 开发提供明确依据。
>
> **RMOS V5 层次 (相关部分)**:
> ```
> ┌─ App ──────────────────────────────────────────────────┐
> │ Module Library (rm_sensor_module, rm_vision_module...) │
> │ ArcRT / ArcInfer (Channel, message_filter, Executor)  │
> │ rm_alg_foundation (纯 C++ 算法库)                      │
> │ HAL (本文档) ← 仅此层属于 Sensor HAL 范畴              │
> │ BSP / Hardware                                         │
> └────────────────────────────────────────────────────────┘
> ```

### §7.1 Module Library 层需求 (rm_sensor_module)

Module 层是 HAL 与上层 App 之间的桥梁。它持有 HAL 实例，负责将 HAL 原始数据转换为
rm_interface 消息并发布到 ArcRT Channel。以下功能经架构复核确认属于此层:

#### 7.1.1 ISyncCoordinator — 多相机帧同步编排

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros FrameAggregator + 我方 §2.10 设计 |
| **理由** | 编排多个 ICameraHAL 实例 = 跨设备业务逻辑, 违反 HAL "单设备抽象" |
| **代码位置** | `rm_sensor_module/sync/sync_coordinator.hpp` |
| **依赖 HAL** | ICameraHAL::setFrameSetCallback(), 帧时间戳 |
| **接口草案** | 见 §2.10.3 (已标注为 Module 层) |

#### 7.1.2 Filter Pipeline — 后处理滤波管线

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros 的 `NamedFilter` 管线 (Decimation/Spatial/Temporal/HoleFilling/PointCloud) |
| **理由** | 后处理属于业务逻辑, HAL 只负责 "原始数据采集抽象" |
| **代码位置** | `rm_sensor_module/filter/` 或调用 `rm_alg_foundation` |
| **接口草案** | |

```cpp
// rm_sensor_module/filter/filter_pipeline.hpp
namespace rm::sensor_module {

/// 滤波器基类 (纯 C++, 不依赖中间件)
class IFilter {
public:
    virtual ~IFilter() = default;
    virtual std::string name() const = 0;
    virtual bool process(const ImageFrame& in, ImageFrame& out) = 0;
    virtual bool isEnabled() const = 0;
    virtual void setEnabled(bool en) = 0;
};

/// 滤波管线: 有序执行一组 IFilter
class FilterPipeline {
public:
    void addFilter(std::shared_ptr<IFilter> filter);
    bool apply(const ImageFrame& in, ImageFrame& out);
private:
    std::vector<std::shared_ptr<IFilter>> filters_;
};

} // namespace rm::sensor_module
```

> **具体滤波器实现** (Decimation / SpatialSmooth / TemporalSmooth / HoleFilling) 的
> 核心算法放在 `rm_alg_foundation` (见 §7.3)，Module 层负责组织调用顺序和参数管理。

#### 7.1.3 TF 坐标树构建与发布

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros `BaseRealSenseNode::publishStaticTransforms()` |
| **理由** | TF2 是 ROS 概念, HAL 中 `#include <rclcpp/...>` 被严格禁止 (RMOS 铁律) |
| **代码位置** | `rm_sensor_module/tf/sensor_tf_publisher.hpp` |
| **依赖 HAL** | ICameraHAL::getExtrinsics(from, to), ICameraHAL::getIntrinsics(stream) |
| **实现要点** | |

```
HAL 提供                                Module 层做
─────────────────────────────            ─────────────────────────
getExtrinsics(DEPTH, COLOR)             → tf2::Transform depth→color
getExtrinsics(DEPTH, IR_LEFT)           → tf2::Transform depth→ir
getIntrinsics(COLOR)                    → camera_info_manager 发布 CameraInfo
光学坐标系 (Z-forward)                  → 转换为 ROS 坐标系 (X-forward)
```

#### 7.1.4 ROS 参数服务映射

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros 的 `parameters.cpp` (150+ 动态参数) |
| **理由** | `rclcpp::Parameter` 是 ROS 概念, HAL 不感知参数服务器 |
| **代码位置** | `rm_sensor_module/param/sensor_param_bridge.hpp` |
| **依赖 HAL** | ICameraHAL::getSupportedOptions/getOption/setOption |
| **实现要点** | |

```
Module 启动时:
  1. 调用 hal->getSupportedOptions() 获取 vector<OptionInfo> (V0.3.2: 含类型/范围/枚举表)
  2. 对每个 opt: 根据 opt.type 选择 ROS 参数类型, 调用 declare_parameter()
     (无需再单独调用 getOptionInfo, getSupportedOptions 已包含完整信息)
  3. 注册 on_set_parameters_callback

参数变更时:
  callback(param) → hal->setOption(name, value)

周期查询:
  timer → hal->getOption(name) → 若与 param 不一致则 set_parameter()
```

#### 7.1.5 设备重连与 Lifecycle 管理

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros `RealSenseNodeFactory::onDeviceEvent()` + Lifecycle 节点 |
| **理由** | Lifecycle (configure/activate/deactivate) 是 ROS 概念; 重连策略是 Module 层业务 |
| **代码位置** | `rm_sensor_module/lifecycle/sensor_lifecycle.hpp` |
| **依赖 HAL** | HALFactory::setDeviceChangedCallback(), ICameraHAL 状态机 |
| **实现要点** | |

```
设备热拔:
  HALFactory::setDeviceChangedCallback() 通知 Module
  → Module 调用 hal->close()
  → 进入 Deactivated 状态
  → 启动重连定时器 (指数退避: 1s → 2s → 4s → max 30s)

设备热插:
  DeviceChangedCallback 报告 added
  → 匹配已知 serial_number
  → hal->configure() → hal->open() → hal->startStreaming()
  → 恢复 Active 状态
```

#### 7.1.6 Diagnostics 健康上报

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros 的 `diagnostic_updater` 集成 |
| **理由** | `diagnostic_updater` 是 ROS 包, HAL 不依赖 ROS |
| **代码位置** | `rm_sensor_module/diag/sensor_diagnostics.hpp` |
| **依赖 HAL** | IHardwareDevice::health(), 帧率统计, 错误码 |
| **实现要点** | 周期调用 health() → 组装 DiagnosticStatus → 发布到 /diagnostics |

---

### §7.2 ArcRT Channel 层需求

ArcRT 是 RMOS 的通信中间件层。以下功能由 ArcRT 框架提供, Module 层调用:

#### 7.2.1 跨品类多传感器时间戳对齐 (message_filter)

| 项目 | 说明 |
|------|------|
| **来源** | realsense-ros 中 ROS message_filters 的 ApproximateTime/ExactTime 同步 |
| **RMOS 依据** | RMOS V5 架构 §横切关注点/时间同步: "多传感器时间戳对齐由 Channel 层 message_filter 支持" |
| **适用场景** | Camera Image + LiDAR PointCloud + IMU Data 联合时间对齐 |
| **与 §2.10 的区别** | §2.10 ISyncCoordinator = 同品类多相机 HAL 级帧对齐; 这里 = 跨品类消息级对齐 |
| **接口形式** | |

```
ArcRT Channel message_filter:
  input:  Channel<ImageMsg>  + Channel<PointCloudMsg> + Channel<ImuMsg>
  policy: ApproximateTime(tolerance=10ms)
  output: SynchronizedCallback(image, cloud, imu)

Module 层仅需:
  1. 将 HAL 数据转换为 rm_interface 消息发布到各 Channel
  2. 在消费端使用 message_filter 注册联合回调
```

---

### §7.3 rm_alg_foundation 层需求 (纯 C++ 算法库)

rm_alg_foundation 提供不依赖任何中间件的纯 C++ 算法。以下是从 realsense-ros 对标
分析中识别的算法需求:

#### 7.3.1 深度图后处理滤波器

| 滤波器 | realsense-ros 对标 | 算法描述 | 输入/输出 |
|--------|-------------------|---------|----------|
| **DecimationFilter** | rs2::decimation_filter | 均值下采样, 降低分辨率 (2×/4×/8×) | Z16 → Z16 (缩小) |
| **SpatialFilter** | rs2::spatial_filter | edge-preserving 空间域平滑 (双边滤波) | Z16 → Z16 |
| **TemporalFilter** | rs2::temporal_filter | 时域 IIR 平滑, 减少闪烁 | Z16 序列 → Z16 |
| **HoleFillingFilter** | rs2::hole_filling_filter | 填充深度空洞 (farest/nearest/left) | Z16 → Z16 |
| **ThresholdFilter** | rs2::threshold_filter | 距离范围截断 [min_dist, max_dist] | Z16 → Z16 |

```cpp
// rm_alg_foundation/filter/depth_filters.hpp
namespace rm::alg {

/// 通用深度图操作接口
class IDepthFilter {
public:
    virtual ~IDepthFilter() = default;
    virtual void apply(const uint16_t* in, uint16_t* out,
                       int width, int height, float depth_scale) = 0;
};

class DecimationFilter : public IDepthFilter { /* factor: 2/4/8 */ };
class SpatialFilter    : public IDepthFilter { /* alpha, delta, iterations */ };
class TemporalFilter   : public IDepthFilter { /* alpha, delta */ };
class HoleFillingFilter: public IDepthFilter { /* mode: farest/nearest/left */ };
class ThresholdFilter  : public IDepthFilter { /* min_m, max_m */ };

} // namespace rm::alg
```

#### 7.3.2 点云生成 (Depth + Intrinsics → 3D)

| 项目 | 说明 |
|------|------|
| **对标** | rs2::pointcloud filter, realsense-ros PointCloudFilter |
| **算法** | 逐像素反投影: $Z = \text{depth}[u,v] \times \text{scale};\; X = (u - c_x) \cdot Z / f_x;\; Y = (v - c_y) \cdot Z / f_y$ |
| **输入** | Z16 深度图 + CameraIntrinsics (fx, fy, cx, cy) |
| **输出** | PointCloud (XYZ 或 XYZRGB, 若提供 Color + Extrinsics) |

```cpp
// rm_alg_foundation/geometry/pointcloud_generator.hpp
namespace rm::alg {

struct PointCloudXYZ {
    std::vector<float> points;  // [x0,y0,z0, x1,y1,z1, ...]
    int valid_count;
};

PointCloudXYZ depthToPointCloud(
    const uint16_t* depth, int w, int h,
    float fx, float fy, float cx, float cy,
    float depth_scale,
    float min_depth = 0.1f,
    float max_depth = 10.0f);

} // namespace rm::alg
```

#### 7.3.3 坐标系变换 (Optical ↔ ROS)

| 项目 | 说明 |
|------|------|
| **对标** | realsense-ros 大量 optical→ROS 坐标变换 |
| **问题** | 相机光学坐标系 (X-right, Y-down, Z-forward) ≠ ROS (X-forward, Y-left, Z-up) |
| **位置** | `rm_alg_foundation/geometry/coord_transform.hpp` |

```cpp
namespace rm::alg {

struct Transform3D {
    float rotation[9];     // 3×3 行主序
    float translation[3];
};

/// 相机光学坐标系 → ROS 标准坐标系
Transform3D opticalToRos();

/// 组合外参和坐标系变换
Transform3D composeTransforms(const Transform3D& a, const Transform3D& b);

} // namespace rm::alg
```

---

### §7.4 层次归属总表

> 本表汇总所有从 realsense-ros 对标分析中识别的差距及其 RMOS 层次归属。

| # | 功能 | RMOS 层次 | 实现位置 | HAL 依赖 | 优先级 |
|---|------|----------|---------|---------|--------|
| 1 | 设备发现 / 热插拔 | **HAL** | HALFactory + DeviceInfo | — | P0 |
| 2 | Profile 查询 | **HAL** | ICameraHAL::getSupportedProfiles() | — | P0 |
| 3 | 运行时硬件选项 | **HAL** | getSupportedOptions()/getOptionInfo()/getOption()/setOption() | — | P1 |
| 4 | 扩展标定参数 | **HAL** | getExtrinsics(from,to), getIntrinsics(stream) | — | P0 |
| 5 | 单设备 FrameSet | **HAL** | ICameraHAL::setFrameSetCallback() | — | P1 |
| 6 | 多相机帧同步 | **Module** | ISyncCoordinator (§2.10, §7.1.1) | FrameSetCallback | P1 |
| 7 | Filter Pipeline | **Module** + **rm_alg** | FilterPipeline + IDepthFilter (§7.1.2, §7.3.1) | 原始帧 | P2 |
| 8 | TF 坐标树 | **Module** | SensorTFPublisher (§7.1.3) | getExtrinsics/getIntrinsics | P1 |
| 9 | ROS 参数映射 | **Module** | SensorParamBridge (§7.1.4) | getOption/setOption | P2 |
| 10 | 设备重连/Lifecycle | **Module** | SensorLifecycle (§7.1.5) | DeviceChangedCallback | P1 |
| 11 | Diagnostics | **Module** | SensorDiagnostics (§7.1.6) | health() | P2 |
| 12 | 跨品类时间对齐 | **ArcRT Channel** | message_filter (§7.2.1) | — (消息级) | P1 |
| 13 | 点云生成 | **rm_alg** | depthToPointCloud (§7.3.2) | 深度图+内参 | P1 |
| 14 | 坐标系变换 | **rm_alg** | opticalToRos (§7.3.3) | — | P2 |
| 15 | StreamIndex 统一流索引 | **HAL** | `StreamIndex{StreamType,int}` (§2.1 V0.3.2) | — | P0 |
| 16 | OptionInfo 自描述选项 | **HAL** | `OptionInfo{type,enum_values,...}` (§2.1 V0.3.2) | — | P1 |
| 17 | TimestampDomain + 帧元数据 | **HAL** | `ImageFrame::timestamp_domain/frame_number/actual_*` | — | P1 |
| 18 | ISyncManager 单设备硬件同步 | **HAL** | ISyncManager (§2.11 V0.3.2) | — | P1 |
| 19 | IDecoder/IDecoderFactory | **HAL (内部)** | 平台解码器 (§2.7.4 V0.3.2) | — | P1 |
| 20 | DistortionModel + IMUCalibration | **HAL** | 标定扩展结构体 (§2.1A V0.3.2) | — | P2 |
| 21 | 进程锁 enableProcessLock | **HAL** | HALFactory::enableProcessLock (§6.1 V0.3.2) | — | P2 |
| 22 | ParameterProvider 自动映射 | **Module** | SensorParamBridge 增强 (§7.5.1 V0.3.2) | getOption/setOption | P2 |
| 23 | TF 光学帧四元数约定 | **Module** | SensorTFPublisher (§7.5.2 V0.3.2) | getExtrinsics | P1 |
| 24 | IMU 软件对齐策略 | **Module/ArcRT** | 插值对齐 (§7.5.3 V0.3.2) | ImuCallback | P2 |
| 25 | 图像翻转/去畸变后处理 | **rm_alg** | FlipFilter/UndistortFilter (§7.5.4 V0.3.2) | 原始帧 | P2 |

> **优先级说明**: P0 = 基础能力, HAL 首版必须具备; P1 = 核心功能, 第一轮 Module 集成需要;
> P2 = 增强功能, 可后续迭代添加。

---

### §7.5 Module 层最佳实践 — 来自双厂商对比 (**V0.3.2 新增**)

> **来源**: 对比分析 `/docs/hal_comparative_analysis/` 17 份文档后提炼的 Module 层实现指南。
> 这些模式属于 Module Library 层 (`rm_sensor_module`), HAL 层**不实现**。

#### 7.5.1 ParameterProvider 自动映射 (来自 RealSense SensorParams)

| 项目 | 说明 |
|------|------|
| **RealSense 做法** | `SensorParams::registerDynamicOptions(rs2::options)` 自动遍历 SDK 选项 → `declare_parameter()` |
| **Orbbec 做法** | 手工声明 100+ 参数 (逐一 `parameters_->setParam("xxx", ...)`) — 不推荐 |
| **RMOS 推荐** | 调用 `ICameraHAL::getSupportedOptions()` → 动态声明 ROS 参数 |

```cpp
// rm_sensor_module/param/sensor_param_bridge.hpp
// 启动时自动注册所有 HAL 硬件选项为 ROS 参数 (动态)
void SensorParamBridge::registerDynamicOptions(ICameraHAL& hal, rclcpp::Node& node) {
    auto options = hal.getSupportedOptions();  // 返回 vector<OptionInfo> (V0.3.2 新增)
    for (const auto& opt : options) {
        // 根据 OptionType 声明对应 ROS 类型
        if (opt.type == OptionType::Bool)
            node.declare_parameter<bool>(opt.name, opt.default_value != 0.0f);
        else if (opt.type == OptionType::Enum)
            node.declare_parameter<std::string>(opt.name, /* enum key for default */);
        else
            node.declare_parameter<double>(opt.name, opt.default_value,
                rcl_interfaces::build<rcl_interfaces::ParameterDescriptor>()
                    .description(opt.description)
                    .floating_point_range({rcl_build<FloatingPointRange>()
                        .from_value(opt.min).to_value(opt.max).step(opt.step)}));

        // 注册变更回调 → 写入 HAL
        node.add_on_set_parameters_callback([&hal, name=opt.name](auto& params) {
            for (auto& p : params) {
                if (p.get_name() == name)
                    hal.setOption(name, static_cast<float>(p.as_double()));
            }
            return rcl_interfaces::build<SetParametersResult>().successful(true);
        });
    }
}
```

**好处**: SDK 升级新增硬件选项时, Module 层**自动暴露**, 无需修改代码。

#### 7.5.2 TF 光学帧四元数约定

| 项目 | 说明 |
|------|------|
| **来源** | 两家厂商均遵循 ROS 光学坐标系约定 |
| **问题** | 相机光学坐标系 (Z-forward, X-right, Y-down) ≠ ROS (X-forward, Y-left, Z-up) |
| **位置** | `rm_sensor_module/tf/sensor_tf_publisher.hpp` |

```
光学坐标系 → ROS 坐标系的变换四元数:
  x = -0.5,  y = 0.5,  z = -0.5,  w = 0.5

验证: 将此四元数转为旋转矩阵:
  [ 0  0  1 ]   (光学 Z-forward → ROS X-forward)
  [-1  0  0 ]   (光学 X-right  → ROS -Y → 左轴)
  [ 0 -1  0 ]   (光学 Y-down   → ROS -Z → 下轴)
```

```cpp
// SensorTFPublisher 中使用此固定四元数
static const geometry_msgs::msg::Quaternion kOpticalToRos =
    []() { geometry_msgs::msg::Quaternion q;
           q.x=-0.5; q.y=0.5; q.z=-0.5; q.w=0.5; return q; }();

// 对每个流: depth_link → depth_optical
tf_static_broadcaster_->sendTransform({
    .header = {.frame_id = "depth_link"},
    .child_frame_id = "depth_optical_frame",
    .transform = {.rotation = kOpticalToRos},  // 纯旋转, 无平移
});
```

#### 7.5.3 IMU 软件对齐策略 (accel + gyro 不同采样率)

| 项目 | 说明 |
|------|------|
| **问题** | Orbbec/RealSense 相机内置 IMU: accel 50Hz, gyro 200Hz (采样率不同) |
| **硬件同步方案** | Orbbec Gemini 330 支持 `enable_sync_output_accel_gyro=true` (设备级硬件对齐, 推荐) |
| **软件对齐方案** | RealSense `imu_callback_sync`: 线性插值 accel 到 gyro 时刻 |
| **RMOS Module 层推荐** | 优先使用 ISyncManager 开启硬件同步; 若不支持, 用 message_filter 软对齐 |

```
策略优先级:
1. [首选] ICameraHAL::setOption("enable_sync_output_accel_gyro", 1.0)
   → HAL 回调直接返回已对齐的 accel+gyro 组合帧

2. [备选] ArcRT Channel message_filter::ApproximateTime
   → 订阅独立 accel/gyro channel → 时间容忍 ±5ms 对齐
   → 输出: SynchronizedMsg<ImuMsg_accel, ImuMsg_gyro>

3. [最后] Module 层软件插值 (仅当硬件不支持且 message_filter 不适用时)
   → 缓存最近 N 个 accel + gyro 样本, 对 gyro 时刻做 accel 线性插值
   → 性能开销小, 但引入最大 1/accel_rate ≈ 20ms 误差
```

**注意**: 相机内置 IMU (通过 ICameraHAL 获取) 与独立外部 IMU (IImuHAL, YESENSE) 的对齐
应在 ArcRT Channel 层通过 message_filter 处理, 不在 Module 层实现。

#### 7.5.4 图像后处理归属 (翻转 / 镜像 / 去畸变)

| 操作 | 来源 | RMOS 归属 | 接口位置 |
|------|------|----------|---------|
| 水平翻转/镜像 `cv::flip(img, ±1)` | 两家均有 `flip_stream[sip]` / `mirror_stream[sip]` | **rm_alg_foundation** | `rm_alg/image/flip_filter.hpp` |
| 旋转 90/180/270° `cv::rotate()` | 两家均有 `rotation_stream[sip]` | **rm_alg_foundation** | `rm_alg/image/rotate_filter.hpp` |
| 去畸变 `cv::undistort()` | Orbbec `enable_color_undistortion_` | **rm_alg_foundation** | `rm_alg/geometry/undistort_filter.hpp` |
| D2C 深度对齐 | Orbbec: HW/SW mode; RealSense: `rs2::align` | **HAL 内部** | `ICameraHAL::CameraConfig.align_mode` |
| 深度后处理滤波器 | 两家 SDK 均有 | **rm_alg_foundation** | `rm_alg/filter/depth_filters.hpp` (§7.3.1) |

```cpp
// rm_alg_foundation/image/flip_filter.hpp — 示例接口草案
namespace rm::alg {

/// 图像几何变换滤波器 (纯 C++, 无中间件依赖)
struct ImageTransformConfig {
    bool flip_vertical   = false;  // 垂直翻转 (cv::flip y=0)
    bool mirror_horizontal = false; // 水平镜像 (cv::flip x=1)
    int  rotation_deg    = 0;      // 旋转度数: 0/90/180/270
};

/// 应用几何变换到 BGR8/MONO8 图像
bool applyImageTransform(const uint8_t* src, uint8_t* dst,
                          int width, int height, int channels,
                          const ImageTransformConfig& cfg);

} // namespace rm::alg
```

**调用方式**: Module 层从 ROS 参数读取 transform 配置, 在收到 HAL 原始帧后调用
`rm::alg::applyImageTransform()`, 再发布到 Topic。HAL 层不做此操作。

---

*本文档 V0.3.2, 2026-04-28。已整合双厂商对比分析精华 (OrbbecSDK_ROS2 v2.7.6 vs realsense-ros 4.57.7)。*
