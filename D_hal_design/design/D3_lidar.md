# D3 – LiDAR HAL 设计（2D / 3D 拆分）

## 设计决策：拆分 I2DLidarHAL 与 I3DLidarHAL

`Sensor HAL.md V0.3.2 §3` 的 `ILidarHAL` 仅覆盖 2D 激光扫描（BlueSea UDP）。
但仓库中同时存在 3D 旋转 LiDAR（Velodyne / Livox）的使用需求。

两类传感器的数据语义根本不同，使用统一接口会导致大量"不适用"的方法，
降低类型安全性和可读性：

| 维度 | I2DLidarHAL（BlueSea） | I3DLidarHAL（Velodyne / Livox） |
|------|----------------------|--------------------------------|
| 输出类型 | `LaserScanData`（极坐标，ROS LaserScan 等效） | `PointCloudXYZI`（笛卡尔，每点含时间偏移） |
| 点密度 | ~360 点 / 圈 | 数万–数十万点 / 圈 |
| per-point 时间 | 无 | `time_offset_s`（运动补偿必需） |
| 协议 | BlueSea 私有 UDP，6 种帧头 | Velodyne PCAP UDP / Livox SDK |
| 连接 | UDP 单播 | UDP 广播 / SDK 管理 |

---

## I2DLidarHAL（2D 激光扫描）

**目标传感器**：BlueSea LDS-U50C-S、LDS-U80C-S

**核心接口**：

```cpp
class I2DLidarHAL : public ISensorHAL {
    virtual bool configure(const Lidar2DConfig& config) = 0;
    virtual bool getScan(LaserScanData& out) = 0;                              // 轮询
    using ScanCallback = std::function<void(shared_ptr<const LaserScanData>)>;
    virtual void setScanCallback(ScanCallback cb) = 0;                         // 回调
    virtual bool setScanFrequency(int hz) { return false; }                    // 可选
};
```

**LaserScanData 时间域**：
- `domain = System`：UDP 接收时打戳（默认）
- `domain = Hardware`：BlueSea HDR2/HDR3 帧头中的 `timestamp_lo/hi` 字段可用时升级

**状态机**：
```
Closed ──configure()──► Configured ──open()──► Streaming
  ▲                                                │
  └─────────── close() ────────────────────────────┘
任意状态 ──fault──► Faulted ──reset()──► Closed
```
open() 后 UDP 数据即开始到达，无独立 startReceiving。

---

## I3DLidarHAL（3D 点云）

**目标传感器**：Velodyne VLP-16、VLP-32C、HDL-64E；Livox Mid-360 等

**核心接口**：

```cpp
class I3DLidarHAL : public ISensorHAL {
    virtual bool configure(const Lidar3DConfig& config) = 0;
    virtual bool getPointCloud(PointCloudXYZI& out) = 0;                        // 轮询
    using PointCloudCallback = std::function<void(shared_ptr<const PointCloudXYZI>)>;
    virtual void setPointCloudCallback(PointCloudCallback cb) = 0;              // 回调
};
```

**PointXYZI 关键字段**：

| 字段 | 含义 |
|------|------|
| `x, y, z` | 笛卡尔坐标（m），LiDAR 机体坐标系 |
| `intensity` | 归一化反射强度 [0,1] |
| `time_offset_s` | 相对 PointCloudXYZI::timestamp 的时间偏移（运动补偿输入） |
| `ring` | 激光环编号（Velodyne channel index，0-based） |

**LidarReturnMode**：`Strongest / Last / Dual`（多回波支持）

---

## 工厂别名与注册

```cpp
using Lidar2DFactory = rm::hal::HALFactory<I2DLidarHAL>;
using Lidar3DFactory = rm::hal::HALFactory<I3DLidarHAL>;

// 驱动注册示例
REGISTER_LIDAR2D_HAL("bluesea",  BlueseaLidar2DHAL)
REGISTER_LIDAR3D_HAL("velodyne", VelodyneLidar3DHAL)
REGISTER_LIDAR3D_HAL("livox",    LivoxLidar3DHAL)
REGISTER_LIDAR3D_HAL("sim",      SimLidar3DHAL)
```

---

## 与原 ILidarHAL 的对比

原 `Sensor HAL.md §3` 的协议细节（BlueSea 6 种帧头、FanAssembler、STM32 CRC32）
完全保留在 `I2DLidarHAL` 对应的 `BlueseaLidar2DHAL` 实现中，接口层面不暴露。
`I3DLidarHAL` 是本文件夹相对原设计的净增量。
