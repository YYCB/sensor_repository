# D4 – IMU HAL 设计

IMU HAL 与 `Sensor HAL.md V0.3.2 §4` 高度一致。本文件只描述变更点和补充说明。

---

## 变更点

| 字段 | 原值 | 修改后 |
|------|------|--------|
| `ImuData::timestamp_ns` | `uint64_t` | `rm::hal::SensorTimestamp timestamp` |

默认 `domain = System`（串口接收时刻）。若设备 TLV 帧中的 DataID `0x80`
（SAMPLE_TIMESTAMP，单位 ms）可用，可将 domain 升级为 `Hardware` 以提供更准确的时钟。

其余一切（`ImuConfig` / `ImuData` 字段 / `ImuFusionMode` / `ImuDeviceInfo` / YESENSE TLV 协议映射）**完全保留原设计**。

---

## IImuHAL 方法表

| 方法 | 说明 |
|------|------|
| `configure(ImuConfig)` | 配置串口、采样率、量程、融合策略 |
| `open() / close() / reset()` | 继承自 ISensorHAL |
| `getData(ImuData& out)` | 轮询最新帧（ring buffer depth=4） |
| `setDataCallback(ImuCallback)` | 每个 TLV 帧到达时触发（~200 Hz） |
| `getDeviceInfo()` | 返回噪声参数，供 EKF / UKF 配置 |
| `resetOrientation()` | 重置 AHRS 姿态估计（可选，默认返回 false） |

---

## ImuData 字段与 YESENSE DataID 映射

| 字段 | DataID | 原始单位 | 缩放因子 | SI 单位 |
|------|--------|---------|---------|---------|
| `accel_x/y/z` | 0x10 | g (int32) | ×1e-6 × 9.80665 | m/s² |
| `gyro_x/y/z` | 0x20 | dps (int32) | ×1e-6 × π/180 | rad/s |
| `quat_w/x/y/z` | 0x41 | 无 (int32) | ×1e-6 | 无量纲 |
| `linear_accel_x/y/z` | 0x11 | g (int32) | ×1e-6 × 9.80665 | m/s²（已去重力） |
| `euler_roll/pitch/yaw` | 0x40 | deg (int32) | ×1e-6 × π/180 | rad |
| `mag_x/y/z` | 0x30 | μT (int32) | ×1e-3 | μT |
| `temperature` | 0x01 | °C (int16) | ×0.01 | °C |
| `status_word` | 0x70 | — | — | 位域 |
| `sample_timestamp_ms` | 0x80 | ms (uint32) | ×1 | ms |

---

## 状态机

```
Closed ──configure()──► Configured ──open()──► Streaming
  ▲                                                │
  └───────────── close() ──────────────────────────┘
任意状态 ──fault──► Faulted ──reset()──► Closed
```

open() 后串口立即持续输出；无独立 startStreaming。

---

## 多实例管理

```yaml
sensors:
  body_imu:    { type: yesense, port: /dev/yesenseIMU_body,    baudrate: 460800 }
  chassis_imu: { type: yesense, port: /dev/yesenseIMU_chassis, baudrate: 460800 }
```

两个独立的 `IImuHAL` 实例各自持有串口 fd 和读取线程，互不干扰，通过 `device_id` 区分。

---

## GNSS 字段说明

YESENSE TLV DataID `0x60`（Location）和 `0x61`（Speed over Ground）当前未在 `ImuData` 中暴露。

若未来有独立 GNSS 接收机接入，应新增 `IGnssHAL` 接口而非在 `ImuData` 中堆砌字段。

---

## 线程模型

```
串口读取线程 (read + TLV 解析)
    │
    ├── getData() 轮询 → ring buffer (depth=4)
    └── ImuCallback → 每帧触发
```

回调在读取线程执行；**回调内禁止调用 open / close / configure**（避免死锁）。
