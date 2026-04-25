# A5. 时钟同步与触发（GPIO / PTP）

## 1. 这篇文章要解决什么问题？

多传感器系统（相机、雷达、激光雷达等）需要统一时间基准，以实现帧对齐、数据融合和因果性保证。本文梳理 GPIO/TTL 硬触发和 PTP/IEEE 1588 软同步两种核心方案。

---

## 2. 数据链路图

### 2.1 GPIO 硬触发（多相机同步曝光）
```
[主时钟源 / MCU / FPGA / GNSS PPS]
        │
        │ GPIO / TTL 方波脉冲（上升沿触发）
        │
        ├──► Camera 0 TRIGGER_IN  →  同步曝光开始
        ├──► Camera 1 TRIGGER_IN  →  同步曝光开始
        ├──► Camera 2 TRIGGER_IN  →  同步曝光开始
        └──► LiDAR   SYNC_IN      →  旋转起始对齐
                     ↓
             [所有传感器帧在同一时刻开始采样]
```

### 2.2 PTP/IEEE 1588（以太网时间同步）
```
[GNSS 时间源 / GPS PPS + NMEA] ──► PTP Grandmaster
                                         │ PTP 报文（Sync/Follow_Up/Delay_Req/Resp）
                                    ─────┼─────────────────────────
                                    以太网交换机（支持 PTP 透传或 Boundary Clock）
                                    ─────┼─────────────────────────
                             ┌───────────┼────────────────┐
                             │           │                │
                       [Camera A]   [LiDAR B]      [Radar C]
                       PTP Slave    PTP Slave       PTP Slave
                             │           │                │
                        hw_timestamp  hw_timestamp  hw_timestamp
                             └───────────┴────────────────┘
                                    对齐精度 < 1 µs（硬件 PTP）
```

---

## 3. 方案对比

| 方案 | 同步精度 | 适用距离 | 硬件要求 | 典型场景 |
|------|----------|----------|----------|---------|
| GPIO/TTL 硬触发 | < 1 µs | 短距离（线缆限制） | 触发引脚 | 多相机同步曝光 |
| GNSS PPS + GPIO | < 100 ns | — | GNSS 模块 + GPIO | 最高精度硬触发 |
| PTP/IEEE 1588 | < 1 µs（硬件）；~10 µs（软件） | 网络范围 | PTP 网卡 / 交换机 | 以太网传感器集群 |
| GPS NTP | ~1–10 ms | — | GNSS 模块 | 粗粒度同步 |
| ROS Time Sync | ~1–5 ms | — | 无特殊硬件 | 软同步（精度要求低） |

---

## 4. GPIO / TTL 硬触发

### 4.1 触发信号规格

| 参数 | 典型值 |
|------|--------|
| 信号电平 | 3.3 V 或 5 V TTL |
| 脉冲宽度 | 1–10 ms（相机要求不同） |
| 触发沿 | 上升沿（多数相机） |
| 最大频率 | ≤ 帧率（避免丢触发） |
| 信号阻抗 | 50 Ω 匹配（长线缆时） |

### 4.2 典型连接（以 FLIR 相机为例）
```
MCU GPIO_OUT  ──── 限流电阻(100Ω) ──── Camera OPTO_IN
                                           │
                                       光耦隔离（±电气隔离）
```

### 4.3 Linux 侧读取触发时间戳
```c
// V4L2 硬件时间戳（需驱动支持 V4L2_BUF_FLAG_TIMESTAMP_SOE）
struct v4l2_buffer buf;
buf.flags & V4L2_BUF_FLAG_TIMESTAMP_SOE; // Start of Exposure
struct timeval ts = buf.timestamp;        // 硬件时间戳
```

### 4.4 MCU 触发脉冲生成（以 STM32 为例）
```c
// TIM2 生成 30 Hz 触发脉冲
void trigger_init(void) {
    // 配置 Timer 输出比较，周期 33.3 ms，脉宽 5 ms
    htim2.Init.Period = 33333 - 1; // 1 µs tick
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
}
```

---

## 5. PTP / IEEE 1588

### 5.1 关键术语

| 术语 | 说明 |
|------|------|
| Grandmaster | 最优时钟源（通常连 GNSS） |
| Boundary Clock | 交换机上的时钟，隔离 PTP 域 |
| Transparent Clock | 交换机透传，修正驻留时间 |
| Hardware Timestamp | 网卡/PHY 打时间戳，精度 < 1 µs |
| Software Timestamp | OS 协议栈打时间戳，精度 ~10 µs |

### 5.2 Linux PTP 配置（linuxptp）
```bash
# 安装
sudo apt install linuxptp

# 查看网卡 PTP 能力
ethtool -T eth0

# 启动 PTP4L（硬件时间戳）
sudo ptp4l -i eth0 -H -m        # -H: 硬件模式；-m: 打印到终端

# 同步系统时钟
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m

# 查看同步状态
sudo pmc -u -b 0 'GET TIME_STATUS_NP'
```

### 5.3 PTP 报文交换（E2E 模式）
```
Master                              Slave
  │── Sync (t1) ──────────────────► │ 记录 t2
  │── Follow_Up(t1) ───────────────► │
  │◄── Delay_Req (t3) ────────────── │
  │── Delay_Resp(t4) ──────────────► │

偏移 = [(t2-t1) - (t4-t3)] / 2
传输延迟 = [(t2-t1) + (t4-t3)] / 2
```

### 5.4 验证同步精度
```bash
# 使用 ts2phc 工具（GNSS PPS 对准）
sudo ts2phc -f /etc/linuxptp/ts2phc-TC.cfg -s nmea -m

# 查看偏移统计（ptp4l 输出）
# master offset   -123 s2 freq  -12345 path delay    456
# 目标：|offset| < 1000 ns（硬件 PTP），< 10000 ns（软件 PTP）
```

---

## 6. 时间戳策略

### 6.1 时间戳类型对比

| 类型 | 精度 | 获取方式 |
|------|------|----------|
| 硬件时间戳（曝光起始） | < 1 µs | V4L2 `V4L2_BUF_FLAG_TIMESTAMP_SOE` |
| 硬件时间戳（DMA 完成） | < 10 µs | V4L2 `V4L2_BUF_FLAG_TIMESTAMP_EOF` |
| 软件时间戳（驱动入队） | ~100 µs | `clock_gettime(CLOCK_MONOTONIC)` |
| 软件时间戳（应用出队） | ~1 ms | 应用层 `gettimeofday` |

**推荐**：优先使用硬件曝光起始时间戳（SOE），配合 PTP 时钟转换为统一时间域。

### 6.2 元数据字段规范

```json
{
  "frame_id": 12345,
  "sensor_id": "camera_front_center",
  "sensor_time_ns": 1700000000123456789,
  "host_time_ns":   1700000000124000000,
  "ptp_time_ns":    1700000000123500000,
  "exposure_us":    8000,
  "trigger_id":     12344
}
```

---

## 7. 多传感器时间对齐流程

```
1. 确定时间基准（Grandmaster / GNSS PPS）
2. 所有以太网传感器接入 PTP 域，配置 PTP Slave
3. GPIO 触发设备从 GNSS PPS 或 MCU（已对齐 PTP）产生脉冲
4. 采集时记录 sensor_time + ptp_time（转换偏移）
5. 上层 ROS/中间件按 ptp_time 排序对齐
6. 定期验证：记录 offset 分布，告警阈值 > 500 µs
```

---

## 8. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| PTP 同步间隔 | 125 ms（-3） | `logSyncInterval` |
| PTP 路径延迟测量间隔 | 1 s（0） | `logMinDelayReqInterval` |
| 触发脉冲宽度 | 5 ms | 多数相机要求 > 1 ms |
| 时间戳类型 | SOE（Start of Exposure） | 最接近真实采样时刻 |
| PTP 偏移告警阈值 | 500 µs | 超过告警并记录 |

---

## 9. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| PTP 同步偏移（硬件） | < 1 µs | `ptp4l` offset 日志 |
| GPIO 触发抖动 | < 1 µs | 示波器测量 |
| 多相机帧时间差 | < 1/2 曝光时间 | 闪光灯测试法 |
| 传感器帧时间戳单调性 | 100% 单调 | 日志分析 |

---

## 10. 常见问题与排查步骤（Checklist）

- [ ] PTP offset 持续漂移 → 检查交换机是否支持 PTP 透传（`-E2E` 模式）
- [ ] 软件时间戳抖动大 → 改用硬件时间戳；检查系统负载
- [ ] 多相机帧不同步 → 确认 GPIO 触发线连接；相机固件是否开启外触发模式
- [ ] V4L2 时间戳为 0 → 内核驱动未支持 `V4L2_BUF_FLAG_TIMESTAMP_SOE`
- [ ] ptp4l 找不到 Grandmaster → 检查 PTP vlan 配置；确认交换机转发 PTP 多播
- [ ] 时间戳回跳（非单调） → NTP 与 PTP 混用冲突，禁用 NTP 或使用 chrony + PTP

---

## 11. 参考资料

- [linuxptp 项目](https://linuxptp.sourceforge.net/)
- [IEEE 1588-2019 标准](https://standards.ieee.org/ieee/1588/6825/)
- [V4L2 时间戳文档](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html#c.v4l2_buffer.timestamp)
- [FLIR 相机触发模式手册](https://www.flir.com/support-center/iis/machine-vision/application-note/configuring-synchronized-capture-with-multiple-cameras/)
- [ROS 时间同步最佳实践](https://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot)
