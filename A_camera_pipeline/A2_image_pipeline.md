# A2. 图像链路（ISP / 预处理）

## 1. 这篇文章要解决什么问题？

将 RAW sensor 输出（或 YUV 直出相机）转换为算法/编码器可用的标准格式，同时完成白平衡、降噪、畸变校正、OSD 等预处理。

---

## 2. 数据链路图

```
[Camera RAW Output]
  (RGGB/BGGR/GRBG …)
        │
        ▼
[ISP（Image Signal Processor）]
  ├─ 去马赛克（Demosaic）
  ├─ 黑电平校正（BLC）
  ├─ 镜头阴影校正（LSC）
  ├─ 白平衡（AWB）
  ├─ 色彩校正矩阵（CCM）
  ├─ 降噪（NR：空域 + 时域）
  ├─ 伽马 / Tone Mapping / HDR 合成
  └─ 输出：RGB888 / YUV420 / YUV422
        │
        ▼
[颜色空间 & 格式转换]
  RGB ↔ YUV（NV12 / I420 / NV21）
        │
        ▼
[叠加 & 几何变换]
  ├─ OSD（时间戳、通道号、水印）
  ├─ 裁剪（Crop）/ 缩放（Scale）
  └─ 畸变校正（Undistort / Remap）
        │
        ▼
[编码器 / 算法消费]
  H.264/H.265 编码器  or  检测/SLAM 算法
```

---

## 3. 关键接口 / 协议 / 格式

| 阶段 | 典型格式 | 说明 |
|------|----------|------|
| RAW Bayer | RGGB10 / RGGB12 | 位宽取决于 sensor |
| ISP 输出 | NV12（YUV420 SP） | 编码器首选 |
| ISP 输出 | RGB888 / BGRA | 算法、OpenGL 纹理 |
| 中间缓冲 | DMABUF fd | 零拷贝跨模块传递 |

---

## 4. ISP 处理流水线详解

### 4.1 去马赛克（Demosaic）

- 算法：双线性（快，质量差）→ AHD/VCD（质量好，算力中等）→ 深度学习（最优）。
- 嵌入式平台通常由硬件 ISP 完成，不需要软件实现。

### 4.2 白平衡（AWB）

| 模式 | 说明 |
|------|------|
| 手动 | 固定 R/G/B 增益，适合受控环境 |
| 自动（灰世界假设） | 运行时调整，适合变光环境 |
| 一次性 AWB | 拍标准白板后锁定 |

### 4.3 HDR 合成

- 多帧曝光合并（Short + Long Exposure Frames）；
- 需要保证帧间运动补偿，否则出现鬼影；
- 输出通常为 16bit，需 Tone Mapping 压缩到 8bit 显示。

### 4.4 降噪

| 类型 | 方法 | 说明 |
|------|------|------|
| 空域 NR | Bilateral / NLM | 单帧处理，保边去噪 |
| 时域 NR | IIR / 3DNR | 跨帧累积，低光效果好 |
| 硬件 NR | ISP 内置 | 推荐优先使用 |

---

## 5. 颜色空间与格式转换

### 5.1 RGB → YUV（BT.601）
```
Y  =  0.299 R + 0.587 G + 0.114 B
Cb = -0.169 R - 0.331 G + 0.500 B + 128
Cr =  0.500 R - 0.419 G - 0.081 B + 128
```

### 5.2 常用 YUV 格式

| 格式 | 别名 | 内存布局 | 适用 |
|------|------|----------|------|
| NV12 | YUV420 SP | YYYY…UV… | H.264/H.265 编码器 |
| I420 | YUV420P | YYY…U…V… | FFmpeg / x264 默认 |
| NV21 | YUV420 SP | YYYY…VU… | Android Camera2 |
| UYVY | YUV422 | UYVY交错 | 采集卡直出 |

### 5.3 快速转换（libyuv / OpenCV / NPP）
```cpp
// libyuv RGB24 → NV12
libyuv::RGB24ToNV12(src_rgb, src_stride,
                    dst_y,   dst_stride_y,
                    dst_uv,  dst_stride_uv,
                    width, height);

// OpenCV BGR → YUV I420
cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
```

---

## 6. 叠加与几何变换

### 6.1 OSD（On-Screen Display）
```
推荐在编码前、ISP 输出后叠加，避免对 RAW 数据造成污染。
内容：时间戳（精确到 ms）、通道编号、IP 地址、告警图标。
```

### 6.2 畸变校正（Undistort）

使用 OpenCV `cv::remap` 预计算 mapX / mapY：
```cpp
// 标定后获得 cameraMatrix, distCoeffs
cv::Mat map1, map2;
cv::initUndistortRectifyMap(
    cameraMatrix, distCoeffs,
    cv::Mat(), newCameraMatrix,
    imageSize, CV_16SC2, map1, map2);

// 逐帧执行（零拷贝 + GPU 加速）
cv::remap(src, dst, map1, map2, cv::INTER_LINEAR);
```

**注意**：仅算法输入流需要校正，录制/监控流可不校正以减少 CPU/GPU 负载。

### 6.3 缩放（Scale）
- 硬件缩放（VI/ISP Scaler）优先，减少 CPU 占用；
- 编码器内部缩放（如 NVENC 的 `--resize`）次选；
- 软件 `libyuv::ScaleUVPlane` 最后考虑。

---

## 7. 关键参数与默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| ISP 输出格式 | NV12 | 编码器最佳输入 |
| 畸变校正 alpha | 0（保留所有有效像素） | 也可用 1 保留全部黑边 |
| OSD 字体大小 | 24–36 px | 视分辨率而定 |
| libyuv 缩放算法 | `kFilterBilinear` | 质量与速度均衡 |

---

## 8. 性能指标与验收标准

| 指标 | 目标值 | 检测方法 |
|------|--------|----------|
| ISP 处理延迟 | < 5 ms（@1080p，硬件 ISP） | 硬件时间戳 |
| 颜色转换延迟 | < 1 ms（GPU / libyuv） | `perf stat` |
| CPU 占用（软件 ISP） | < 15%（单路 1080p30） | `htop` |
| PSNR（畸变校正后） | ≥ 38 dB | 对比参考图 |

---

## 9. 常见问题与排查步骤（Checklist）

- [ ] 图像色彩偏红/绿 → 检查 AWB 参数或 CCM 矩阵
- [ ] 马赛克 / 花屏（RAW 场景）→ 确认 Bayer 模式（RGGB/BGGR）
- [ ] 图像黑边或裁剪异常 → 检查畸变校正 alpha 参数
- [ ] OSD 时间戳不更新 → 检查 OSD 渲染线程是否死锁
- [ ] HDR 合成出现鬼影 → 检查帧间对齐与运动补偿
- [ ] GPU 纹理颜色错误 → 确认 NV12 stride 与宽度对齐（通常 64/128 字节对齐）
- [ ] 色彩空间转换精度问题 → 使用 BT.601 / BT.709 正确系数

---

## 10. 参考资料

- [libyuv 库](https://chromium.googlesource.com/libyuv/libyuv)
- [OpenCV 相机标定](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Jetson ISP / V4L2 ISP API](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/CameraDevelopment.html)
- [NVIDIA VPI（Vision Programming Interface）](https://docs.nvidia.com/vpi/)
