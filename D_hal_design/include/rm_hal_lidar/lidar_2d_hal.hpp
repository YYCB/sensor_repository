#pragma once
#include "rm_hal_lidar/lidar_2d_types.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include <functional>
#include <memory>

namespace rm::hal::sensor {

/// HAL interface for 2D rotating laser scanners (BlueSea LDS-U50C-S / LDS-U80C-S).
///
/// Output: LaserScanData — equivalent to ROS sensor_msgs/LaserScan.
/// Use I3DLidarHAL for 3D rotating/solid-state LiDAR (Velodyne, Livox).
///
/// Lifecycle:
///   Closed ──configure()──► Configured ──open()──► Streaming
///   (UDP data arrives immediately after open(); no explicit startReceiving.)
///   Faulted ◄── fault ◄── (any state)    Closed ◄── reset() ◄── Faulted
class I2DLidarHAL : public rm::hal::ISensorHAL {
public:
    virtual bool configure(const Lidar2DConfig& config) = 0;
    // open() / close() / isOpen() / deviceId() / health() / reset() from ISensorHAL

    // ── Polling API ───────────────────────────────────────────────────────────
    /// Get the most recently assembled 360° scan.
    /// Returns false if no scan has been completed yet or the device is not open.
    virtual bool getScan(LaserScanData& out) = 0;

    // ── Callback API ──────────────────────────────────────────────────────────
    /// Callback fired once per complete 360° scan.
    using ScanCallback = std::function<void(std::shared_ptr<const LaserScanData>)>;
    virtual void setScanCallback(ScanCallback cb) = 0;

    // ── Runtime control ───────────────────────────────────────────────────────
    /// Adjust motor speed / scan frequency at runtime (device-dependent).
    /// Default implementation returns false (not supported).
    virtual bool setScanFrequency(int hz) { (void)hz; return false; }
};

// ── Factory ───────────────────────────────────────────────────────────────────

using Lidar2DFactory = rm::hal::HALFactory<I2DLidarHAL>;

/// Registration helper. Typical usage:
///   REGISTER_LIDAR2D_HAL("bluesea", BlueseaLidar2DHAL)
///   REGISTER_LIDAR2D_HAL("sim",     SimLidar2DHAL)
#define REGISTER_LIDAR2D_HAL(type_name, Impl) \
    REGISTER_HAL(rm::hal::sensor::Lidar2DFactory, type_name, Impl)

}  // namespace rm::hal::sensor
