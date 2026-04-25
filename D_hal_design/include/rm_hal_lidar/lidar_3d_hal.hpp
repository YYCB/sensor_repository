#pragma once
#include "rm_hal_lidar/lidar_3d_types.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include <functional>
#include <memory>

namespace rm::hal::sensor {

/// HAL interface for 3D rotating or solid-state LiDAR sensors.
///
/// Target hardware: Velodyne VLP-16, VLP-32C, HDL-64E; Livox Mid-360; etc.
///
/// Rationale for separate interface (split from I2DLidarHAL):
///   2D laser scanners (BlueSea) output polar range scan data (LaserScanData),
///   semantically equivalent to ROS sensor_msgs/LaserScan.
///   3D LiDARs output dense Cartesian point clouds (PointCloudXYZI) with
///   per-point timing offsets (motion-distortion correction), ring IDs,
///   and a completely different network protocol.
///   A shared interface would mandate large numbers of "not applicable" methods,
///   reducing type safety and readability.
///
/// Lifecycle:
///   Closed ──configure()──► Configured ──open()──► Streaming
///   (Data arrives immediately after open(); no explicit startReceiving.)
///   Faulted ◄── fault ◄── (any state)    Closed ◄── reset() ◄── Faulted
class I3DLidarHAL : public rm::hal::ISensorHAL {
public:
    virtual bool configure(const Lidar3DConfig& config) = 0;
    // open() / close() / isOpen() / deviceId() / health() / reset() from ISensorHAL

    // ── Polling API ───────────────────────────────────────────────────────────
    /// Get the most recently assembled point cloud.
    /// Returns false if no scan has completed yet or the device is not open.
    virtual bool getPointCloud(PointCloudXYZI& out) = 0;

    // ── Callback API ──────────────────────────────────────────────────────────
    /// Callback fired once per completed scan packet.
    using PointCloudCallback = std::function<void(std::shared_ptr<const PointCloudXYZI>)>;
    virtual void setPointCloudCallback(PointCloudCallback cb) = 0;
};

// ── Factory ───────────────────────────────────────────────────────────────────

using Lidar3DFactory = rm::hal::HALFactory<I3DLidarHAL>;

/// Registration helper. Typical usage:
///   REGISTER_LIDAR3D_HAL("velodyne", VelodyneLidar3DHAL)
///   REGISTER_LIDAR3D_HAL("livox",    LivoxLidar3DHAL)
///   REGISTER_LIDAR3D_HAL("sim",      SimLidar3DHAL)
#define REGISTER_LIDAR3D_HAL(type_name, Impl) \
    REGISTER_HAL(rm::hal::sensor::Lidar3DFactory, type_name, Impl)

}  // namespace rm::hal::sensor
