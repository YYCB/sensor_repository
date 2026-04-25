#pragma once
#include "rm_hal_common/sensor_timestamp.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace rm::hal::sensor {

/// Multi-return mode — relevant for Velodyne and solid-state LiDAR
/// that can report more than one return per laser pulse.
enum class LidarReturnMode : uint8_t {
    Strongest = 0,  ///< Single return: highest-intensity return
    Last      = 1,  ///< Single return: last-in-time return
    Dual      = 2,  ///< Dual return: both strongest and last
};

/// One point in a 3D LiDAR scan.
///
/// time_offset_s carries the per-point firing time relative to scan start,
/// enabling Motion Distortion Correction (MDC) in the processing pipeline.
struct PointXYZI {
    float x            = 0.f;  ///< metres (right-hand, LiDAR frame)
    float y            = 0.f;  ///< metres
    float z            = 0.f;  ///< metres
    float intensity    = 0.f;  ///< Normalised reflectivity [0, 1]
    float time_offset_s = 0.f; ///< Seconds relative to PointCloudXYZI::timestamp
    uint8_t ring       = 0;    ///< Laser ring / channel index (0-based)
};

/// A complete 3D LiDAR scan packet.
struct PointCloudXYZI {
    std::vector<PointXYZI>   points;
    int                      valid_count    = 0;
    rm::hal::SensorTimestamp timestamp;               ///< Start-of-scan timestamp
    double                   scan_duration_s = 0.0;  ///< Duration of this scan packet (s)
    uint32_t                 sequence        = 0;    ///< Monotonic scan counter
};

/// Known 3D LiDAR hardware models.
/// Used by Lidar3DConfig::model to select the correct packet decoder.
/// Use LidarModel::Custom together with Lidar3DConfig::custom_model_name
/// for any device not listed here.
enum class LidarModel : uint8_t {
    VLP_16       = 0,   ///< Velodyne VLP-16 (Puck)
    VLP_32C      = 1,   ///< Velodyne VLP-32C
    HDL_64E      = 2,   ///< Velodyne HDL-64E
    Livox_Mid360 = 3,   ///< Livox Mid-360
    Sim          = 4,   ///< Synthetic / simulation source
    Custom       = 255, ///< Other device; see Lidar3DConfig::custom_model_name
};


struct Lidar3DConfig {
    std::string device_id;

    // ── Network ───────────────────────────────────────────────────────────────
    std::string host;
    int         port       = 2368;              ///< Velodyne default data port; Livox: 56000

    // ── Return mode ───────────────────────────────────────────────────────────
    LidarReturnMode return_mode = LidarReturnMode::Strongest;

    // ── Scan geometry ─────────────────────────────────────────────────────────
    double range_min  = 0.1;                    ///< metres
    double range_max  = 200.0;                  ///< metres (VLP-32C max: 200 m)
    int    target_fps = 10;                     ///< Desired scans per second (10 or 20)

    // ── Filtering ─────────────────────────────────────────────────────────────
    float min_intensity = 0.f;                  ///< Discard points below this intensity

    // ── Network tuning ────────────────────────────────────────────────────────
    int recv_buf_size  = 2 * 1024 * 1024;       ///< UDP receive buffer (bytes)
    int udp_timeout_ms = 2000;

    // ── Device model ─────────────────────────────────────────────────────────
    /// Hardware model — selects the packet decoder inside the driver.
    LidarModel  model             = LidarModel::VLP_16;
    /// Optional free-form name used when model == LidarModel::Custom.
    std::string custom_model_name;
};

}  // namespace rm::hal::sensor
