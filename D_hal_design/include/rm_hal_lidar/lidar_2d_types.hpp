#pragma once
#include "rm_hal_common/sensor_timestamp.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace rm::hal::sensor {

/// Scan mode hint for 2D LiDAR devices that support multiple modes.
enum class LidarScanMode : int {
    Standard = 0,   ///< Normal scan rate / resolution
    Express  = 1,   ///< High speed at reduced resolution
    Boost    = 2,   ///< Enhanced scan (device-specific)
};

/// Configuration for a 2D laser scanner (BlueSea UDP protocol).
struct Lidar2DConfig {
    std::string device_id;

    // ── Network ───────────────────────────────────────────────────────────────
    std::string host;
    int         port         = 6543;          ///< BlueSea default UDP port

    // ── Scan geometry ─────────────────────────────────────────────────────────
    double angle_min = -M_PI;                 ///< rad
    double angle_max =  M_PI;
    double range_min = 0.1;                   ///< metres
    double range_max = 30.0;                  ///< metres

    // ── Scan parameters ───────────────────────────────────────────────────────
    int           scan_frequency_hz = 10;
    LidarScanMode scan_mode         = LidarScanMode::Standard;
    int           angle_resolution  = 100;    ///< 0.01° units; 100 = 1.00°
    int           rpm               = 600;    ///< Motor speed

    // ── BlueSea protocol flags ────────────────────────────────────────────────
    bool raw_bytes      = true;               ///< Use raw-byte frame parser
    bool with_checksum  = false;              ///< Validate STM32 CRC32 (HDR2/HDR7 variants)
    bool with_intensity = false;              ///< Frames carry per-point intensity byte
    bool output_360     = true;               ///< Assemble full 360° before publishing

    // ── Network tuning ────────────────────────────────────────────────────────
    int recv_buf_size  = 1024 * 1024;
    int udp_timeout_ms = 5000;

    // ── Filtering ─────────────────────────────────────────────────────────────
    bool  enable_intensity_filter = false;
    float min_intensity           = 0.f;
    int   mask                    = 0;         ///< Angular sector bitmask to suppress
    int   error_circle            = 3;
    bool  with_deshadow           = false;
};

/// One complete 360° laser scan — equivalent to ROS sensor_msgs/LaserScan.
///
/// Timestamp change vs Sensor HAL.md V0.3.2:
///   uint64_t timestamp_ns → SensorTimestamp timestamp
///   domain = Hardware when the BlueSea HDR2 device timestamp is available,
///   domain = System otherwise (packet arrival time on steady_clock).
struct LaserScanData {
    rm::hal::SensorTimestamp timestamp;    ///< Start-of-scan timestamp

    // ── Geometry ──────────────────────────────────────────────────────────────
    double angle_min       = 0.0;          ///< rad — start angle of this scan
    double angle_max       = 0.0;          ///< rad — end angle
    double angle_increment = 0.0;          ///< rad per point
    double time_increment  = 0.0;          ///< seconds between adjacent points
    double scan_time       = 0.0;          ///< seconds for a full revolution

    // ── Range ─────────────────────────────────────────────────────────────────
    double range_min = 0.0;                ///< metres — valid range minimum
    double range_max = 0.0;                ///< metres — valid range maximum

    // ── Data ──────────────────────────────────────────────────────────────────
    std::vector<float> ranges;             ///< metres; +inf = no return (< range_min or > range_max)
    std::vector<float> intensities;        ///< Normalised [0,1]; empty if with_intensity = false
};

}  // namespace rm::hal::sensor
