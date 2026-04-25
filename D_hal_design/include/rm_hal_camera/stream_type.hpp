#pragma once
#include <cstdint>

namespace rm::hal::sensor {

/// Logical stream type within a camera or IMU device.
/// Used as the primary key in StreamIndex and as a parameter to
/// ICameraHAL::getIntrinsics() / getExtrinsics() / getIMUCalibration().
///
/// Note: LiDAR-specific stream types are NOT included here.
/// I2DLidarHAL / I3DLidarHAL use their own data types (LaserScanData / PointCloudXYZI)
/// and do not share this enumeration.
enum class StreamType : uint8_t {
    // ── Image streams ────────────────────────────────────────────────────────
    COLOR      = 0x00,
    DEPTH      = 0x01,
    IR_LEFT    = 0x02,
    IR_RIGHT   = 0x03,
    FISHEYE    = 0x04,

    // ── IMU streams (camera-embedded IMU) ────────────────────────────────────
    GYRO       = 0x10,  ///< Gyroscope
    ACCEL      = 0x11,  ///< Accelerometer
    MOTION     = 0x12,  ///< Combined motion stream (some SDKs expose a single merged stream)

    UNKNOWN    = 0xFF,
};

/// Identifies a specific stream instance within a device.
/// index = 0 for the default single stream.
/// index > 0 for additional streams of the same type
/// (e.g. IR_LEFT(0) / IR_RIGHT(0), COLOR(0) / COLOR(1) on a stereo camera).
struct StreamIndex {
    StreamType type  = StreamType::UNKNOWN;
    int        index = 0;

    bool operator==(const StreamIndex& o) const noexcept {
        return type == o.type && index == o.index;
    }
    bool operator!=(const StreamIndex& o) const noexcept { return !(*this == o); }
    bool operator<(const StreamIndex& o)  const noexcept {
        return type < o.type || (type == o.type && index < o.index);
    }
};

}  // namespace rm::hal::sensor
