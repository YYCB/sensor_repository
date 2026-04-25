#pragma once
#include "rm_hal_camera/stream_type.hpp"
#include <cstdint>

namespace rm::hal::sensor {

// ── Distortion model ─────────────────────────────────────────────────────────

/// Lens distortion model enumeration.
/// Both OrbbecSDK and librealsense2 expose this concept; HAL unifies the names.
enum class DistortionModel : uint8_t {
    None,
    BrownConrady,           ///< 5- or 8-parameter radial + tangential (plumb_bob / opencv); both vendors
    InverseBrownConrady,    ///< RealSense D4xx depth-stream variant
    KannalaBrandt4,         ///< 4-parameter fisheye lens model
};

// ── Camera intrinsics ─────────────────────────────────────────────────────────

/// Monocular camera intrinsic parameters for a single stream.
struct CameraIntrinsics {
    float fx = 0.f, fy = 0.f;      ///< Focal length in pixels
    float cx = 0.f, cy = 0.f;      ///< Principal point in pixels
    int   width  = 0;
    int   height = 0;
    DistortionModel distortion_model = DistortionModel::BrownConrady;
    /// Distortion coefficients: k1,k2,p1,p2,k3[,k4,k5,k6] for BrownConrady,
    /// or k1–k4 for KannalaBrandt4. Unused elements are 0.
    float distortion_coeffs[8] = {};
    bool  valid = false;  ///< False when calibration data is unavailable
};

// ── Camera extrinsics ─────────────────────────────────────────────────────────

/// Rigid-body transform from one camera stream coordinate frame to another.
struct CameraExtrinsics {
    float rotation[9]    = {};  ///< 3×3 row-major rotation matrix
    float translation[3] = {};  ///< Translation vector in metres
    bool  valid          = false;
};

// ── IMU calibration ───────────────────────────────────────────────────────────

/// Axis-level calibration parameters for a camera-embedded IMU stream.
/// Returned by ICameraHAL::getIMUCalibration(StreamIndex{StreamType::GYRO, 0})
/// or ICameraHAL::getIMUCalibration(StreamIndex{StreamType::ACCEL, 0}).
struct IMUCalibration {
    StreamType stream = StreamType::GYRO;   ///< StreamType::GYRO or StreamType::ACCEL
    /// 3×4 row-major calibration matrix: [R_3x3 | b_3x1].
    ///
    /// Correction formula:
    ///   corrected_SI = R * raw_int32 + b
    ///
    /// where R = scale_bias[0..8] (3×3 row-major scale / cross-axis matrix)
    ///       b = scale_bias[9..11] (3×1 bias vector, same SI units as output)
    ///
    /// Concrete example for gyro X-axis (row 0):
    ///   corrected_x = scale_bias[0]*raw_x + scale_bias[1]*raw_y
    ///               + scale_bias[2]*raw_z + scale_bias[9]
    ///
    /// For accelerometers: output is m/s².  For gyroscopes: output is rad/s.
    float scale_bias[12]     = {};
    float noise_variances[3] = {};  ///< Measurement noise variance [x,y,z] (SI²/Hz)
    float bias_variances[3]  = {};  ///< Bias random-walk variance [x,y,z] (SI²·Hz)
    bool  valid              = false;
};

// ── Depth metadata ────────────────────────────────────────────────────────────

/// Depth stream scale and valid range.
/// Returned by ICameraHAL::getDepthMetadata().
struct DepthMetadata {
    float depth_scale      = 0.001f;  ///< 1 LSB → metres (Orbbec default: 0.001)
    float depth_min_meters = 0.1f;
    float depth_max_meters = 10.0f;
    /// False when depth calibration data is unavailable (device not yet opened,
    /// or the SDK did not expose scale information).  When false, depth_scale
    /// retains its default value 0.001 as a best-effort fallback — callers
    /// should log a warning rather than treating the value as authoritative.
    bool  valid            = false;
};

}  // namespace rm::hal::sensor
