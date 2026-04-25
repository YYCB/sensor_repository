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
/// Returned by ICameraHAL::getIMUCalibration(StreamType::GYRO | ACCEL).
struct IMUCalibration {
    StreamType stream = StreamType::GYRO;   ///< StreamType::GYRO or StreamType::ACCEL
    /// 3×4 row-major matrix: [scale_3x3 | bias_3x1].
    /// Multiply raw int32 reading by this to get corrected SI output.
    float scale_bias[12]     = {};
    float noise_variances[3] = {};  ///< Measurement noise variance [x,y,z]
    float bias_variances[3]  = {};  ///< Bias random-walk variance [x,y,z]
    bool  valid              = false;
};

// ── Depth metadata ────────────────────────────────────────────────────────────

/// Depth stream scale and valid range.
/// Returned by ICameraHAL::getDepthMetadata().
struct DepthMetadata {
    float depth_scale      = 0.001f;  ///< 1 LSB → metres (Orbbec default: 0.001)
    float depth_min_meters = 0.1f;
    float depth_max_meters = 10.0f;
};

}  // namespace rm::hal::sensor
