#pragma once
#include "rm_hal_camera/pixel_encoding.hpp"
#include "rm_hal_camera/stream_type.hpp"
#include "rm_hal_camera/sync_manager.hpp"
#include "rm_hal_common/sensor_timestamp.hpp"
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rm::hal::sensor {

// ── Image frame ───────────────────────────────────────────────────────────────

/// A single decoded image frame from a camera stream.
struct ImageFrame {
    PixelEncoding encoding  = PixelEncoding::BGR8;
    int           width     = 0;
    int           height    = 0;
    int           stride    = 0;               ///< Row stride in bytes (≥ width * bytesPerPixel)
    std::vector<uint8_t> data;                 ///< Raw pixel data

    rm::hal::SensorTimestamp timestamp;        ///< Unified timestamp (domain = Hardware / System / Global)
    uint64_t frame_number          = 0;        ///< Monotonic hardware frame counter
    float    actual_exposure_us    = 0.f;      ///< Actual exposure time in μs; 0 = unknown
    float    actual_gain           = 0.f;      ///< Actual gain in dB; 0 = unknown
    bool     auto_exposure_enabled = false;
};

// ── Point cloud (from ICameraHAL::getPointCloud) ─────────────────────────────

/// Camera-derived point cloud (from depth + intrinsics via SDK PointCloudFilter).
/// For 3D LiDAR point clouds see rm_hal_lidar/lidar_3d_types.hpp.
struct PointCloud {
    std::vector<float>   points;      ///< Interleaved [x0,y0,z0, x1,y1,z1, …] in metres
    std::vector<uint8_t> colors;      ///< Optional interleaved [r,g,b, …]; empty = XYZ only
    int                  valid_count = 0;
    rm::hal::SensorTimestamp timestamp;
};

// ── Stream profile ────────────────────────────────────────────────────────────

/// One stream configuration entry returned by getSupportedProfiles().
struct StreamProfile {
    StreamIndex   stream;
    int           width  = 0;
    int           height = 0;
    int           fps    = 0;
    PixelEncoding format = PixelEncoding::BGR8;

    bool operator==(const StreamProfile& o) const noexcept {
        return stream == o.stream && width == o.width && height == o.height
            && fps == o.fps && format == o.format;
    }

    /// Returns true if all non-zero fields in `req` match this profile (0 = wildcard).
    bool partialMatch(const StreamProfile& req) const noexcept {
        if (req.stream.type != StreamType::UNKNOWN && stream != req.stream) return false;
        if (req.width  != 0 && width  != req.width)  return false;
        if (req.height != 0 && height != req.height) return false;
        if (req.fps    != 0 && fps    != req.fps)    return false;
        if (req.format != PixelEncoding::BGR8 && format != req.format) return false;
        return true;
    }
};

// ── Hardware option descriptor ────────────────────────────────────────────────

/// Data type of a hardware option value.
enum class OptionType : uint8_t { Bool, Int, Float, Enum };

/// Full descriptor for a hardware option (exposure, gain, white-balance, sync-mode, …).
/// Returned by getSupportedOptions() and getOptionInfo().
struct OptionInfo {
    std::string name;
    std::string description;                    ///< Human-readable description from SDK
    OptionType  type          = OptionType::Float;
    float       min           = 0.f;
    float       max           = 0.f;
    float       step          = 0.f;
    float       default_value = 0.f;
    bool        is_readonly   = false;
    /// Populated only for OptionType::Enum.
    /// Key = enum label (e.g. "FreeRun"), Value = numeric representation.
    std::map<std::string, float> enum_values;
};

// ── FrameSet — single-device aligned frame group ──────────────────────────────

/// A set of synchronised frames from a single camera device.
/// Delivered via ICameraHAL::setFrameSetCallback().
struct FrameSet {
    std::shared_ptr<const ImageFrame> color;
    std::shared_ptr<const ImageFrame> depth;
    std::shared_ptr<const ImageFrame> ir;        ///< Nullable — present only if IR stream enabled
    rm::hal::SensorTimestamp          timestamp; ///< Representative aligned timestamp
};

// ── Camera configuration enumerations ────────────────────────────────────────

/// Depth-colour alignment mode applied by the SDK or HAL before frame delivery.
enum class AlignMode : uint8_t {
    None,           ///< No alignment — depth and colour are in their native resolution/FOV
    DepthToColor,   ///< Depth frame is warped to match the colour sensor FOV
    ColorToDepth,   ///< Colour frame is warped to match the depth sensor FOV
};

/// Frame aggregation policy for multi-stream capture.
/// Controls which stream combination triggers a FrameSet delivery.
enum class FrameAggregateMode : uint8_t {
    FullFrame,   ///< Deliver FrameSet only when ALL enabled streams have a new frame
    ColorFrame,  ///< Deliver whenever a new colour frame arrives (depth/IR may be stale)
    Any,         ///< Deliver on any new frame from any enabled stream
    Disabled,    ///< Do not aggregate; deliver each stream's frames independently
};

// ── Camera configuration ──────────────────────────────────────────────────────

struct CameraConfig {
    std::string device_id;
    std::string serial_number;

    // ── Colour stream ─────────────────────────────────────────────────────────
    int           width          = 1280;
    int           height         =  720;
    int           fps            =   30;
    PixelEncoding color_encoding = PixelEncoding::BGR8;
    bool          enable_color   = true;

    // ── Depth stream ──────────────────────────────────────────────────────────
    int  depth_width   = 640;
    int  depth_height  = 480;
    int  depth_fps     =  30;
    bool enable_depth  = true;

    // ── IR stream ─────────────────────────────────────────────────────────────
    bool enable_ir = false;

    // ── Streaming behaviour ───────────────────────────────────────────────────
    int                ring_buffer_depth    = 4;
    AlignMode          align_mode           = AlignMode::None;
    FrameAggregateMode frame_aggregate_mode = FrameAggregateMode::Any;

    // ── Synchronisation ───────────────────────────────────────────────────────
    /// Initial hardware sync mode applied during open().
    /// Can be changed at runtime via ICameraHAL::getSyncManager()->setSyncConfig().
    SyncMode sync_mode = SyncMode::FreeRun;

    // ── GMSL-specific ─────────────────────────────────────────────────────────
    std::string usb_port;                         ///< GMSL channel id: "gmsl2-1", "gmsl2-3", …
    bool        enable_gmsl_trigger = false;
    float       gmsl_trigger_fps_hz = 30.0f;      ///< GMSL trigger frequency in Hz

    // ── V4L2-specific ─────────────────────────────────────────────────────────
    std::string v4l2_node;                        ///< e.g. "/dev/video0"

    // ── Driver-specific extra parameters ─────────────────────────────────────
    std::unordered_map<std::string, std::string> extra_params;
};

}  // namespace rm::hal::sensor
