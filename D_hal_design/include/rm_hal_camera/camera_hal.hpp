#pragma once
#include "rm_hal_camera/calibration_types.hpp"
#include "rm_hal_camera/camera_types.hpp"
#include "rm_hal_camera/sync_manager.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rm::hal::sensor {

/// Primary HAL interface for colour / depth / IR cameras.
///
/// Supported adapters (see Sensor HAL.md §2.6):
///   OrbbecCameraHAL (USB)  — OrbbecSDK ob::Pipeline
///   OrbbecGmslCameraHAL   — OrbbecSDK + IGmslTrigger (/dev/camsync)
///   RealsenseCameraHAL    — librealsense2 rs2::pipeline
///   UsbCameraHAL (V4L2)   — generic UVC via V4L2 ioctl
///   SimCameraHAL          — synthetic data for CI / offline dev
///
/// Lifecycle:
///   Closed ──configure()──► Configured ──open()──► Opened ──startStreaming()──► Streaming
///     ▲                                                                              │
///     └──────────────────────── close() ────────────────────────────────────────────┘
///   Faulted ◄── fault ◄── (any state)       Closed ◄── reset() ◄── Faulted
class ICameraHAL : public rm::hal::ISensorHAL {
public:
    // ── Configuration ────────────────────────────────────────────────────────

    /// Configure the camera before opening.
    /// May be called again after close() to reconfigure without re-creating the object.
    virtual bool configure(const CameraConfig& config) = 0;
    // open() / close() / isOpen() / deviceId() / health() / reset() from ISensorHAL

    // ── Streaming control ─────────────────────────────────────────────────────

    virtual bool startStreaming() override = 0;
    virtual bool stopStreaming()  override = 0;

    // ── Polling API ───────────────────────────────────────────────────────────
    // Active when no callback is registered for the corresponding stream.
    // Returns false and sets health().error_msg if a callback is registered.
    //
    // timeout_ms controls blocking behaviour:
    //   0   — non-blocking: return false immediately if no new frame is available.
    //   > 0 — block for up to timeout_ms milliseconds waiting for a new frame;
    //         return false on timeout.
    //  -1   — block indefinitely until a frame arrives (use with caution).
    // The same timeout semantics apply to getDepthFrame, getIRFrame, and getPointCloud.

    virtual bool getColorFrame(ImageFrame& out, int timeout_ms = 0) = 0;
    virtual bool getDepthFrame(ImageFrame& out, int timeout_ms = 0) = 0;
    /// Get the latest IR frame.
    /// For single-IR devices this is the only IR stream.
    /// For stereo-IR devices (e.g. Orbbec Gemini 330) this returns the IR_LEFT
    /// frame only; use setFrameSetCallback() to receive ir_right simultaneously.
    virtual bool getIRFrame   (ImageFrame& out, int timeout_ms = 0) = 0;
    /// Get the latest point cloud (built by SDK PointCloudFilter or HAL).
    virtual bool getPointCloud(PointCloud& out, int timeout_ms = 0) = 0;

    // ── Callback API ──────────────────────────────────────────────────────────
    // Setting a callback disables polling for the corresponding stream.
    // Callbacks are invoked on a dedicated HAL-internal thread.
    // IMPORTANT: do not call any method of the same ICameraHAL instance
    // from inside a callback — doing so risks deadlock.

    using FrameCallback    = std::function<void(std::shared_ptr<const ImageFrame>)>;
    using FrameSetCallback = std::function<void(const FrameSet&)>;

    virtual void setColorCallback   (FrameCallback cb)      = 0;
    virtual void setDepthCallback   (FrameCallback cb)      = 0;
    /// IR callback for single-IR devices, or for the IR_LEFT stream on stereo-IR
    /// devices (e.g. Orbbec Gemini 330).  To receive ir_right as well, register
    /// setFrameSetCallback() which delivers FrameSet::ir_left and ir_right together.
    virtual void setIRCallback      (FrameCallback cb)      = 0;
    /// Aligned frame set (colour + depth + IR from a single device).
    virtual void setFrameSetCallback(FrameSetCallback cb)   = 0;

    // ── Profile discovery ─────────────────────────────────────────────────────

    /// Returns all stream configurations supported by the connected device.
    virtual std::vector<StreamProfile> getSupportedProfiles() const = 0;

    // ── Calibration ───────────────────────────────────────────────────────────

    virtual CameraIntrinsics getIntrinsics(StreamIndex stream) const = 0;
    virtual CameraExtrinsics getExtrinsics(StreamIndex from, StreamIndex to) const = 0;
    /// Retrieve calibration for a camera-embedded IMU stream.
    /// Pass StreamIndex{StreamType::GYRO, 0} or StreamIndex{StreamType::ACCEL, 0}.
    virtual IMUCalibration   getIMUCalibration(StreamIndex imu_stream) const = 0;
    virtual DepthMetadata    getDepthMetadata() const = 0;

    /// Load user-supplied calibration from a YAML file, overriding factory calibration.
    virtual bool        loadUserCalibration(const std::string& yaml_path) = 0;
    /// Export current calibration to a YAML string.
    virtual std::string exportCalibration() const = 0;

    // ── Runtime hardware options ──────────────────────────────────────────────

    /// Get full descriptors for all options the device supports (exposure, gain, WB, …).
    virtual std::vector<OptionInfo> getSupportedOptions() const = 0;
    /// Get the full descriptor for a single option.
    virtual OptionInfo getOptionInfo(const std::string& name) const = 0;
    /// Get the current numeric value of an option.
    virtual float getOption(const std::string& name) const = 0;
    /// Set an option value at runtime.
    virtual bool  setOption(const std::string& name, float value) = 0;

    // ── Hardware synchronisation (single-device) ──────────────────────────────

    /// Get the single-device hardware sync manager.
    /// Returns nullptr if the device does not support hardware sync.
    virtual std::shared_ptr<ISyncManager> getSyncManager() = 0;
};

// ── Factory ───────────────────────────────────────────────────────────────────

using CameraFactory = rm::hal::HALFactory<ICameraHAL>;

/// Registration helper. Typical usage (in driver .cpp):
///   REGISTER_CAMERA_HAL("orbbec",   OrbbecCameraHAL)
///   REGISTER_CAMERA_HAL("sim",      SimCameraHAL)
#define REGISTER_CAMERA_HAL(type_name, Impl) \
    REGISTER_HAL(rm::hal::sensor::CameraFactory, type_name, Impl)

}  // namespace rm::hal::sensor
