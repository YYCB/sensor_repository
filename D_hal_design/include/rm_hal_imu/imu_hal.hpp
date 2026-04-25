#pragma once
#include "rm_hal_imu/imu_types.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include <functional>

namespace rm::hal::sensor {

/// HAL interface for a 6-DOF inertial measurement unit.
///
/// Target hardware: YESENSE series (TLV serial protocol, 460800 bps).
///
/// Lifecycle:
///   Closed ──configure()──► Configured ──open()──► Streaming
///   (Serial data arrives immediately after open(); no explicit startStreaming.)
///   Faulted ◄── fault ◄── (any state)    Closed ◄── reset() ◄── Faulted
///
/// Multi-instance:
///   Create independent IImuHAL instances for each physical IMU.
///   Each holds its own file descriptor and internal thread.
///   Example:  body_imu  (/dev/yesenseIMU_body)
///             chassis_imu (/dev/yesenseIMU_chassis)
class IImuHAL : public rm::hal::ISensorHAL {
public:
    virtual bool configure(const ImuConfig& config) = 0;
    // open() / close() / isOpen() / deviceId() / health() / reset() from ISensorHAL

    // ── Polling API ───────────────────────────────────────────────────────────
    /// Get the most recently decoded IMU data.
    /// Returns false if no data has been received yet or the device is not open.
    virtual bool getData(ImuData& out) = 0;

    // ── Callback API ──────────────────────────────────────────────────────────
    /// Callback invoked on every decoded TLV frame (~200 Hz).
    /// Invoked on a HAL-internal serial-reader thread — keep the handler brief.
    using ImuCallback = std::function<void(const ImuData&)>;
    virtual void setDataCallback(ImuCallback cb) = 0;

    // ── Device metadata ───────────────────────────────────────────────────────
    /// Returns device-level noise / calibration parameters for EKF / UKF config.
    virtual ImuDeviceInfo getDeviceInfo() const = 0;

    /// Reset the on-device AHRS orientation estimate (if supported).
    /// Default implementation returns false.
    virtual bool resetOrientation() { return false; }
};

// ── Factory ───────────────────────────────────────────────────────────────────

using ImuFactory = rm::hal::HALFactory<IImuHAL>;

/// Registration helper. Typical usage:
///   REGISTER_IMU_HAL("yesense", YesenseImuHAL)
///   REGISTER_IMU_HAL("sim",     SimImuHAL)
#define REGISTER_IMU_HAL(type_name, Impl) \
    REGISTER_HAL(rm::hal::sensor::ImuFactory, type_name, Impl)

}  // namespace rm::hal::sensor
