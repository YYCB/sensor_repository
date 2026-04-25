#pragma once
#include "rm_hal_common/health_status.hpp"
#include <string>

namespace rm::hal {

/// Base interface for every hardware device managed by RMOS Sensor HAL.
///
/// Lifecycle (all sensor types share this skeleton):
///
///   Closed ──configure()──► Configured ──open()──► Opened ──[startX()]──► Streaming
///     ▲                                                                        │
///     └──────────────────────── close() ──────────────────────────────────────┘
///   Faulted ◄──── fault ◄──── (any state)
///   Closed  ◄──── reset() ◄── Faulted
///
/// Thread safety contract:
///   • open / close / configure / startX / stopX: caller must not overlap these calls.
///   • health() / isOpen() / deviceId(): safe to call concurrently from any thread.
class IHardwareDevice {
public:
    virtual ~IHardwareDevice() = default;

    /// Activate the hardware device (open USB / serial / socket, start SDK, etc.).
    /// Precondition: configure() has been called; device is in Configured or Closed state.
    virtual bool open() = 0;

    /// Deactivate the device and release all associated resources.
    /// Idempotent: calling close() on an already-closed device must return true.
    virtual bool close() = 0;

    /// Returns true if the device is currently open (Opened or Streaming state).
    /// Thread-safe.
    virtual bool isOpen() const = 0;

    /// Unique device identifier (serial number, port path, or synthetic id for sim).
    /// Returns the same string for the lifetime of the object.
    /// Thread-safe.
    virtual std::string deviceId() const = 0;

    /// Non-blocking health snapshot.
    /// Thread-safe; never throws.
    virtual HealthStatus health() const = 0;
};

}  // namespace rm::hal
