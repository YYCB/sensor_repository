#pragma once
#include "rm_hal_common/hardware_device.hpp"

namespace rm::hal {

/// Common base for all sensor HAL interfaces.
///
/// Extends IHardwareDevice with two concepts:
///
///   1. reset() — formal Faulted ──reset()──► Closed transition.
///      Default implementation: calls close() and propagates its return value.
///      Concrete implementations may override to perform deeper cleanup
///      (e.g. reinitialise SDK internal state, flush buffers).
///
///   2. startStreaming() / stopStreaming() — uniform streaming lifecycle.
///      All sensor types share the same lifecycle skeleton:
///        Closed ──configure()──► Configured ──open()──► Opened
///               ──startStreaming()──► Streaming
///        Streaming ──stopStreaming()──► Opened
///        (any state) ──close()──► Closed
///
///      For devices where data flows immediately after open() (e.g. 2D/3D LiDAR,
///      IMU), the default no-op implementations are sufficient.
///      ICameraHAL and IAudioHAL override these with real start/stop logic.
///
///   Thread safety contract (inherited):
///   • open / close / configure / startStreaming / stopStreaming: caller must not overlap.
///   • health() / isOpen() / deviceId(): safe to call concurrently from any thread.
class ISensorHAL : public IHardwareDevice {
public:
    /// Recover from Faulted state back to Closed.
    /// Default: delegates to close() and propagates its return value.
    virtual bool reset() {
        return close();
    }

    /// Begin data delivery.
    /// For devices where open() already starts data flow (LiDAR, IMU), the
    /// default implementation is a no-op that returns true.
    /// ICameraHAL and IAudioHAL override this to start their capture pipelines.
    virtual bool startStreaming() { return true; }

    /// Stop data delivery without closing the device (returns to Opened state).
    /// The default no-op is sufficient for devices that cannot pause mid-stream.
    virtual bool stopStreaming()  { return true; }
};

}  // namespace rm::hal
