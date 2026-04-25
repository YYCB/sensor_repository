#pragma once
#include "rm_hal_common/hardware_device.hpp"

namespace rm::hal {

/// Common base for all sensor HAL interfaces.
///
/// Adds reset() to the IHardwareDevice lifecycle.
/// reset() is the formal declaration corresponding to the
///   Faulted ──reset()──► Closed
/// transition that appears in every state-machine diagram in Sensor HAL.md but
/// was missing from any interface definition in V0.3.2.
///
/// Default implementation: calls close() and returns true.
/// Concrete implementations may override to perform deeper cleanup
/// (e.g. reinitialise SDK internal state, flush buffers).
class ISensorHAL : public IHardwareDevice {
public:
    /// Recover from Faulted state back to Closed.
    /// Default: delegates to close() and propagates its return value.
    virtual bool reset() {
        return close();
    }
};

}  // namespace rm::hal
