#pragma once
#include <cstdint>
#include <string>

namespace rm::hal {

/// Runtime health snapshot of a HAL device.
/// Returned by IHardwareDevice::health().
///
/// Thread safety: health() itself must be callable from any thread (the virtual
/// method is declared thread-safe in IHardwareDevice).  However, the individual
/// fields are NOT independently atomic — in particular, std::string error_msg
/// requires a mutex or copy-on-write scheme inside the implementation.
/// Callers receive a VALUE COPY of HealthStatus returned from health(); reading
/// that local copy is safe.  Never read fields of a HealthStatus reference
/// obtained across threads without external synchronisation.
struct HealthStatus {
    bool        alive        = false;  ///< Device is open and actively delivering data
    double      data_rate_hz = 0.0;    ///< Measured output rate (Hz); 0 when not streaming
    std::string error_msg;             ///< Description of the last error; empty = no error
    uint32_t    drop_count   = 0;      ///< Cumulative dropped frames / packets since open()
    uint32_t    error_count  = 0;      ///< Cumulative protocol / CRC / SDK errors since open()
};

}  // namespace rm::hal
