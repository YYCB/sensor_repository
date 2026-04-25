#pragma once
#include <cstdint>
#include <string>

namespace rm::hal {

/// Runtime health snapshot of a HAL device.
/// Returned by IHardwareDevice::health().
///
/// Thread safety: the health() method must use internal synchronisation (a
/// mutex or copy-on-write scheme) when writing error_msg, because std::string
/// is not atomically copyable.  Callers receive a VALUE COPY of HealthStatus
/// returned from health(), and may read all fields of that local copy without
/// any external locking — the copy itself is thread-safe once received.
struct HealthStatus {
    bool        alive        = false;  ///< Device is open and actively delivering data
    double      data_rate_hz = 0.0;    ///< Measured output rate (Hz); 0 when not streaming
    std::string error_msg;             ///< Description of the last error; empty = no error
    uint32_t    drop_count   = 0;      ///< Cumulative dropped frames / packets since open()
    uint32_t    error_count  = 0;      ///< Cumulative protocol / CRC / SDK errors since open()
};

}  // namespace rm::hal
