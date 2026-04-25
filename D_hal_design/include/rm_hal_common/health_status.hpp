#pragma once
#include <cstdint>
#include <string>

namespace rm::hal {

/// Runtime health snapshot of a HAL device.
/// Returned by IHardwareDevice::health().
/// All fields are independently atomic/trivially readable — safe to call from any thread.
struct HealthStatus {
    bool        alive        = false;  ///< Device is open and actively delivering data
    double      data_rate_hz = 0.0;    ///< Measured output rate (Hz); 0 when not streaming
    std::string error_msg;             ///< Description of the last error; empty = no error
    uint32_t    drop_count   = 0;      ///< Cumulative dropped frames / packets since open()
    uint32_t    error_count  = 0;      ///< Cumulative protocol / CRC / SDK errors since open()
};

}  // namespace rm::hal
