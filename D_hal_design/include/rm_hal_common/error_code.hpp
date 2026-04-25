#pragma once
#include <cstdint>
#include <string>

namespace rm::hal {

/// Unified HAL error codes.
///
/// Current phase: driver implementations use ErrorCode internally for logging
/// and health().error_msg; public methods still return bool.
/// Future V3.5 migration: bool open() → ErrorCode open().
enum class ErrorCode : int32_t {
    // ── General (0–99) ──────────────────────────────────────────────────────
    OK                     =   0,
    UNKNOWN                =   1,
    NOT_IMPLEMENTED        =   2,

    // ── Device lifecycle (100–199) ──────────────────────────────────────────
    DEVICE_NOT_FOUND       = 100,  ///< Device not found during enumeration / open
    DEVICE_BUSY            = 101,  ///< Device is already opened by another process
    DEVICE_DISCONNECTED    = 102,  ///< Device disconnected while streaming (hot-unplug)
    INVALID_STATE          = 103,  ///< Operation not allowed in current state
    ALREADY_OPEN           = 104,  ///< open() called on an already-open device
    NOT_OPEN               = 105,  ///< Data method called before open()

    // ── Configuration (200–299) ─────────────────────────────────────────────
    INVALID_CONFIG         = 200,  ///< Configuration parameter validation failed
    UNSUPPORTED_FORMAT     = 201,  ///< Requested PixelEncoding not supported by device
    UNSUPPORTED_RESOLUTION = 202,
    UNSUPPORTED_FPS        = 203,

    // ── Data / IO (300–399) ─────────────────────────────────────────────────
    TIMEOUT                = 300,  ///< Data acquisition / command timed out
    IO_ERROR               = 301,  ///< Underlying IO error (serial / USB / network / CAN)
    FRAME_DROPPED          = 302,  ///< Frame lost (ring buffer overflow or SDK drop)
    CRC_ERROR              = 303,  ///< Protocol CRC / checksum validation failed
    BUFFER_OVERFLOW        = 304,  ///< Internal buffer overflow

    // ── SDK / driver (400–499) ──────────────────────────────────────────────
    SDK_ERROR              = 400,  ///< Vendor SDK returned an error (details in error_msg)
    SDK_NOT_INITIALIZED    = 401,
    FIRMWARE_MISMATCH      = 402,  ///< Firmware version incompatible with SDK

    // ── Permissions / resources (500–599) ───────────────────────────────────
    PERMISSION_DENIED      = 500,  ///< Insufficient permissions (e.g. /dev/video* requires root)
    RESOURCE_EXHAUSTED     = 501,  ///< System resource exhausted (fd / memory / GPU)
};

/// Returns a short human-readable label for an ErrorCode. Never returns nullptr.
inline const char* errorCodeToString(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::OK:                     return "OK";
        case ErrorCode::UNKNOWN:                return "UNKNOWN";
        case ErrorCode::NOT_IMPLEMENTED:        return "NOT_IMPLEMENTED";
        case ErrorCode::DEVICE_NOT_FOUND:       return "DEVICE_NOT_FOUND";
        case ErrorCode::DEVICE_BUSY:            return "DEVICE_BUSY";
        case ErrorCode::DEVICE_DISCONNECTED:    return "DEVICE_DISCONNECTED";
        case ErrorCode::INVALID_STATE:          return "INVALID_STATE";
        case ErrorCode::ALREADY_OPEN:           return "ALREADY_OPEN";
        case ErrorCode::NOT_OPEN:               return "NOT_OPEN";
        case ErrorCode::INVALID_CONFIG:         return "INVALID_CONFIG";
        case ErrorCode::UNSUPPORTED_FORMAT:     return "UNSUPPORTED_FORMAT";
        case ErrorCode::UNSUPPORTED_RESOLUTION: return "UNSUPPORTED_RESOLUTION";
        case ErrorCode::UNSUPPORTED_FPS:        return "UNSUPPORTED_FPS";
        case ErrorCode::TIMEOUT:                return "TIMEOUT";
        case ErrorCode::IO_ERROR:               return "IO_ERROR";
        case ErrorCode::FRAME_DROPPED:          return "FRAME_DROPPED";
        case ErrorCode::CRC_ERROR:              return "CRC_ERROR";
        case ErrorCode::BUFFER_OVERFLOW:        return "BUFFER_OVERFLOW";
        case ErrorCode::SDK_ERROR:              return "SDK_ERROR";
        case ErrorCode::SDK_NOT_INITIALIZED:    return "SDK_NOT_INITIALIZED";
        case ErrorCode::FIRMWARE_MISMATCH:      return "FIRMWARE_MISMATCH";
        case ErrorCode::PERMISSION_DENIED:      return "PERMISSION_DENIED";
        case ErrorCode::RESOURCE_EXHAUSTED:     return "RESOURCE_EXHAUSTED";
        default:                                return "UNKNOWN";
    }
}

/// Extended error information.
struct ErrorInfo {
    ErrorCode   code             = ErrorCode::OK;
    std::string message;           ///< Human-readable description
    std::string sdk_error_detail;  ///< Raw vendor SDK error string (optional)
};

}  // namespace rm::hal
