#pragma once
#include "rm_hal_common/sensor_timestamp.hpp"
#include <cstdint>
#include <string>

namespace rm::hal::sensor {

// ── Sensor range enums ────────────────────────────────────────────────────────

enum class AccelRange : int {
    G2  =  2,   ///< ±2 g
    G4  =  4,   ///< ±4 g
    G8  =  8,   ///< ±8 g
    G16 = 16,   ///< ±16 g
};

enum class GyroRange : int {
    DPS250  =  250,   ///< ±250 °/s
    DPS500  =  500,
    DPS1000 = 1000,
    DPS2000 = 2000,
};

/// Fusion strategy used when accelerometer and gyroscope run at different rates.
enum class ImuFusionMode : int {
    None          = 0,  ///< Accel and gyro delivered independently (no alignment)
    Copy          = 1,  ///< Latest value is copied to matching timestamp
    Interpolation = 2,  ///< Linear interpolation to align timestamps (recommended)
};

// ── IMU configuration ─────────────────────────────────────────────────────────

struct ImuConfig {
    std::string device_id;

    // ── Serial port (YESENSE) ─────────────────────────────────────────────────
    std::string port;
    int         baudrate       = 460800;

    // ── Sampling ─────────────────────────────────────────────────────────────
    int           output_rate_hz = 200;
    AccelRange    accel_range    = AccelRange::G8;
    GyroRange     gyro_range     = GyroRange::DPS2000;
    ImuFusionMode fusion_mode    = ImuFusionMode::Interpolation;

    // ── Correction switches ───────────────────────────────────────────────────
    bool enable_accel_correction = true;
    bool enable_gyro_correction  = true;
    bool enable_mag_correction   = false;

    // ── Noise density parameters (for EKF / UKF) ─────────────────────────────
    double accel_noise_density = 1e-4;   ///< m/s²/√Hz
    double gyro_noise_density  = 1e-4;   ///< rad/s/√Hz
    double accel_random_walk   = 1e-4;   ///< m/s³/√Hz
    double gyro_random_walk    = 1e-4;   ///< rad/s²/√Hz
};

// ── IMU data frame ────────────────────────────────────────────────────────────

/// One decoded IMU frame from the YESENSE TLV protocol.
///
/// Timestamp change vs Sensor HAL.md V0.3.2:
///   uint64_t timestamp_ns → SensorTimestamp timestamp
///   domain = System (serial port reception time) unless the device provides
///   a hardware timestamp via DataID 0x80 (SAMPLE_TIMESTAMP), in which case
///   domain = Hardware.
struct ImuData {
    rm::hal::SensorTimestamp timestamp;

    // ── Acceleration (m/s²) — DataID 0x10 ACCEL_RAW × 0.000001 × 9.80665 ──────
    double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;

    // ── Angular velocity (rad/s) — DataID 0x20 × 0.000001 × π/180 ──────────────
    double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

    // ── Orientation quaternion — DataID 0x41 × 0.000001 ──────────────────────────
    double quat_w = 1.0, quat_x = 0.0, quat_y = 0.0, quat_z = 0.0;
    bool   has_orientation = false;

    // ── Linear acceleration without gravity — DataID 0x11 ────────────────────────
    double linear_accel_x = 0.0, linear_accel_y = 0.0, linear_accel_z = 0.0;
    bool   has_linear_accel = false;

    // ── Euler angles (rad) — DataID 0x40 × 0.000001 × π/180 ────────────────────
    double euler_roll = 0.0, euler_pitch = 0.0, euler_yaw = 0.0;
    bool   has_euler  = false;

    // ── Magnetometer (μT) — DataID 0x30 × 0.001 ─────────────────────────────────
    double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
    bool   has_magnetometer = false;

    // ── Temperature (°C) — DataID 0x01 × 0.01 ───────────────────────────────────
    double temperature     = 0.0;
    bool   has_temperature = false;

    // ── Data quality ──────────────────────────────────────────────────────────────
    bool     is_calibrated       = true;
    uint32_t sequence            = 0;    ///< Monotonic counter
    uint16_t status_word         = 0;    ///< DataID 0x70 — device status bit field
    uint32_t sample_timestamp_ms = 0;    ///< DataID 0x80 — device-side sample time (ms)
};

// ── IMU device information ────────────────────────────────────────────────────

/// Noise and calibration metadata — used to configure an EKF / UKF.
struct ImuDeviceInfo {
    AccelRange accel_range;
    GyroRange  gyro_range;
    double     accel_noise_density      = 1e-4;
    double     gyro_noise_density       = 1e-4;
    double     accel_random_walk        = 1e-4;
    double     gyro_random_walk         = 1e-4;
    double     reference_temperature_c  = 25.0;  ///< Temperature at calibration time
};

}  // namespace rm::hal::sensor
