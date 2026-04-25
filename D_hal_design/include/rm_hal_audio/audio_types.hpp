#pragma once
#include "rm_hal_common/sensor_timestamp.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rm::hal::sensor {

/// PCM sample format.
enum class AudioSampleFormat : uint8_t {
    S16_LE = 0,   ///< 16-bit signed little-endian (most common; speech recognition)
    S24_LE = 1,   ///< 24-bit signed LE
    S32_LE = 2,   ///< 32-bit signed LE
    F32_LE = 3,   ///< 32-bit float LE
};

/// One ALSA period of PCM audio samples.
///
/// OWNERSHIP FIX vs Sensor HAL.md V0.3.2:
///   Original: `const int16_t* data` — raw pointer that aliases the ALSA mmap buffer.
///   Problem:  If a callback holds AudioFrame past its call scope, the ALSA buffer
///             is returned to the kernel by snd_pcm_readi() causing a dangling reference.
///   Fix:      `shared_ptr<const vector<uint8_t>>` — the HAL copies raw bytes once before
///             enqueue, then creates a shared_ptr. Callbacks may safely extend lifetime
///             by holding their own copy of the shared_ptr.
///   Interpretation: callers must cast / reinterpret the byte buffer according to
///             AudioFrame::format (e.g. reinterpret_cast<const int16_t*> for S16_LE).
///   Cost:     One memcpy per period (~8 KB for 1024 frames × 4 channels × 2 B).
struct AudioFrame {
    /// PCM samples stored as raw bytes: interleaved channels [ch0_s0, ch1_s0, …]
    /// Interpret according to AudioFrame::format.
    std::shared_ptr<const std::vector<uint8_t>> data;
    size_t            frame_count  = 0;    ///< Samples per channel in this period
    uint8_t           channels     = 0;
    uint32_t          sample_rate  = 0;    ///< Hz
    AudioSampleFormat format       = AudioSampleFormat::S16_LE;
    rm::hal::SensorTimestamp timestamp;
    uint32_t          sequence     = 0;

    /// Convenience: total number of samples across all channels.
    size_t totalSamples() const noexcept { return frame_count * channels; }
};

/// Direction-of-arrival (DOA) result from a microphone array.
struct DOAResult {
    float    azimuth_deg   = 0.f;   ///< 0 = forward-facing direction, clockwise positive
    float    elevation_deg = 0.f;   ///< 0 = horizontal plane
    float    confidence    = 0.f;   ///< [0, 1]
    rm::hal::SensorTimestamp timestamp;
};

/// Audio capture configuration.
struct AudioConfig {
    std::string device_type  = "alsa_generic";  ///< "respeaker_4mic" | "respeaker_6mic" | "alsa_generic"
    std::string device_name  = "default";        ///< ALSA device string, e.g. "plughw:2,0"
    uint32_t    sample_rate  = 16000;            ///< Hz (16000 for speech; 48000 for music)
    uint8_t     channels     = 4;
    AudioSampleFormat format = AudioSampleFormat::S16_LE;
    size_t      period_frames = 1024;            ///< ALSA period size in sample frames
    size_t      buffer_frames = 4096;            ///< ALSA buffer size in sample frames
    bool        enable_doa    = true;            ///< Read DOA from ReSpeaker HID (register 21)
};

}  // namespace rm::hal::sensor
