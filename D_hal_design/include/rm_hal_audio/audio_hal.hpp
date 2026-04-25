#pragma once
#include "rm_hal_audio/audio_types.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include <functional>
#include <optional>

namespace rm::hal::sensor {

/// HAL interface for microphone array audio capture.
///
/// Target hardware:
///   ReSpeaker 4-Mic / 6-Mic (USB audio + USB HID DOA)
///   Generic ALSA microphone (no DOA capability)
///
/// Lifecycle:
///   Closed ──configure()──► Configured ──open()──► Opened ──startStreaming()──► Streaming
///     ▲                                                                           │
///     └──────────────────────── close() ─────────────────────────────────────────┘
///   Faulted ◄── fault ◄── (any state)    Closed ◄── reset() ◄── Faulted
class IAudioHAL : public rm::hal::ISensorHAL {
public:
    virtual bool configure(const AudioConfig& config) = 0;
    // open() / close() / isOpen() / deviceId() / health() / reset() from ISensorHAL

    // ── Streaming control ─────────────────────────────────────────────────────

    /// Start the ALSA capture loop (launches internal thread).
    /// Overrides ISensorHAL::startStreaming().
    virtual bool startStreaming() override = 0;
    /// Stop the ALSA capture loop.
    /// Overrides ISensorHAL::stopStreaming().
    virtual bool stopStreaming()  override = 0;

    // ── Callback API ──────────────────────────────────────────────────────────

    /// Callback fired once per ALSA period (typically ~64 ms at 16 kHz / 1024 frames).
    using AudioCallback = std::function<void(const AudioFrame&)>;
    virtual void setAudioCallback(AudioCallback cb) = 0;

    /// Callback fired whenever a new DOA estimate is computed (if enable_doa = true).
    using DOACallback = std::function<void(const DOAResult&)>;
    virtual void setDOACallback(DOACallback cb) = 0;

    /// Synchronous getter for the most recent DOA estimate.
    /// Returns nullopt if enable_doa = false or no estimate is available yet.
    virtual std::optional<DOAResult> getLatestDOA() const = 0;

    /// Returns true when the device can estimate elevation angle (3D DOA).
    /// ReSpeaker 4-Mic / 6-Mic circular arrays provide azimuth only and always
    /// return false.  Returns false if enable_doa = false or the device is not open.
    /// When true, DOAResult::elevation_valid will be set in every delivered result.
    virtual bool hasElevationCapability() const = 0;
};

// ── Factory ───────────────────────────────────────────────────────────────────

using AudioFactory = rm::hal::HALFactory<IAudioHAL>;

/// Registration helper. Typical usage:
///   REGISTER_AUDIO_HAL("respeaker", RespeakerAudioHAL)
///   REGISTER_AUDIO_HAL("sim",       SimAudioHAL)
#define REGISTER_AUDIO_HAL(type_name, Impl) \
    REGISTER_HAL(rm::hal::sensor::AudioFactory, type_name, Impl)

}  // namespace rm::hal::sensor
