#pragma once
#include <cstdint>
#include <vector>

namespace rm::hal::sensor {

/// Hardware synchronisation mode of a single camera device.
/// Maps to Orbbec OBMultiDeviceSyncMode and RealSense inter_cam_sync_mode.
enum class SyncMode : uint8_t {
    FreeRun,          ///< Default: device runs at its own internal rate
    Standalone,       ///< Ignores external trigger; does not output a trigger signal
    Primary,          ///< Generates a trigger signal to drive Secondary devices
    Secondary,        ///< Passively receives trigger from Primary; may have frame drops
    SecondarySynced,  ///< Secondary with guaranteed synchronised output
    SoftwareTrigger,  ///< Single-shot trigger via ISyncManager::triggerOnce()
    HardwareTrigger,  ///< External GPIO / GMSL PWM trigger (pairs with /dev/camsync)
};

/// Fine-grained timing parameters for hardware synchronisation.
/// Not all fields are honoured by every vendor SDK or device model.
struct SyncConfig {
    SyncMode mode                   = SyncMode::FreeRun;
    int      depth_delay_us         = 0;   ///< Depth stream trigger delay (μs)
    int      color_delay_us         = 0;   ///< Colour stream trigger delay (μs)
    int      trigger2image_delay_us = 0;   ///< Trigger-signal to first-pixel delay (μs)
    int      trigger_out_delay_us   = 0;   ///< Output trigger pin delay in Primary mode (μs)
    bool     trigger_out_enabled    = false; ///< Whether to drive the output trigger pin (Primary only)
    int      frames_per_trigger     = 1;   ///< Frames generated per trigger pulse
};

/// Single-device hardware synchronisation manager.
///
/// Obtained via ICameraHAL::getSyncManager().
/// Returns nullptr if the device does not support hardware sync.
///
/// Layer attribution: ISyncManager is HAL-layer (single device hardware config).
/// It is distinct from ISyncCoordinator (Module layer, multi-device orchestration).
class ISyncManager {
public:
    virtual ~ISyncManager() = default;

    /// List the synchronisation modes supported by this device.
    virtual std::vector<SyncMode> getSupportedSyncModes() const = 0;

    /// Apply a synchronisation configuration.
    /// Some devices require the stream to be stopped before changes take effect.
    virtual bool setSyncConfig(const SyncConfig& config) = 0;

    /// Read the currently active synchronisation configuration.
    virtual SyncConfig getSyncConfig() const = 0;

    /// Fire a software trigger — valid only in SyncMode::SoftwareTrigger.
    /// Maps to Orbbec device->triggerCapture().
    virtual bool triggerOnce() = 0;

    /// Returns true when the device (in Secondary mode) has received a trigger
    /// from the Primary and is producing synchronised output.
    virtual bool isSynced() const = 0;
};

}  // namespace rm::hal::sensor
