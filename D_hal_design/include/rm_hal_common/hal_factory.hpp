#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rm::hal {

/// Device discovery metadata; returned by HALFactory::enumerateDevices().
struct DeviceInfo {
    std::string type;              ///< Factory registration name ("orbbec", "bluesea", "sim", …)
    std::string serial_number;     ///< Device serial number
    std::string name;              ///< Human-readable model name (e.g. "Gemini 330", "VLP-16")
    std::string connection;        ///< "usb" | "gmsl2" | "ethernet" | "serial" | "sim"
    std::string port;              ///< Physical port identifier ("gmsl2-1", "/dev/video0", …)
    std::string firmware_version;
};

using DeviceChangedCallback = std::function<void(
    const std::vector<DeviceInfo>& added,
    const std::vector<DeviceInfo>& removed)>;

/// Type-parameterised HAL factory with hot-plug support.
///
/// Specialise as CameraFactory / Lidar2DFactory / Lidar3DFactory / ImuFactory / AudioFactory
/// using the aliases defined at the bottom of each HAL header.
///
/// Registration (typically at static-init time):
///   REGISTER_HAL(CameraFactory, "orbbec", OrbbecCameraHAL)
///
/// Usage:
///   auto cam = CameraFactory::instance().create("orbbec");
///
/// Thread safety: registerType(), registerEnumerator(), create(), and
/// enumerateDevices() are all protected by an internal mutex and may be
/// called concurrently from multiple threads.
template<typename Interface>
class HALFactory {
public:
    using Creator    = std::function<std::unique_ptr<Interface>()>;
    using Enumerator = std::function<std::vector<DeviceInfo>()>;

    static HALFactory& instance() {
        static HALFactory inst;
        return inst;
    }

    /// Register a driver constructor under a string key.
    void registerType(const std::string& type_name, Creator creator) {
        std::lock_guard<std::mutex> lock(mutex_);
        creators_[type_name] = std::move(creator);
    }

    /// Register a static device enumerator for a driver type (optional).
    void registerEnumerator(const std::string& type_name, Enumerator enumerator) {
        std::lock_guard<std::mutex> lock(mutex_);
        enumerators_[type_name] = std::move(enumerator);
    }

    /// Create a new driver instance for the given type key.
    /// Returns nullptr if the key is not registered.
    std::unique_ptr<Interface> create(const std::string& type_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = creators_.find(type_name);
        return (it != creators_.end()) ? it->second() : nullptr;
    }

    /// Enumerate all available devices across every registered driver type.
    std::vector<DeviceInfo> enumerateDevices() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<DeviceInfo> all;
        for (const auto& [name, fn] : enumerators_) {
            auto devs = fn();
            all.insert(all.end(), devs.begin(), devs.end());
        }
        return all;
    }

    /// Register a callback for hot-plug device add / remove events.
    /// NOTE: Hot-plug event delivery is NOT YET IMPLEMENTED (reserved for V2.0).
    /// The callback is stored but never invoked by this version of the factory.
    void setDeviceChangedCallback(DeviceChangedCallback cb) {
        std::lock_guard<std::mutex> lock(mutex_);
        device_changed_cb_ = std::move(cb);
    }

    /// Enable POSIX shared-memory process mutex to prevent concurrent device
    /// access from multiple processes (e.g. ROS composable container + debug tool).
    /// NOTE: NOT YET IMPLEMENTED (reserved for V2.0).
    void enableProcessLock(const std::string& lock_name = "rmos_hal_lock") {
        std::lock_guard<std::mutex> lock(mutex_);
        process_lock_name_    = lock_name;
        process_lock_enabled_ = true;
    }

private:
    HALFactory() = default;
    HALFactory(const HALFactory&) = delete;
    HALFactory& operator=(const HALFactory&) = delete;

    mutable std::mutex                          mutex_;
    std::unordered_map<std::string, Creator>    creators_;
    std::unordered_map<std::string, Enumerator> enumerators_;
    DeviceChangedCallback                       device_changed_cb_;
    bool                                        process_lock_enabled_ = false;
    std::string                                 process_lock_name_;
};

}  // namespace rm::hal

// ── Registration macro ────────────────────────────────────────────────────────
// Usage (at namespace scope in a .cpp or header):
//   REGISTER_HAL(rm::hal::sensor::CameraFactory, "orbbec", OrbbecCameraHAL)
//
// The macro creates a function-local static that triggers exactly once at
// program startup to register the driver with the factory singleton.
#define REGISTER_HAL(Factory, type_name, Impl)                              \
    static const bool _hal_reg_##Impl = []() {                              \
        Factory::instance().registerType(                                   \
            type_name, []() { return std::make_unique<Impl>(); });          \
        return true;                                                        \
    }()
