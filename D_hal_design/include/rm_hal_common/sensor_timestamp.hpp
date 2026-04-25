#pragma once
#include <cstdint>

namespace rm::hal {

/// Identifies the clock source of a sensor timestamp.
/// Use this when comparing timestamps across sensor types to decide whether
/// direct subtraction is valid or whether a PTP / clock-domain adjustment is needed.
enum class TimestampDomain : uint8_t {
    Hardware,  ///< Device hardware clock — highest accuracy; requires host PTP sync
    System,    ///< Host steady_clock stamped at packet / frame reception
    Global,    ///< SDK-aligned PTP clock, if the vendor SDK supports it (e.g. Orbbec "global" domain)
};

/// Unified timestamp carried by every sensor data frame.
///
/// Replaces the bare uint64_t timestamp_ns fields that were spread across
/// LaserScanData, ImuData, AudioFrame etc. in Sensor HAL.md V0.3.2.
/// All frame structs use SensorTimestamp so the upper layer (ArcRT message_filter)
/// can inspect the domain before deciding whether timestamps are directly comparable.
struct SensorTimestamp {
    uint64_t        ns     = 0;
    TimestampDomain domain = TimestampDomain::System;

    bool operator<(const SensorTimestamp& o)  const noexcept {
        // Compare ns first; break ties by domain to remain consistent with
        // operator==, which requires both ns and domain to match.
        // Rationale: operator== already compares both fields, so operator< must
        // also use both fields to satisfy strict-weak-ordering: if neither
        // a<b nor b<a holds, then a and b must compare equal (a==b).
        // Without the domain tie-break, two timestamps with identical ns but
        // different domains would be "equivalent" under < yet unequal under ==,
        // violating the invariant and causing undefined behaviour in ordered
        // containers such as std::set and std::map.
        if (ns != o.ns) return ns < o.ns;
        return domain < o.domain;
    }
    bool operator>(const SensorTimestamp& o)  const noexcept { return o < *this; }
    bool operator<=(const SensorTimestamp& o) const noexcept { return !(o < *this); }
    bool operator>=(const SensorTimestamp& o) const noexcept { return !(*this < o); }
    bool operator==(const SensorTimestamp& o) const noexcept {
        return ns == o.ns && domain == o.domain;
    }
    bool operator!=(const SensorTimestamp& o) const noexcept { return !(*this == o); }

    /// Signed nanosecond delta: (this - other).
    /// Only meaningful when both timestamps share the same domain.
    int64_t deltaNs(const SensorTimestamp& o) const noexcept {
        return static_cast<int64_t>(ns) - static_cast<int64_t>(o.ns);
    }

    /// Signed microsecond delta: (this - other).
    /// Only meaningful when both timestamps share the same domain;
    /// calling across different domains produces an unspecified result.
    int64_t deltaUs(const SensorTimestamp& o) const noexcept {
        return deltaNs(o) / 1000;
    }
};

}  // namespace rm::hal
