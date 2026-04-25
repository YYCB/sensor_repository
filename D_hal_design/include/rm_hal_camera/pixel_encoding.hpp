#pragma once
#include <cstdint>

namespace rm::hal::sensor {

/// Pixel encoding / format for an image frame.
/// Numeric values are stable — do not reorder or renumber existing entries.
enum class PixelEncoding : uint8_t {
    // ── Colour (0x00–0x1F) ──────────────────────────────────────────────────
    RGB8   = 0x00,   ///< 24-bit RGB packed
    BGR8   = 0x01,   ///< 24-bit BGR packed (OpenCV default)
    RGBA8  = 0x02,
    BGRA8  = 0x03,
    YUYV   = 0x04,   ///< YUV422 packed: Y0 U0 Y1 V0
    UYVY   = 0x05,   ///< YUV422 packed: U0 Y0 V0 Y1
    NV12   = 0x06,   ///< YUV420 semi-planar (Y plane + interleaved UV plane)
    NV21   = 0x07,   ///< YUV420 semi-planar (Y plane + interleaved VU plane)
    I420   = 0x08,   ///< YUV420 fully planar
    M420   = 0x09,   ///< YUV420 variant

    // ── Grayscale (0x20–0x2F) ───────────────────────────────────────────────
    MONO8  = 0x20,   ///< 8-bit luminance (IR, greyscale)
    MONO16 = 0x21,   ///< 16-bit luminance

    // ── Depth (0x30–0x3F) ───────────────────────────────────────────────────
    Z16    = 0x30,   ///< 16-bit depth in mm (default Orbbec / RealSense depth format)
    Z32F   = 0x31,   ///< 32-bit float depth in metres

    // ── Compressed (0x40–0x4F) ──────────────────────────────────────────────
    MJPEG  = 0x40,
    H264   = 0x41,
    H265   = 0x42,
    HEVC   = H265,   ///< Alias for H265

    // ── IR alias names used by some vendor SDKs ──────────────────────────────
    Y8     = MONO8,   ///< Alias for MONO8  (OB_FORMAT_Y8 maps to 8-bit luminance)
    Y16    = MONO16,  ///< Alias for MONO16 (OB_FORMAT_Y16 maps to 16-bit luminance)

    // ── Raw / custom (0xF0–0xFF) ─────────────────────────────────────────────
    RAW16  = 0xF0,   ///< Bayer-pattern 16-bit raw
    CUSTOM = 0xFF,
};

/// Returns the number of bytes per pixel for packed formats.
/// Returns 0 for compressed or variable-length formats (MJPEG, H264, H265),
/// and for planar / semi-planar YUV formats (NV12, NV21, I420, M420) where
/// stride calculation requires knowledge of the plane layout.
inline int bytesPerPixel(PixelEncoding enc) noexcept {
    switch (enc) {
        case PixelEncoding::RGB8:   return 3;
        case PixelEncoding::BGR8:   return 3;
        case PixelEncoding::RGBA8:  return 4;
        case PixelEncoding::BGRA8:  return 4;
        case PixelEncoding::YUYV:   return 2;
        case PixelEncoding::UYVY:   return 2;
        case PixelEncoding::NV12:   return 0;  // planar; use height * stride * 3/2
        case PixelEncoding::NV21:   return 0;
        case PixelEncoding::I420:   return 0;
        case PixelEncoding::M420:   return 0;
        case PixelEncoding::MONO8:  return 1;
        case PixelEncoding::MONO16: return 2;
        case PixelEncoding::Z16:    return 2;
        case PixelEncoding::Z32F:   return 4;
        case PixelEncoding::RAW16:  return 2;
        case PixelEncoding::MJPEG:  return 0;
        case PixelEncoding::H264:   return 0;
        case PixelEncoding::H265:   return 0;
        case PixelEncoding::CUSTOM: return 0;
        default:                    return 0;
    }
}

/// Returns true for formats that require a software or hardware decoder
/// before individual pixel access is possible.
inline bool isCompressed(PixelEncoding enc) noexcept {
    return enc == PixelEncoding::MJPEG
        || enc == PixelEncoding::H264
        || enc == PixelEncoding::H265;  // H265 == HEVC alias
}

/// Returns a short ASCII label (e.g. "BGR8", "Z16", "MJPEG"). Never returns nullptr.
inline const char* pixelEncodingToString(PixelEncoding enc) noexcept {
    switch (enc) {
        case PixelEncoding::RGB8:   return "RGB8";
        case PixelEncoding::BGR8:   return "BGR8";
        case PixelEncoding::RGBA8:  return "RGBA8";
        case PixelEncoding::BGRA8:  return "BGRA8";
        case PixelEncoding::YUYV:   return "YUYV";
        case PixelEncoding::UYVY:   return "UYVY";
        case PixelEncoding::NV12:   return "NV12";
        case PixelEncoding::NV21:   return "NV21";
        case PixelEncoding::I420:   return "I420";
        case PixelEncoding::M420:   return "M420";
        case PixelEncoding::MONO8:  return "MONO8";
        case PixelEncoding::MONO16: return "MONO16";
        case PixelEncoding::Z16:    return "Z16";
        case PixelEncoding::Z32F:   return "Z32F";
        case PixelEncoding::MJPEG:  return "MJPEG";
        case PixelEncoding::H264:   return "H264";
        case PixelEncoding::H265:   return "H265";
        case PixelEncoding::RAW16:  return "RAW16";
        case PixelEncoding::CUSTOM: return "CUSTOM";
        default:                    return "UNKNOWN";
    }
}

}  // namespace rm::hal::sensor
