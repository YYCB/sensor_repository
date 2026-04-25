// test_types.cpp
//
// Runtime unit tests for HAL types, free functions, and default values.
// Uses a custom CHECK macro so assertions run in both Debug and Release builds.

#include "rm_hal_audio/audio_types.hpp"
#include "rm_hal_camera/camera_types.hpp"
#include "rm_hal_camera/pixel_encoding.hpp"
#include "rm_hal_camera/stream_type.hpp"
#include "rm_hal_common/error_code.hpp"
#include "rm_hal_common/health_status.hpp"
#include "rm_hal_common/sensor_timestamp.hpp"
#include "rm_hal_imu/imu_types.hpp"
#include "rm_hal_lidar/lidar_3d_types.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_set>

// ── helpers ────────────────────────────────────────────────────────────────────
// CHECK always evaluates cond (unlike assert which is a no-op in Release builds).
#define CHECK(cond) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL  %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        std::exit(1); \
    } \
} while (0)

#define RUN(name) do { \
    std::puts("  " #name " ..."); \
    test_##name(); \
    std::puts("  " #name " OK"); \
} while (0)

// ── SensorTimestamp ────────────────────────────────────────────────────────────
static void test_sensor_timestamp()
{
    using rm::hal::SensorTimestamp;
    using rm::hal::TimestampDomain;

    // Default values
    SensorTimestamp ts;
    CHECK(ts.ns     == 0);
    CHECK(ts.domain == TimestampDomain::System);

    // Comparison operators
    SensorTimestamp a, b;
    a.ns = 1000;
    b.ns = 2000;
    CHECK(a <  b);
    CHECK(b >  a);
    CHECK(a <= b);
    CHECK(b >= a);
    CHECK(a != b);

    SensorTimestamp c;
    c.ns     = 1000;
    c.domain = TimestampDomain::System;
    CHECK(a == c);

    // deltaNs / deltaUs
    CHECK(b.deltaNs(a) ==  1000);
    CHECK(a.deltaNs(b) == -1000);
    CHECK(b.deltaUs(a) ==     1);
    CHECK(a.deltaUs(b) ==    -1);
}

// ── HealthStatus ───────────────────────────────────────────────────────────────
static void test_health_status()
{
    rm::hal::HealthStatus h;
    CHECK(!h.alive);
    CHECK(h.data_rate_hz == 0.0);
    CHECK(h.drop_count   == 0);
    CHECK(h.error_count  == 0);
    CHECK(h.error_msg.empty());
}

// ── bytesPerPixel ──────────────────────────────────────────────────────────────
static void test_bytes_per_pixel()
{
    namespace pe = rm::hal::sensor;
    using pe::PixelEncoding;
    using pe::bytesPerPixel;

    CHECK(bytesPerPixel(PixelEncoding::RGB8)   == 3);
    CHECK(bytesPerPixel(PixelEncoding::BGR8)   == 3);
    CHECK(bytesPerPixel(PixelEncoding::RGBA8)  == 4);
    CHECK(bytesPerPixel(PixelEncoding::BGRA8)  == 4);
    CHECK(bytesPerPixel(PixelEncoding::YUYV)   == 2);
    CHECK(bytesPerPixel(PixelEncoding::UYVY)   == 2);
    CHECK(bytesPerPixel(PixelEncoding::NV12)   == 0);   // planar — no single bpp
    CHECK(bytesPerPixel(PixelEncoding::NV21)   == 0);
    CHECK(bytesPerPixel(PixelEncoding::I420)   == 0);
    CHECK(bytesPerPixel(PixelEncoding::M420)   == 0);
    CHECK(bytesPerPixel(PixelEncoding::MONO8)  == 1);
    CHECK(bytesPerPixel(PixelEncoding::MONO16) == 2);
    CHECK(bytesPerPixel(PixelEncoding::Z16)    == 2);
    CHECK(bytesPerPixel(PixelEncoding::Z32F)   == 4);
    CHECK(bytesPerPixel(PixelEncoding::RAW16)  == 2);
    CHECK(bytesPerPixel(PixelEncoding::MJPEG)  == 0);
    CHECK(bytesPerPixel(PixelEncoding::H264)   == 0);
    CHECK(bytesPerPixel(PixelEncoding::H265)   == 0);
    CHECK(bytesPerPixel(PixelEncoding::CUSTOM) == 0);

    // Aliases must resolve to same bpp as their canonical names
    CHECK(bytesPerPixel(PixelEncoding::Y8)   == bytesPerPixel(PixelEncoding::MONO8));
    CHECK(bytesPerPixel(PixelEncoding::Y16)  == bytesPerPixel(PixelEncoding::MONO16));
    CHECK(bytesPerPixel(PixelEncoding::HEVC) == bytesPerPixel(PixelEncoding::H265));
}

// ── isCompressed ───────────────────────────────────────────────────────────────
static void test_is_compressed()
{
    using rm::hal::sensor::PixelEncoding;
    using rm::hal::sensor::isCompressed;

    CHECK( isCompressed(PixelEncoding::MJPEG));
    CHECK( isCompressed(PixelEncoding::H264));
    CHECK( isCompressed(PixelEncoding::H265));
    CHECK( isCompressed(PixelEncoding::HEVC));   // alias

    CHECK(!isCompressed(PixelEncoding::BGR8));
    CHECK(!isCompressed(PixelEncoding::RGB8));
    CHECK(!isCompressed(PixelEncoding::Z16));
    CHECK(!isCompressed(PixelEncoding::MONO8));
    CHECK(!isCompressed(PixelEncoding::NV12));
}

// ── pixelEncodingToString ──────────────────────────────────────────────────────
static void test_pixel_encoding_to_string()
{
    using rm::hal::sensor::PixelEncoding;
    using rm::hal::sensor::pixelEncodingToString;

    CHECK(std::string(pixelEncodingToString(PixelEncoding::BGR8))   == "BGR8");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::RGB8))   == "RGB8");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::RGBA8))  == "RGBA8");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::BGRA8))  == "BGRA8");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::YUYV))   == "YUYV");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::MONO8))  == "MONO8");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::MONO16)) == "MONO16");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::Z16))    == "Z16");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::Z32F))   == "Z32F");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::MJPEG))  == "MJPEG");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::H264))   == "H264");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::H265))   == "H265");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::RAW16))  == "RAW16");
    CHECK(std::string(pixelEncodingToString(PixelEncoding::CUSTOM)) == "CUSTOM");
}

// ── errorCodeToString ──────────────────────────────────────────────────────────
static void test_error_code_to_string()
{
    using rm::hal::ErrorCode;
    using rm::hal::errorCodeToString;

    CHECK(std::string(errorCodeToString(ErrorCode::OK))                     == "OK");
    CHECK(std::string(errorCodeToString(ErrorCode::UNKNOWN))                == "UNKNOWN");
    CHECK(std::string(errorCodeToString(ErrorCode::NOT_IMPLEMENTED))        == "NOT_IMPLEMENTED");
    CHECK(std::string(errorCodeToString(ErrorCode::DEVICE_NOT_FOUND))       == "DEVICE_NOT_FOUND");
    CHECK(std::string(errorCodeToString(ErrorCode::DEVICE_BUSY))            == "DEVICE_BUSY");
    CHECK(std::string(errorCodeToString(ErrorCode::DEVICE_DISCONNECTED))    == "DEVICE_DISCONNECTED");
    CHECK(std::string(errorCodeToString(ErrorCode::INVALID_STATE))          == "INVALID_STATE");
    CHECK(std::string(errorCodeToString(ErrorCode::ALREADY_OPEN))           == "ALREADY_OPEN");
    CHECK(std::string(errorCodeToString(ErrorCode::NOT_OPEN))               == "NOT_OPEN");
    CHECK(std::string(errorCodeToString(ErrorCode::INVALID_CONFIG))         == "INVALID_CONFIG");
    CHECK(std::string(errorCodeToString(ErrorCode::UNSUPPORTED_FORMAT))     == "UNSUPPORTED_FORMAT");
    CHECK(std::string(errorCodeToString(ErrorCode::UNSUPPORTED_RESOLUTION)) == "UNSUPPORTED_RESOLUTION");
    CHECK(std::string(errorCodeToString(ErrorCode::UNSUPPORTED_FPS))        == "UNSUPPORTED_FPS");
    CHECK(std::string(errorCodeToString(ErrorCode::TIMEOUT))                == "TIMEOUT");
    CHECK(std::string(errorCodeToString(ErrorCode::IO_ERROR))               == "IO_ERROR");
    CHECK(std::string(errorCodeToString(ErrorCode::FRAME_DROPPED))          == "FRAME_DROPPED");
    CHECK(std::string(errorCodeToString(ErrorCode::CRC_ERROR))              == "CRC_ERROR");
    CHECK(std::string(errorCodeToString(ErrorCode::BUFFER_OVERFLOW))        == "BUFFER_OVERFLOW");
    CHECK(std::string(errorCodeToString(ErrorCode::SDK_ERROR))              == "SDK_ERROR");
    CHECK(std::string(errorCodeToString(ErrorCode::SDK_NOT_INITIALIZED))    == "SDK_NOT_INITIALIZED");
    CHECK(std::string(errorCodeToString(ErrorCode::FIRMWARE_MISMATCH))      == "FIRMWARE_MISMATCH");
    CHECK(std::string(errorCodeToString(ErrorCode::PERMISSION_DENIED))      == "PERMISSION_DENIED");
    CHECK(std::string(errorCodeToString(ErrorCode::RESOURCE_EXHAUSTED))     == "RESOURCE_EXHAUSTED");
    // Unknown code falls through to default "UNKNOWN"
    CHECK(std::string(errorCodeToString(static_cast<ErrorCode>(9999)))      == "UNKNOWN");
}

// ── Lidar3DConfig::defaultsFor ─────────────────────────────────────────────────
static void test_lidar3d_defaults()
{
    using rm::hal::sensor::LidarModel;
    using rm::hal::sensor::Lidar3DConfig;

    auto vlp16 = Lidar3DConfig::defaultsFor(LidarModel::VLP_16);
    CHECK(vlp16.model     == LidarModel::VLP_16);
    CHECK(vlp16.port      == 2368);
    CHECK(vlp16.range_max == 100.0);

    auto vlp32 = Lidar3DConfig::defaultsFor(LidarModel::VLP_32C);
    CHECK(vlp32.port      == 2368);
    CHECK(vlp32.range_max == 200.0);

    auto hdl64 = Lidar3DConfig::defaultsFor(LidarModel::HDL_64E);
    CHECK(hdl64.port      == 2368);
    CHECK(hdl64.range_max == 120.0);

    auto livox = Lidar3DConfig::defaultsFor(LidarModel::Livox_Mid360);
    CHECK(livox.model     == LidarModel::Livox_Mid360);
    CHECK(livox.port      == 56000);
    CHECK(livox.range_max == 70.0);
}

// ── StreamProfile::partialMatch ────────────────────────────────────────────────
static void test_stream_profile_partial_match()
{
    using rm::hal::sensor::StreamProfile;
    using rm::hal::sensor::StreamIndex;
    using rm::hal::sensor::StreamType;
    using rm::hal::sensor::PixelEncoding;

    // Device profile: 1280×720 @ 30fps BGR8 COLOR
    StreamProfile device;
    device.stream = StreamIndex{StreamType::COLOR, 0};
    device.width  = 1280;
    device.height =  720;
    device.fps    =   30;
    device.format = PixelEncoding::BGR8;

    // Exact match
    StreamProfile exact;
    exact.stream = StreamIndex{StreamType::COLOR, 0};
    exact.width  = 1280;
    exact.height =  720;
    exact.fps    =   30;
    exact.format = PixelEncoding::BGR8;
    CHECK(device.partialMatch(exact));

    // Wildcard stream type (UNKNOWN matches any)
    StreamProfile wildStream;
    wildStream.stream           = StreamIndex{StreamType::UNKNOWN, 0};
    wildStream.width            = 1280;
    wildStream.height           =  720;
    wildStream.fps              =   30;
    wildStream.match_any_format = true;
    CHECK(device.partialMatch(wildStream));

    // Wildcard dimensions (0 = don't care)
    StreamProfile wildDims;
    wildDims.stream           = StreamIndex{StreamType::COLOR, 0};
    wildDims.width            = 0;   // any width
    wildDims.height           = 0;   // any height
    wildDims.fps              = 0;   // any fps
    wildDims.match_any_format = true;
    CHECK(device.partialMatch(wildDims));

    // Width mismatch
    StreamProfile badWidth;
    badWidth.stream = StreamIndex{StreamType::COLOR, 0};
    badWidth.width  = 640;  // different from 1280
    CHECK(!device.partialMatch(badWidth));

    // Format mismatch (match_any_format = false by default)
    StreamProfile badFormat;
    badFormat.stream = StreamIndex{StreamType::COLOR, 0};
    badFormat.width  = 1280;
    badFormat.height =  720;
    badFormat.fps    =   30;
    badFormat.format = PixelEncoding::RGB8;
    CHECK(!device.partialMatch(badFormat));
}

// ── DOAResult defaults ─────────────────────────────────────────────────────────
static void test_doa_result_defaults()
{
    rm::hal::sensor::DOAResult doa;
    CHECK(!doa.elevation_valid);
    CHECK(doa.azimuth_deg   == 0.f);
    CHECK(doa.elevation_deg == 0.f);
    CHECK(doa.confidence    == 0.f);
}

// ── ImuDeviceInfo defaults ─────────────────────────────────────────────────────
static void test_imu_device_info_defaults()
{
    rm::hal::sensor::ImuDeviceInfo info;
    CHECK(info.accel_range == rm::hal::sensor::AccelRange::G8);
    CHECK(info.gyro_range  == rm::hal::sensor::GyroRange::DPS2000);
}

// ── AudioFrame::totalSamples ───────────────────────────────────────────────────
static void test_audio_frame_total_samples()
{
    rm::hal::sensor::AudioFrame af;
    af.frame_count = 1024;
    af.channels    = 4;
    CHECK(af.totalSamples() == 4096u);

    af.frame_count = 0;
    CHECK(af.totalSamples() == 0u);
}

// ── StreamIndex operators ──────────────────────────────────────────────────────
static void test_stream_index_operators()
{
    using rm::hal::sensor::StreamIndex;
    using rm::hal::sensor::StreamType;

    StreamIndex color0{StreamType::COLOR, 0};
    StreamIndex color0b{StreamType::COLOR, 0};
    StreamIndex depth0{StreamType::DEPTH, 0};
    StreamIndex color1{StreamType::COLOR, 1};

    CHECK(color0  == color0b);
    CHECK(color0  != depth0);
    CHECK(color0  <  depth0);    // COLOR(0x00) < DEPTH(0x01)
    CHECK(color0  <  color1);    // same type, lower index
    CHECK(!( depth0 < color0));
}

// ── std::hash<StreamIndex> ─────────────────────────────────────────────────────
static void test_stream_index_hash()
{
    using rm::hal::sensor::StreamIndex;
    using rm::hal::sensor::StreamType;

    // Verify the hash specialisation allows StreamIndex in unordered containers.
    std::unordered_set<StreamIndex> seen;
    seen.insert(StreamIndex{StreamType::COLOR,    0});
    seen.insert(StreamIndex{StreamType::DEPTH,    0});
    seen.insert(StreamIndex{StreamType::IR_LEFT,  0});
    seen.insert(StreamIndex{StreamType::IR_RIGHT, 0});
    seen.insert(StreamIndex{StreamType::COLOR,    1});
    CHECK(seen.size() == 5u);
    CHECK(seen.count(StreamIndex{StreamType::DEPTH, 0}) == 1u);
    CHECK(seen.count(StreamIndex{StreamType::GYRO,  0}) == 0u);
}

// ── main ───────────────────────────────────────────────────────────────────────
int main()
{
    std::puts("=== rm_hal type tests ===");
    RUN(sensor_timestamp);
    RUN(health_status);
    RUN(bytes_per_pixel);
    RUN(is_compressed);
    RUN(pixel_encoding_to_string);
    RUN(error_code_to_string);
    RUN(lidar3d_defaults);
    RUN(stream_profile_partial_match);
    RUN(doa_result_defaults);
    RUN(imu_device_info_defaults);
    RUN(audio_frame_total_samples);
    RUN(stream_index_operators);
    RUN(stream_index_hash);
    std::puts("=== All tests passed ===");
    return 0;
}
