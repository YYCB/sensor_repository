// test_types.cpp
//
// Runtime unit tests for HAL types, free functions, and default values.
// Uses a custom CHECK macro so assertions run in both Debug and Release builds.

#include "rm_hal_audio/audio_types.hpp"
#include "rm_hal_camera/calibration_types.hpp"
#include "rm_hal_camera/camera_types.hpp"
#include "rm_hal_camera/pixel_encoding.hpp"
#include "rm_hal_camera/stream_type.hpp"
#include "rm_hal_camera/sync_manager.hpp"
#include "rm_hal_common/error_code.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/health_status.hpp"
#include "rm_hal_common/sensor_timestamp.hpp"
#include "rm_hal_imu/imu_types.hpp"
#include "rm_hal_lidar/lidar_2d_types.hpp"
#include "rm_hal_lidar/lidar_3d_types.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <set>
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

    // Comparison operators — same domain
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

    // Cross-domain ordering consistency (Bug #1 fix validation):
    // Two timestamps with the same ns but different domains must NOT be
    // considered both "equivalent" under < AND "not equal" under ==.
    // After the fix, operator< includes domain as a tie-breaker.
    SensorTimestamp hw, sys;
    hw.ns  = 5000;  hw.domain  = TimestampDomain::Hardware;
    sys.ns = 5000;  sys.domain = TimestampDomain::System;
    CHECK(hw != sys);            // different domains → not equal
    // Exactly one ordering must hold (strict weak ordering)
    CHECK((hw < sys) != (sys < hw));   // exactly one is less-than
    // Transitive: a < hw, hw < b, then a < b
    SensorTimestamp a2; a2.ns = 4999; a2.domain = TimestampDomain::System;
    SensorTimestamp b2; b2.ns = 5001; b2.domain = TimestampDomain::System;
    CHECK(a2 < hw);
    CHECK(hw < b2);

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

// ── SensorTimestamp in std::set (strict-weak-ordering validation) ──────────────
static void test_sensor_timestamp_ordered_set()
{
    using rm::hal::SensorTimestamp;
    using rm::hal::TimestampDomain;

    // std::set requires strict weak ordering. After Bug #1 fix, inserting
    // timestamps with the same ns but different domains must produce 2 distinct
    // elements (not silently collapse to 1 as it would with the old ordering).
    std::set<SensorTimestamp> s;

    SensorTimestamp hw;  hw.ns  = 1000;  hw.domain  = TimestampDomain::Hardware;
    SensorTimestamp sys; sys.ns = 1000;  sys.domain = TimestampDomain::System;
    SensorTimestamp gl;  gl.ns  = 1000;  gl.domain  = TimestampDomain::Global;

    s.insert(hw);
    s.insert(sys);
    s.insert(gl);
    CHECK(s.size() == 3u);   // all three are distinct elements

    // Iterating in sorted order should reflect the domain tie-break ordering.
    // Hardware < System < Global (enum values 0, 1, 2).
    auto it = s.begin();
    CHECK(it->domain == TimestampDomain::Hardware); ++it;
    CHECK(it->domain == TimestampDomain::System);   ++it;
    CHECK(it->domain == TimestampDomain::Global);
}

// ── HALFactory register / create / enumerate ───────────────────────────────────

// A minimal concrete interface used only in this test to avoid depending on
// any real sensor HAL interface (those are pure-virtual and cannot be instantiated).
struct ITestDevice { virtual ~ITestDevice() = default; virtual int id() const = 0; };
struct AlphaDevice : ITestDevice { int id() const override { return 1; } };
struct BetaDevice  : ITestDevice { int id() const override { return 2; } };

static void test_hal_factory_register_create()
{
    using rm::hal::HALFactory;

    // Use a local factory type to isolate this test from real sensor factories.
    HALFactory<ITestDevice>& fac = HALFactory<ITestDevice>::instance();

    fac.registerType("alpha", []() { return std::make_unique<AlphaDevice>(); });
    fac.registerType("beta",  []() { return std::make_unique<BetaDevice>(); });

    // Create registered types
    auto a = fac.create("alpha");
    auto b = fac.create("beta");
    CHECK(a != nullptr);
    CHECK(b != nullptr);
    CHECK(a->id() == 1);
    CHECK(b->id() == 2);

    // Unknown type returns nullptr (no exception)
    auto unknown = fac.create("gamma");
    CHECK(unknown == nullptr);

    // Re-registering with a new creator replaces the old one
    fac.registerType("alpha", []() { return std::make_unique<BetaDevice>(); });
    auto a2 = fac.create("alpha");
    CHECK(a2 != nullptr);
    CHECK(a2->id() == 2);   // now returns BetaDevice
}

static void test_hal_factory_enumerate()
{
    using rm::hal::HALFactory;
    using rm::hal::DeviceInfo;

    HALFactory<ITestDevice>& fac = HALFactory<ITestDevice>::instance();

    // Register an enumerator that returns one DeviceInfo
    fac.registerEnumerator("alpha", []() {
        DeviceInfo d;
        d.type          = "alpha";
        d.serial_number = "SN0001";
        d.name          = "Alpha Device";
        return std::vector<DeviceInfo>{d};
    });

    auto devs = fac.enumerateDevices();
    CHECK(!devs.empty());
    bool found = false;
    for (const auto& d : devs) {
        if (d.serial_number == "SN0001") { found = true; break; }
    }
    CHECK(found);
}

// ── Calibration type defaults ──────────────────────────────────────────────────
static void test_calibration_defaults()
{
    using rm::hal::sensor::CameraIntrinsics;
    using rm::hal::sensor::CameraExtrinsics;
    using rm::hal::sensor::IMUCalibration;
    using rm::hal::sensor::DepthMetadata;
    using rm::hal::sensor::DistortionModel;
    using rm::hal::sensor::StreamType;

    CameraIntrinsics intr;
    CHECK(intr.fx == 0.f && intr.fy == 0.f);
    CHECK(intr.cx == 0.f && intr.cy == 0.f);
    CHECK(intr.width == 0 && intr.height == 0);
    CHECK(intr.distortion_model == DistortionModel::BrownConrady);
    CHECK(!intr.valid);

    CameraExtrinsics extr;
    CHECK(!extr.valid);
    for (float v : extr.rotation)    CHECK(v == 0.f);
    for (float v : extr.translation) CHECK(v == 0.f);

    IMUCalibration imu_cal;
    CHECK(imu_cal.stream == StreamType::GYRO);
    CHECK(!imu_cal.valid);
    for (float v : imu_cal.scale_bias)       CHECK(v == 0.f);
    for (float v : imu_cal.noise_variances)  CHECK(v == 0.f);
    for (float v : imu_cal.bias_variances)   CHECK(v == 0.f);

    DepthMetadata dm;
    CHECK(dm.depth_scale      == 0.001f);
    CHECK(dm.depth_min_meters == 0.1f);
    CHECK(dm.depth_max_meters == 10.0f);
    CHECK(!dm.valid);
}

// ── CameraConfig defaults ──────────────────────────────────────────────────────
static void test_camera_config_defaults()
{
    using rm::hal::sensor::CameraConfig;
    using rm::hal::sensor::AlignMode;
    using rm::hal::sensor::FrameAggregateMode;
    using rm::hal::sensor::SyncMode;
    using rm::hal::sensor::PixelEncoding;

    CameraConfig cfg;
    CHECK(cfg.width         == 1280);
    CHECK(cfg.height        ==  720);
    CHECK(cfg.fps           ==   30);
    CHECK(cfg.color_encoding == PixelEncoding::BGR8);
    CHECK(cfg.enable_color);
    CHECK(cfg.depth_width   ==  640);
    CHECK(cfg.depth_height  ==  480);
    CHECK(cfg.depth_fps     ==   30);
    CHECK(cfg.enable_depth);
    CHECK(!cfg.enable_ir);
    CHECK(cfg.ring_buffer_depth    == 4);
    CHECK(CameraConfig::MIN_RING_BUFFER_DEPTH == 2);
    CHECK(cfg.align_mode           == AlignMode::None);
    CHECK(cfg.frame_aggregate_mode == FrameAggregateMode::Any);
    CHECK(cfg.sync_mode            == SyncMode::FreeRun);
    CHECK(!cfg.enable_gmsl_trigger);
    CHECK(cfg.gmsl_trigger_fps_hz  == 30.0f);
}

// ── SyncConfig defaults ────────────────────────────────────────────────────────
static void test_sync_config_defaults()
{
    using rm::hal::sensor::SyncConfig;
    using rm::hal::sensor::SyncMode;

    SyncConfig cfg;
    CHECK(cfg.mode                   == SyncMode::FreeRun);
    CHECK(cfg.depth_delay_us         == 0);
    CHECK(cfg.color_delay_us         == 0);
    CHECK(cfg.trigger2image_delay_us == 0);
    CHECK(cfg.trigger_out_delay_us   == 0);
    CHECK(!cfg.trigger_out_enabled);
    CHECK(cfg.frames_per_trigger     == 1);
}

// ── ImageFrame / FrameSet defaults ────────────────────────────────────────────
static void test_image_frame_defaults()
{
    using rm::hal::sensor::ImageFrame;
    using rm::hal::sensor::FrameSet;
    using rm::hal::sensor::PixelEncoding;

    ImageFrame f;
    CHECK(f.encoding             == PixelEncoding::BGR8);
    CHECK(f.width                == 0);
    CHECK(f.height               == 0);
    CHECK(f.stride               == 0);
    CHECK(f.data.empty());
    CHECK(f.frame_number         == 0u);
    CHECK(f.actual_exposure_us   == 0.f);
    CHECK(f.actual_gain          == 0.f);
    CHECK(!f.auto_exposure_enabled);

    FrameSet fs;
    CHECK(fs.color    == nullptr);
    CHECK(fs.depth    == nullptr);
    CHECK(fs.ir_left  == nullptr);
    CHECK(fs.ir_right == nullptr);
}

// ── Lidar2DConfig defaults ─────────────────────────────────────────────────────
static void test_lidar2d_defaults()
{
    using rm::hal::sensor::Lidar2DConfig;
    using rm::hal::sensor::LidarScanMode;
    using rm::hal::sensor::LaserScanData;

    Lidar2DConfig cfg;
    CHECK(cfg.port              == 6543);
    CHECK(cfg.range_min         == 0.1);
    CHECK(cfg.range_max         == 30.0);
    CHECK(cfg.scan_frequency_hz == 10);
    CHECK(cfg.scan_mode         == LidarScanMode::Standard);
    CHECK(cfg.angle_resolution_deg == 1.0f);
    CHECK(cfg.rpm               == 600);
    CHECK(cfg.raw_bytes);
    CHECK(!cfg.with_checksum);
    CHECK(!cfg.with_intensity);
    CHECK(cfg.output_360);
    CHECK(cfg.recv_buf_size     == 1024 * 1024);
    CHECK(cfg.udp_timeout_ms    == 5000);
    CHECK(!cfg.enable_intensity_filter);
    CHECK(cfg.min_intensity     == 0.f);
    CHECK(cfg.mask              == 0);
    CHECK(cfg.error_circle      == 3);
    CHECK(!cfg.with_deshadow);

    // LaserScanData defaults
    LaserScanData scan;
    CHECK(scan.angle_min       == 0.0);
    CHECK(scan.angle_max       == 0.0);
    CHECK(scan.angle_increment == 0.0);
    CHECK(scan.scan_time       == 0.0);
    CHECK(scan.range_min       == 0.0);
    CHECK(scan.range_max       == 0.0);
    CHECK(scan.ranges.empty());
    CHECK(scan.intensities.empty());
}

// ── ImuConfig / ImuData defaults ──────────────────────────────────────────────
static void test_imu_defaults()
{
    using rm::hal::sensor::ImuConfig;
    using rm::hal::sensor::ImuData;
    using rm::hal::sensor::AccelRange;
    using rm::hal::sensor::GyroRange;
    using rm::hal::sensor::ImuFusionMode;

    ImuConfig cfg;
    CHECK(cfg.baudrate           == 460800);
    CHECK(cfg.output_rate_hz     == 200);
    CHECK(cfg.accel_range        == AccelRange::G8);
    CHECK(cfg.gyro_range         == GyroRange::DPS2000);
    CHECK(cfg.fusion_mode        == ImuFusionMode::Interpolation);
    CHECK(cfg.enable_accel_correction);
    CHECK(cfg.enable_gyro_correction);
    CHECK(!cfg.enable_mag_correction);

    ImuData d;
    CHECK(d.accel_x == 0.0 && d.accel_y == 0.0 && d.accel_z == 0.0);
    CHECK(d.gyro_x  == 0.0 && d.gyro_y  == 0.0 && d.gyro_z  == 0.0);
    // Quaternion identity: w=1, x=y=z=0
    CHECK(d.quat_w  == 1.0);
    CHECK(d.quat_x  == 0.0 && d.quat_y == 0.0 && d.quat_z == 0.0);
    CHECK(!d.has_orientation);
    CHECK(!d.has_linear_accel);
    CHECK(d.euler_roll  == 0.0);
    CHECK(d.euler_pitch == 0.0);
    CHECK(d.euler_yaw   == 0.0);
}

// ── ErrorInfo defaults ─────────────────────────────────────────────────────────
static void test_error_info_defaults()
{
    rm::hal::ErrorInfo ei;
    CHECK(ei.code == rm::hal::ErrorCode::OK);
    CHECK(ei.message.empty());
    CHECK(ei.sdk_error_detail.empty());
}

// ── DeviceInfo / ConnectionType ────────────────────────────────────────────────
static void test_device_info_defaults()
{
    using rm::hal::DeviceInfo;
    using rm::hal::ConnectionType;

    DeviceInfo d;
    CHECK(d.type.empty());
    CHECK(d.serial_number.empty());
    CHECK(d.name.empty());
    CHECK(d.connection == ConnectionType::Unknown);
    CHECK(d.port.empty());
    CHECK(d.firmware_version.empty());
}

// ── PointXYZI / PointCloudXYZI defaults ──────────────────────────────────────
static void test_pointcloud_defaults()
{
    using rm::hal::sensor::PointXYZI;
    using rm::hal::sensor::PointCloudXYZI;

    PointXYZI p;
    CHECK(p.x            == 0.f);
    CHECK(p.y            == 0.f);
    CHECK(p.z            == 0.f);
    CHECK(p.intensity    == 0.f);
    CHECK(p.time_offset_s == 0.f);
    CHECK(p.ring         == 0u);

    PointCloudXYZI cloud;
    CHECK(cloud.valid_count   == 0);
    CHECK(cloud.scan_duration_s == 0.0);
    CHECK(cloud.sequence      == 0u);
    CHECK(cloud.points.empty());
}

// ── AudioConfig defaults ───────────────────────────────────────────────────────
static void test_audio_config_defaults()
{
    rm::hal::sensor::AudioConfig cfg;
    CHECK(cfg.sample_rate == 16000u);
    CHECK(cfg.channels    == 4u);
    CHECK(cfg.enable_doa);
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
    // ── new tests ──
    RUN(sensor_timestamp_ordered_set);
    RUN(hal_factory_register_create);
    RUN(hal_factory_enumerate);
    RUN(calibration_defaults);
    RUN(camera_config_defaults);
    RUN(sync_config_defaults);
    RUN(image_frame_defaults);
    RUN(lidar2d_defaults);
    RUN(imu_defaults);
    RUN(error_info_defaults);
    RUN(device_info_defaults);
    RUN(pointcloud_defaults);
    RUN(audio_config_defaults);
    std::puts("=== All tests passed ===");
    return 0;
}
