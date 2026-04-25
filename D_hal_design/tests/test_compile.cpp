// test_compile.cpp
//
// Includes every public header in the rm_hal_headers interface library to verify
// they all compile cleanly together in a single translation unit.
// No runtime logic — the test passes if compilation and linking succeed.

#include "rm_hal_audio/audio_hal.hpp"
#include "rm_hal_audio/audio_types.hpp"
#include "rm_hal_camera/calibration_types.hpp"
#include "rm_hal_camera/camera_hal.hpp"
#include "rm_hal_camera/camera_types.hpp"
#include "rm_hal_camera/pixel_encoding.hpp"
#include "rm_hal_camera/stream_type.hpp"
#include "rm_hal_camera/sync_manager.hpp"
#include "rm_hal_common/error_code.hpp"
#include "rm_hal_common/hal_factory.hpp"
#include "rm_hal_common/hardware_device.hpp"
#include "rm_hal_common/health_status.hpp"
#include "rm_hal_common/sensor_hal_base.hpp"
#include "rm_hal_common/sensor_timestamp.hpp"
#include "rm_hal_imu/imu_hal.hpp"
#include "rm_hal_imu/imu_types.hpp"
#include "rm_hal_lidar/lidar_2d_hal.hpp"
#include "rm_hal_lidar/lidar_2d_types.hpp"
#include "rm_hal_lidar/lidar_3d_hal.hpp"
#include "rm_hal_lidar/lidar_3d_types.hpp"

int main() { return 0; }
