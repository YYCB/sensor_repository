cmake_minimum_required(VERSION 3.22)

project(rmos_sensor_hal
    VERSION   0.4.0
    LANGUAGES CXX
    DESCRIPTION "RMOS Sensor HAL — header-only C++17 interface library"
)

# ── C++ standard ───────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ── Compiler warnings (function used by test targets) ─────────────────────────
include(cmake/CompilerWarnings.cmake)

# ── Testing ────────────────────────────────────────────────────────────────────
# Enable with:  cmake -DBUILD_TESTING=ON ...
include(CTest)

# ── Sub-projects ───────────────────────────────────────────────────────────────
add_subdirectory(D_hal_design)
