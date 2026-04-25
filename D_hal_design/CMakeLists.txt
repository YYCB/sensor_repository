# D_hal_design/CMakeLists.txt
#
# Defines the rm_hal_headers INTERFACE library.
# Consumers link with:  target_link_libraries(<target> PRIVATE rm_hal::headers)

# ── INTERFACE library ──────────────────────────────────────────────────────────
add_library(rm_hal_headers INTERFACE)
add_library(rm_hal::headers ALIAS rm_hal_headers)

target_include_directories(rm_hal_headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(rm_hal_headers INTERFACE cxx_std_17)

# ── Install rules ──────────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS rm_hal_headers
    EXPORT rm_hal_targets
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

install(EXPORT rm_hal_targets
    FILE        rm_hal_headersTargets.cmake
    NAMESPACE   rm_hal::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rm_hal_headers
)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
