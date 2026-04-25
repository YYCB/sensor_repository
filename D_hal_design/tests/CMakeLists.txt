# D_hal_design/tests/CMakeLists.txt

# ── test_compile ───────────────────────────────────────────────────────────────
# Includes every public header in a single translation unit.
# The test passes simply by compiling and linking without errors.
add_executable(test_compile test_compile.cpp)
target_link_libraries(test_compile PRIVATE rm_hal::headers)
target_enable_warnings(test_compile)
add_test(NAME hal.compile_check COMMAND test_compile)

# ── test_types ─────────────────────────────────────────────────────────────────
# Runtime checks for types, free functions, and struct default values.
add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE rm_hal::headers)
target_enable_warnings(test_types)
add_test(NAME hal.type_tests COMMAND test_types)
