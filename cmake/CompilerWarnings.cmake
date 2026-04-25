# cmake/CompilerWarnings.cmake
#
# Provides target_enable_warnings(<target>) — applies a strict but
# portable warning set to non-INTERFACE executable / library targets.
#
# Usage (in any CMakeLists.txt after including this file from the root):
#   add_executable(my_test ...)
#   target_enable_warnings(my_test)

function(target_enable_warnings target)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wnull-dereference
            -Wcast-align
        >
        $<$<CXX_COMPILER_ID:MSVC>:
            /W4
            /permissive-
        >
    )
endfunction()
