#!/usr/bin/env bash
# scripts/build.sh — Local one-shot configure, build, and test.
#
# Usage:
#   ./scripts/build.sh             # Debug build in ./build
#   ./scripts/build.sh Release     # Release build
#   ./scripts/build.sh Debug out   # Custom build type and directory

set -euo pipefail

BUILD_TYPE="${1:-Debug}"
BUILD_DIR="${2:-build}"

echo "==> Configure  (type=${BUILD_TYPE}, dir=${BUILD_DIR})"
cmake -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_TESTING=ON

echo "==> Build"
cmake --build "${BUILD_DIR}" --parallel

echo "==> Test"
ctest --test-dir "${BUILD_DIR}" --output-on-failure --parallel

echo "==> Done ✓"
