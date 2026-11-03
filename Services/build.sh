#!/usr/bin/env bash
# ==============================================================================
# Helper build script for Services with --static support
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
STATIC_FLAG="OFF"
TARGET="all"
TOOLCHAIN_FLAG=""

usage() {
    cat <<EOF
Usage: $0 [options] [target]

Options:
  --static       Build executables with static linking (-static)
  --embedded     Cross-compile for aarch64 using toolchain-aarch64.cmake
  --clean        Remove build directory and exit
  -h, --help     Show this help message

Target:
  all            Build all services (default)
  embedded       Build embedded targets using cross-compiler
  static         Build embedded static targets
  <service>      Build specific service (e.g. TelemetryService, ServiceManager)

Examples:
  $0                         # Build all services dynamically (default)
  $0 --static                # Build all services statically
  $0 --embedded              # Build all services for aarch64 (dynamic)
  $0 --embedded --static     # Build all services for aarch64 (static)
  $0 static                  # Build dedicated static targets
EOF
    exit "${1:-0}"
}

POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --static)
            STATIC_FLAG="ON"
            shift
            ;;
        --embedded)
            TOOLCHAIN_FLAG="-DCMAKE_TOOLCHAIN_FILE=${SCRIPT_DIR}/toolchain-aarch64.cmake"
            shift
            ;;
        --clean)
            rm -rf "${BUILD_DIR}"
            echo "Cleaned '${BUILD_DIR}'"
            exit 0
            ;;
        -h|--help)
            usage 0
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ${#POSITIONAL_ARGS[@]} -gt 0 ]]; then
    TARGET="${POSITIONAL_ARGS[0]}"
fi

echo "==> Configuring with CMake (STATIC=${STATIC_FLAG})..."
cmake -B "${BUILD_DIR}" -DSTATIC="${STATIC_FLAG}" ${TOOLCHAIN_FLAG}

echo "==> Building target '${TARGET}'..."
if [[ "${TARGET}" == "all" ]]; then
    cmake --build "${BUILD_DIR}"
else
    cmake --build "${BUILD_DIR}" --target "${TARGET}"
fi
