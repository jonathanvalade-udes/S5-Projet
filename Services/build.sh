#!/usr/bin/env bash
# ==============================================================================
# Helper build script for Services with --static & --embedded (ARM64) support
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATIC_FLAG="OFF"
EMBEDDED_MODE=false
TARGET="all"
BUILD_DIR="${SCRIPT_DIR}/build"

usage() {
    cat <<EOF
Usage: $0 [options] [target]

Options:
  --static       Build executables with static linking (-static)
  --embedded     Build for Linux ARM64 (aarch64). Uses Docker automatically on macOS.
  --clean        Remove build directories (build and build-embedded) and exit
  -h, --help     Show this help message

Target:
  all            Build all services (default)
  <service>      Build specific service (e.g. FSM, Motors, Sensors, TelemetryService, ServiceManager)

Examples:
  $0                         # Build all services natively for host (macOS / Linux)
  $0 --embedded --static     # Build all services statically for Raspberry Pi (ARM64)
  $0 --clean                 # Clean all build artifacts
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
            EMBEDDED_MODE=true
            BUILD_DIR="${SCRIPT_DIR}/build-embedded"
            shift
            ;;
        --clean)
            rm -rf "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/build-embedded"
            echo "Cleaned build directories."
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

# Detect if we are on macOS needing Linux ARM64 compilation
if [[ "${EMBEDDED_MODE}" == true ]]; then
    if [[ "$(uname -s)" == "Darwin" ]] && ! command -v aarch64-linux-gnu-g++ &>/dev/null; then
        if command -v docker &>/dev/null && docker info &>/dev/null; then
            echo "==> macOS detected without aarch64 cross-toolchain."
            echo "==> Using Docker (native aarch64 Linux container) to build static ARM64 binaries..."
            IMAGE_NAME="alpine"
            CMD="apk add --no-cache build-base cmake linux-headers >/dev/null && cmake -B /workspace/build-embedded -S /workspace -DSTATIC=${STATIC_FLAG} && cmake --build /workspace/build-embedded --target ${TARGET}"
            if docker image inspect alpine-picar-builder &>/dev/null; then
                IMAGE_NAME="alpine-picar-builder"
                CMD="cmake -B /workspace/build-embedded -S /workspace -DSTATIC=${STATIC_FLAG} && cmake --build /workspace/build-embedded --target ${TARGET}"
            fi
            docker run --rm --dns 8.8.8.8 -v "${SCRIPT_DIR}":/workspace -w /workspace "${IMAGE_NAME}" sh -c "${CMD}"
            echo ""
            echo "==> SUCCESS! Binaries generated in Services/build-embedded/:"
            file "${BUILD_DIR}/ServiceManager/init" "${BUILD_DIR}/FSM/FSM" "${BUILD_DIR}/Sensors/Sensors" "${BUILD_DIR}/Motors/Motors" "${BUILD_DIR}/TelemetryService/TelemetryService"
            exit 0
        else
            echo "Error: aarch64-linux-gnu-g++ not found and Docker is not running." >&2
            echo "Please start Docker Desktop or install aarch64-linux-gnu-gcc." >&2
            exit 1
        fi
    fi
fi

echo "==> Configuring with CMake (STATIC=${STATIC_FLAG})..."
TOOLCHAIN_FLAG=""
if [[ "${EMBEDDED_MODE}" == true ]]; then
    TOOLCHAIN_FLAG="-DCMAKE_TOOLCHAIN_FILE=${SCRIPT_DIR}/toolchain-aarch64.cmake"
fi

cmake -B "${BUILD_DIR}" -DSTATIC="${STATIC_FLAG}" ${TOOLCHAIN_FLAG}

echo "==> Building target '${TARGET}'..."
if [[ "${TARGET}" == "all" ]]; then
    cmake --build "${BUILD_DIR}"
else
    cmake --build "${BUILD_DIR}" --target "${TARGET}"
fi
