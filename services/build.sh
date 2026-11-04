#!/usr/bin/env bash
# ==============================================================================
# Helper build script for Services with --static & --embedded (ARM64) support
# ==============================================================================

set -euo pipefail

# cmake peut vivre dans un venv utilisateur si apt n'est pas disponible (comme dans run_sim.sh)
[ -d "$HOME/.picar-venv/bin" ] && export PATH="$HOME/.picar-venv/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATIC_FLAG="OFF"
EMBEDDED_MODE=false
RUN_TESTS=false
SIM_MODE=false
TARGET="all"
BUILD_DIR="${SCRIPT_DIR}/build"

usage() {
    cat <<EOF
Usage: $0 [options] [target]

Options:
  --static        Build executables with static linking (-static)
  --embedded      Build for Linux ARM64 (aarch64). Uses Docker automatically on macOS.
  --test          Run the full test suite (ctest) inside the Linux environment
  --sim           Start SimBridge + FSM in Docker for Godot simulation (port 9000)
  --build-docker  Build or update the Docker builder image (alpine-picar-builder) and exit
  --clean         Remove build directories (build and build-embedded) and exit
  -h, --help      Show this help message

Target:
  all            Build all services (default)
  <service>      Build specific service (e.g. FSM, Motors, Sensors, TelemetryService, ServiceManager)

Examples:
  $0 --embedded --static     # Build all services statically for Raspberry Pi (ARM64)
  $0 --test                  # Run all tests in the Linux Docker container on Mac
  $0 --sim                   # Launch Godot simulation stack (SimBridge + FSM) via Docker
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
        --test)
            RUN_TESTS=true
            EMBEDDED_MODE=true
            BUILD_DIR="${SCRIPT_DIR}/build-embedded"
            shift
            ;;
        --sim)
            SIM_MODE=true
            shift
            ;;
        --build-docker)
            echo "==> Building Docker builder image 'alpine-picar-builder' from Dockerfile.builder..."
            docker build -f "${SCRIPT_DIR}/Dockerfile.builder" -t alpine-picar-builder "${SCRIPT_DIR}"
            echo "==> Image 'alpine-picar-builder' is ready for instant builds!"
            exit 0
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

if [[ "${SIM_MODE}" == true ]]; then
    echo "==> Starting Godot Simulation stack (SimBridge + FSM) via Docker Compose..."
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up --build simulation
    exit 0
fi

# Detect if we are on macOS needing Linux ARM64 compilation
if [[ "${EMBEDDED_MODE}" == true ]]; then
    if [[ "$(uname -s)" == "Darwin" ]] && ! command -v aarch64-linux-gnu-g++ &>/dev/null; then
        if command -v docker &>/dev/null && docker info &>/dev/null; then
            echo "==> macOS detected without aarch64 cross-toolchain."
            echo "==> Using Docker (native aarch64 Linux container) to build static ARM64 binaries..."
            IMAGE_NAME="alpine-picar-builder"
            if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
                echo "==> Docker image '${IMAGE_NAME}' not found. Building it once from Dockerfile.builder..."
                docker build -f "${SCRIPT_DIR}/Dockerfile.builder" -t "${IMAGE_NAME}" "${SCRIPT_DIR}"
            fi
            if [[ "${RUN_TESTS}" == true ]]; then
                CMD="cmake -B /workspace/build-embedded -S /workspace -DSTATIC=${STATIC_FLAG} && cmake --build /workspace/build-embedded && ctest --test-dir /workspace/build-embedded --output-on-failure"
                docker run --rm --dns 8.8.8.8 -v "${SCRIPT_DIR}":/workspace -w /workspace "${IMAGE_NAME}" sh -c "${CMD}"
                echo ""
                echo "==> ALL TESTS PASSED SUCCESSFULLY IN DOCKER (LINUX)!"
                exit 0
            fi
            CMD="cmake -B /workspace/build-embedded -S /workspace -DSTATIC=${STATIC_FLAG} && cmake --build /workspace/build-embedded --target ${TARGET}"
            docker run --rm --dns 8.8.8.8 -v "${SCRIPT_DIR}":/workspace -w /workspace "${IMAGE_NAME}" sh -c "${CMD}"
            echo ""
            echo "==> SUCCESS! Binaries generated in services/build-embedded/:"
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
cmake -B "${BUILD_DIR}" -DSTATIC="${STATIC_FLAG}"

echo "==> Building target '${TARGET}'..."
if [[ "${TARGET}" == "all" ]]; then
    cmake --build "${BUILD_DIR}"
else
    cmake --build "${BUILD_DIR}" --target "${TARGET}"
fi

if [[ "${RUN_TESTS}" == true ]]; then
    echo "==> Running test suite..."
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
fi
