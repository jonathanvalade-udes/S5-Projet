#!/usr/bin/env bash
# ==============================================================================
# build_for_embedded.sh
# Cross-compilation for ARM64 (aarch64):
# - ServiceManager (init) : 100% Statically linked aarch64 (-static -no-pie)
# - All other services   : Dynamically linked aarch64
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-embedded"
TARGET="all"
NUM_JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"

usage() {
    cat <<EOF
Usage: $0 [options] [target]

Options:
  --clean         Remove build-embedded directory and exit
  -j, --jobs N    Number of parallel compilation jobs (default: ${NUM_JOBS})
  -h, --help      Show this help message

Targets:
  all (default)   Build all services (ServiceManager static, others dynamic)
  <service>       Build specific service (e.g. ServiceManager, FSM, Motors, Sensors, SimBridge, TelemetryService)

Binaries produced in build-embedded/:
  - ServiceManager/init           : ELF 64-bit aarch64, statically linked
  - Motors/Motors                 : ELF 64-bit aarch64, dynamically linked
  - Sensors/Sensors               : ELF 64-bit aarch64, dynamically linked
  - FSM/FSM                       : ELF 64-bit aarch64, dynamically linked
  - SimBridge/SimBridge           : ELF 64-bit aarch64, dynamically linked
  - TelemetryService/TelemetryService : ELF 64-bit aarch64, dynamically linked
  - PeripheralsService/PeripheralsService : ELF 64-bit aarch64, dynamically linked
EOF
    exit "${1:-0}"
}

POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            echo "==> Cleaning ${BUILD_DIR}..."
            rm -rf "${BUILD_DIR}"
            echo "Done."
            exit 0
            ;;
        -j|--jobs)
            NUM_JOBS="$2"
            shift 2
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

echo "======================================================================="
echo " Building for Embedded ARM64 (aarch64)"
echo " - ServiceManager (init) -> Statically linked aarch64"
echo " - Other Services       -> Dynamically linked aarch64"
echo " Target: ${TARGET} | Jobs: ${NUM_JOBS}"
echo "======================================================================="

# Check toolchain: native cross-compiler or Docker container
if command -v aarch64-linux-gnu-g++ &>/dev/null && command -v aarch64-linux-gnu-gcc &>/dev/null; then
    echo "==> Using host cross-compiler: $(which aarch64-linux-gnu-g++)"

    cmake -B "${BUILD_DIR}" -S "${SCRIPT_DIR}" \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
        -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
        -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
        -DSTATIC=ON \
        -DCMAKE_BUILD_TYPE=Release

    if [[ "${TARGET}" == "all" ]]; then
        cmake --build "${BUILD_DIR}" -j"${NUM_JOBS}"
    else
        cmake --build "${BUILD_DIR}" --target "${TARGET}" -j"${NUM_JOBS}"
    fi

elif command -v docker &>/dev/null && docker info &>/dev/null; then
    echo "==> aarch64 cross-compiler not found on host. Falling back to Docker..."
    IMAGE_NAME="alpine-picar-builder"
    if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
        echo "==> Building Docker image '${IMAGE_NAME}'..."
        docker build -f "${SCRIPT_DIR}/Dockerfile.builder" -t "${IMAGE_NAME}" "${SCRIPT_DIR}"
    fi

    CMD="cmake -B /workspace/build-embedded -S /workspace -DSTATIC=ON -DCMAKE_BUILD_TYPE=Release && cmake --build /workspace/build-embedded --target ${TARGET} -j${NUM_JOBS}"
    docker run --rm --platform linux/arm64 --dns 8.8.8.8 -v "${SCRIPT_DIR}":/workspace -w /workspace "${IMAGE_NAME}" sh -c "${CMD}"
else
    echo "Error: Neither aarch64-linux-gnu-g++ nor Docker was found." >&2
    echo "Please install gcc-aarch64-linux-gnu and g++-aarch64-linux-gnu (or Docker)." >&2
    exit 1
fi

BIN_DIR="${BUILD_DIR}/bin"
mkdir -p "${BIN_DIR}"

SERVICES=("ServiceManager/init" "Motors/Motors" "Sensors/Sensors" "FSM/FSM" "SimBridge/SimBridge" "TelemetryService/TelemetryService" "PeripheralsService/PeripheralsService")

echo ""
echo "==> Copying ARM64 binaries to unified directory: ${BIN_DIR}/"
for svc in "${SERVICES[@]}"; do
    bin_name="$(basename "${svc}")"
    src_path="${BUILD_DIR}/${svc}"
    dst_path="${BIN_DIR}/${bin_name}"
    if [[ -f "${src_path}" ]]; then
        cp -f "${src_path}" "${dst_path}"
    fi
done

echo ""
echo "======================================================================="
echo " Embedded ARM64 Binaries in ${BIN_DIR}/:"
echo "======================================================================="

EXPECTED_BINS=("init" "Motors" "Sensors" "FSM" "SimBridge" "TelemetryService" "PeripheralsService")

for b in "${EXPECTED_BINS[@]}"; do
    bin_path="${BIN_DIR}/${b}"
    if [[ -f "${bin_path}" ]]; then
        info=$(file -b "${bin_path}")
        echo -e "\033[1;32m[OK]\033[0m ${b}:"
        echo "     ${info}"
    else
        if [[ "${TARGET}" == "all" ]] || [[ "${b}" == *"${TARGET}"* ]] || [[ "${TARGET}" == "ServiceManager" && "${b}" == "init" ]]; then
            echo -e "\033[1;31m[MISSING]\033[0m ${b}"
        fi
    fi
done
echo "======================================================================="

