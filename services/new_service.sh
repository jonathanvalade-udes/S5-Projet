#!/usr/bin/env bash
# ==============================================================================
# Script to create a new service template based on ServiceManager with CMake
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Display usage
usage() {
    cat <<EOF
Usage: $0 <ServiceName> [TargetDirectory]

Arguments:
  ServiceName       Name of the new service (e.g., TelemetryService, MotorService)
  TargetDirectory   Optional custom path to create the service directory
                    (Default: ${SCRIPT_DIR}/<ServiceName>)

Options:
  -h, --help        Show this help message

Examples:
  $0 TelemetryService
  $0 MotorService /custom/path/MotorService
EOF
    exit "${1:-1}"
}

# Parse options
if [[ $# -eq 0 ]]; then
    usage 1
fi

case "$1" in
    -h|--help)
        usage 0
        ;;
    *)
        SERVICE_NAME="$1"
        ;;
esac

# Validate service name (alphanumeric and underscore only)
if [[ ! "${SERVICE_NAME}" =~ ^[a-zA-Z0-9_]+$ ]]; then
    echo "Error: ServiceName '${SERVICE_NAME}' contains invalid characters. Use letters, numbers, and underscores only." >&2
    exit 1
fi

TARGET_DIR="${2:-${SCRIPT_DIR}/${SERVICE_NAME}}"

# Check if target directory exists and is non-empty
if [[ -d "${TARGET_DIR}" ]]; then
    if [[ -n "$(ls -A "${TARGET_DIR}" 2>/dev/null)" ]]; then
        echo "Error: Target directory '${TARGET_DIR}' already exists and is not empty." >&2
        exit 1
    fi
fi

echo "Creating new service '${SERVICE_NAME}' at: ${TARGET_DIR}"

# Create directories
mkdir -p "${TARGET_DIR}/src"
mkdir -p "${TARGET_DIR}/inc"

# Generate CMakeLists.txt
cat <<'EOF' > "${TARGET_DIR}/CMakeLists.txt"
cmake_minimum_required(VERSION 3.16)
project(__SERVICE_NAME__ LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
add_compile_options(-Wall -Wextra -O2)

# Build options - static linking is optional (OFF by default)
option(STATIC "Build with static linking (-static)" OFF)
option(STATIC_BUILD "Build with static linking (-static)" OFF)

# Source files
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

if(NOT SRCS)
    message(WARNING "No .cpp files found in 'src/'. Add source files to build target.")
endif()

# Primary target executable name
set(TARGET_NAME "${PROJECT_NAME}")
set(BINARY_OUTPUT_NAME "${PROJECT_NAME}")

add_executable(${TARGET_NAME} ${SRCS})

# Target output naming: <name>_debug for debug mode, <name> for release mode
set_target_properties(${TARGET_NAME} PROPERTIES
    OUTPUT_NAME "${BINARY_OUTPUT_NAME}"
    OUTPUT_NAME_DEBUG "${BINARY_OUTPUT_NAME}_debug"
    OUTPUT_NAME_RELEASE "${BINARY_OUTPUT_NAME}"
)

# Apply static linking if requested via -DSTATIC=ON or -DSTATIC_BUILD=ON
if(STATIC OR STATIC_BUILD)
    target_link_options(${TARGET_NAME} PRIVATE -static)
    set(EMBEDDED_STATIC_FLAG "-static")
else()
    set(EMBEDDED_STATIC_FLAG "")
endif()

# Include local headers
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Locate and include shared libraries directory (libs)
if(NOT DEFINED LIBS_DIR)
    get_filename_component(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../libs" ABSOLUTE)
endif()

if(EXISTS "${LIBS_DIR}")
    target_include_directories(${TARGET_NAME} PRIVATE
        "${LIBS_DIR}"
        "${LIBS_DIR}/shared-headers"
    )
    # Dynamically include any additional subdirectories in libs
    file(GLOB LIB_SUBDIRS LIST_DIRECTORIES true "${LIBS_DIR}/*")
    foreach(SUBDIR ${LIB_SUBDIRS})
        if(IS_DIRECTORY "${SUBDIR}")
            target_include_directories(${TARGET_NAME} PRIVATE "${SUBDIR}")
            if(EXISTS "${SUBDIR}/inc")
                target_include_directories(${TARGET_NAME} PRIVATE "${SUBDIR}/inc")
            endif()
            if(EXISTS "${SUBDIR}/include")
                target_include_directories(${TARGET_NAME} PRIVATE "${SUBDIR}/include")
            endif()
        endif()
    endforeach()
endif()

if(TARGET shared_headers)
    target_link_libraries(${TARGET_NAME} PRIVATE shared_headers)
endif()

# Embedded compilation targets
find_program(CROSS_CXX NAMES aarch64-gnu-g++ aarch64-linux-gnu-g++)
if(CROSS_CXX AND NOT CMAKE_CROSSCOMPILING)
    # Standard embedded target (dynamic by default unless STATIC/STATIC_BUILD=ON)
    add_custom_target(${PROJECT_NAME}_embedded
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}"
        COMMAND ${CROSS_CXX} -Wall -Wextra -O2 -std=c++17 ${EMBEDDED_STATIC_FLAG}
                -I${CMAKE_CURRENT_SOURCE_DIR}/inc
                -I${LIBS_DIR}
                -I${LIBS_DIR}/shared-headers
                ${SRCS}
                -o "${CMAKE_CURRENT_BINARY_DIR}/${BINARY_OUTPUT_NAME}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Building embedded target: ${BINARY_OUTPUT_NAME} (static: ${STATIC})"
    )
    if(NOT TARGET embedded)
        add_custom_target(embedded)
    endif()
    add_dependencies(embedded ${PROJECT_NAME}_embedded)

    # Dedicated static target: cmake --build . --target static
    add_custom_target(${PROJECT_NAME}_static
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}"
        COMMAND ${CROSS_CXX} -Wall -Wextra -O2 -std=c++17 -static
                -I${CMAKE_CURRENT_SOURCE_DIR}/inc
                -I${LIBS_DIR}
                -I${LIBS_DIR}/shared-headers
                ${SRCS}
                -o "${CMAKE_CURRENT_BINARY_DIR}/${BINARY_OUTPUT_NAME}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Building embedded static target: ${BINARY_OUTPUT_NAME}"
    )
    if(NOT TARGET static)
        add_custom_target(static)
    endif()
    add_dependencies(static ${PROJECT_NAME}_static)
endif()
EOF

# Substitute placeholder __SERVICE_NAME__
sed -i "s/__SERVICE_NAME__/${SERVICE_NAME}/g" "${TARGET_DIR}/CMakeLists.txt"

# Generate header file in inc/
cat <<EOF > "${TARGET_DIR}/inc/${SERVICE_NAME}.hpp"
#pragma once

#include <atomic>
#include <string>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

namespace services {

class ${SERVICE_NAME} {
public:
    ${SERVICE_NAME}();
    ~${SERVICE_NAME}();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

private:
    std::atomic<bool> m_running{false};
};

} // namespace services
EOF

# Generate service implementation in src/
cat <<EOF > "${TARGET_DIR}/src/${SERVICE_NAME}.cpp"
#include "${SERVICE_NAME}.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

${SERVICE_NAME}::${SERVICE_NAME}() : m_running(false) {}

${SERVICE_NAME}::~${SERVICE_NAME}() {
    stop();
}

bool ${SERVICE_NAME}::init() {
    std::cout << "[${SERVICE_NAME}] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void ${SERVICE_NAME}::run() {
    std::cout << "[${SERVICE_NAME}] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[${SERVICE_NAME}] Service loop terminated." << std::endl;
}

void ${SERVICE_NAME}::stop() {
    if (m_running.load()) {
        std::cout << "[${SERVICE_NAME}] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
EOF

# Generate main.cpp in src/ based on ServiceManager template
cat <<EOF > "${TARGET_DIR}/src/main.cpp"
#include "${SERVICE_NAME}.hpp"
#include <csignal>
#include <iostream>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>

// Shared libraries headers included from libs/shared-headers
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

static services::${SERVICE_NAME}* g_service_instance = nullptr;

static void log_msg(int fd, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0) return;

    ssize_t written = -1;
    if (fd >= 0) {
        written = write(fd, buffer, len);
    }
    if (written < 0) {
        int cfd = open("/dev/console", O_WRONLY | O_NOCTTY);
        if (cfd >= 0) {
            ssize_t ret = write(cfd, buffer, len);
            (void)ret;
            close(cfd);
        }
    }
}

static void signal_handler(int sig) {
    log_msg(1, "[${SERVICE_NAME}] Received signal %d, initiating shutdown...\n", sig);
    if (g_service_instance) {
        g_service_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    log_msg(1, "[${SERVICE_NAME}] Starting up...\n");

    // Register POSIX signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    services::${SERVICE_NAME} service;
    g_service_instance = &service;

    if (!service.init()) {
        log_msg(2, "[${SERVICE_NAME}] Failed to initialize service!\n");
        return 1;
    }

    service.run();

    log_msg(1, "[${SERVICE_NAME}] Exited cleanly.\n");
    return 0;
}
EOF

# Generate README.md
cat <<EOF > "${TARGET_DIR}/README.md"
# ${SERVICE_NAME}

## Description
Service created based on the \`ServiceManager\` template with modern CMake build system and shared \`libs\` integration.

## Directory Structure
\`\`\`
${SERVICE_NAME}/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Documentation
├── inc/
│   └── ${SERVICE_NAME}.hpp    # Service header
└── src/
    ├── ${SERVICE_NAME}.cpp    # Service implementation
    └── main.cpp            # Entry point & signal handling
\`\`\`

## Shared Libraries Integration
This service automatically links and includes libraries found in \`../libs\` (e.g. \`libs/shared-headers\`).
You can directly include shared headers in your code:
\`\`\`cpp
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "car/full_packet.hpp"
\`\`\`

## Build Instructions

### Host Build (Dynamic / Default)
\`\`\`bash
mkdir -p build && cd build
cmake ..
cmake --build .
\`\`\`

### Embedded Build (Dynamic / Default)
\`\`\`bash
mkdir -p build && cd build
cmake ..
cmake --build . --target embedded
\`\`\`

### Static Build (When Needed)
To build statically linked binary, pass \`-DSTATIC=ON\` to CMake:
\`\`\`bash
mkdir -p build && cd build
cmake .. -DSTATIC=ON
cmake --build .
\`\`\`
Or build the dedicated static target:
\`\`\`bash
cmake --build . --target static
\`\`\`

## Run
\`\`\`bash
./${SERVICE_NAME}
\`\`\`
EOF

echo "Service '${SERVICE_NAME}' generated successfully in '${TARGET_DIR}'."
echo "To build '${SERVICE_NAME}':"
echo "  cd '${TARGET_DIR}'"
echo "  mkdir -p build && cd build"
echo "  cmake .."
echo "  cmake --build .                   # Dynamic (default)"
echo "  cmake --build . --target embedded # Embedded aarch64 (dynamic)"
echo "  cmake --build . --target static   # Embedded aarch64 (static)"
echo "  # Or pass -DSTATIC=ON to cmake for static linking:"
echo "  cmake .. -DSTATIC=ON && cmake --build ."
