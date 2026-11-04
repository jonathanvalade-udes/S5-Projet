cmake_minimum_required(VERSION 3.16)
project(SimBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

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
