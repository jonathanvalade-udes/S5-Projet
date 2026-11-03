cmake_minimum_required(VERSION 3.16)
project(Services LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
add_compile_options(-Wall -Wextra -O2)

# Global static linking option (OFF by default)
option(STATIC "Build with static linking (-static)" OFF)
option(STATIC_BUILD "Build with static linking (-static)" OFF)

# Set global libs directory
set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs" CACHE PATH "Path to libs directory")

# Add libs subdirectory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/CMakeLists.txt")
    add_subdirectory(libs)
endif()

# Auto-discover and add all service subdirectories containing CMakeLists.txt
file(GLOB SUB_ENTRIES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(ENTRY ${SUB_ENTRIES})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${ENTRY}" AND
       EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${ENTRY}/CMakeLists.txt" AND
       NOT "${ENTRY}" STREQUAL "libs" AND
       NOT "${ENTRY}" STREQUAL "build" AND
       NOT "${ENTRY}" MATCHES "^build")
        message(STATUS "Configuring service: ${ENTRY}")
        add_subdirectory("${ENTRY}")
    endif()
endforeach()
