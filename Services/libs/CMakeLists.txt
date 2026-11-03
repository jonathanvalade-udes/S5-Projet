cmake_minimum_required(VERSION 3.16)
project(SharedLibs LANGUAGES CXX)

find_package(Threads REQUIRED)

# Header-only shared-headers interface library
add_library(shared_headers INTERFACE)
target_include_directories(shared_headers INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/shared-headers
    ${CMAKE_CURRENT_SOURCE_DIR}/shared-services
)

# Namespace alias for shared_headers
add_library(libs::shared_headers ALIAS shared_headers)

# Shared services library (memory server and subscriber)
add_library(shared_services STATIC
    shared-services/memory/server.cpp
    shared-services/memory/subscriber.cpp
)

target_include_directories(shared_services PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/shared-services
    ${CMAKE_CURRENT_SOURCE_DIR}/shared-services/memory
    ${CMAKE_CURRENT_SOURCE_DIR}/shared-headers
)

target_link_libraries(shared_services PUBLIC
    Threads::Threads
    ${CMAKE_DL_LIBS}
    rt
)

# Namespace alias for shared_services
add_library(libs::shared_services ALIAS shared_services)

# Transitive interface linking so existing services get shared_services
target_link_libraries(shared_headers INTERFACE shared_services)
