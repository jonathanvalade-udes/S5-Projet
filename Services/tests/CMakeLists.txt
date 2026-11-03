cmake_minimum_required(VERSION 3.16)
project(ServicesTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_filters test_filters.cpp)
target_include_directories(test_filters PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_filters PRIVATE Threads::Threads)
add_test(NAME test_filters COMMAND test_filters)

add_executable(test_ipc test_ipc_services.cpp)
target_include_directories(test_ipc PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_ipc PRIVATE Threads::Threads)
add_test(NAME test_ipc COMMAND test_ipc)

add_executable(test_control test_control.cpp)
target_include_directories(test_control PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_control PRIVATE Threads::Threads)
add_test(NAME test_control COMMAND test_control)

add_executable(test_sim_bridge test_sim_bridge.cpp)
target_include_directories(test_sim_bridge PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_sim_bridge PRIVATE Threads::Threads)
add_test(NAME test_sim_bridge COMMAND test_sim_bridge)
