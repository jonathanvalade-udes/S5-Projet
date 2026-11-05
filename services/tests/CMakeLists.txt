cmake_minimum_required(VERSION 3.16)
project(ServicesTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

option(STATIC "Build with static linking (-static)" OFF)
option(STATIC_BUILD "Build with static linking (-static)" OFF)

if(STATIC OR STATIC_BUILD)
    link_libraries(-static)
endif()


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
target_link_libraries(test_ipc PRIVATE Threads::Threads shared_headers)
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

add_executable(test_i2c test_i2c.cpp)
target_include_directories(test_i2c PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_i2c PRIVATE Threads::Threads)
add_test(NAME test_i2c COMMAND test_i2c)

add_executable(test_telemetry 
    test_telemetry.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/src/TelemetryService.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/src/PosixClient.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/src/TcpStreamer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/src/ShmCollector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/src/UsbLogger.cpp
)
target_include_directories(test_telemetry PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
    ${CMAKE_CURRENT_SOURCE_DIR}/../TelemetryService/inc
)
target_link_libraries(test_telemetry PRIVATE 
    Threads::Threads 
    shared_headers
)
add_test(NAME test_telemetry COMMAND test_telemetry)

add_executable(test_live_motor test_live_motor.cpp)
target_include_directories(test_live_motor PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_live_motor PRIVATE Threads::Threads)

add_executable(test_service_manager 
    test_service_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../ServiceManager/src/ServiceManager.cpp
)
target_include_directories(test_service_manager PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../ServiceManager/inc
)
target_link_libraries(test_service_manager PRIVATE Threads::Threads)
add_test(NAME test_service_manager COMMAND test_service_manager)

add_executable(test_peripherals 
    test_peripherals.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../PeripheralsService/src/PeripheralsService.cpp
)
target_include_directories(test_peripherals PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
    ${CMAKE_CURRENT_SOURCE_DIR}/../PeripheralsService/inc
)
target_link_libraries(test_peripherals PRIVATE Threads::Threads)
add_test(NAME test_peripherals COMMAND test_peripherals)


# Navigation FSM : ralentissement a l'approche d'un obstacle
add_executable(test_fsm_navigator
    test_fsm_navigator.cpp
    ../FSM/src/FsmNavigator.cpp
    ../FSM/src/ObstacleAvoidance.cpp
)
target_include_directories(test_fsm_navigator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../FSM/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/shared-headers
)
target_link_libraries(test_fsm_navigator PRIVATE Threads::Threads)
add_test(NAME test_fsm_navigator COMMAND test_fsm_navigator)
