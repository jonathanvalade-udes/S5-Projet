cmake_minimum_required(VERSION 3.16)
project(telemetry_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options de compilation
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# Option pour linkage statique si désiré
option(STATIC "Build with static linking (-static)" OFF)
if(STATIC)
    add_link_options(-static -no-pie)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static -no-pie")
endif()

find_package(Threads REQUIRED)

set(SRCS
    src/TcpClient.cpp
    src/ProtocolHandler.cpp
    src/CsvLogger.cpp
    src/WebServer.cpp
    src/TelemetryClient.cpp
    src/main.cpp
)

add_executable(telemetry_client ${SRCS})

# Chemins d'inclusion : inc local et shared-headers en lecture seule
get_filename_component(SHARED_HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../services/libs/shared-headers" ABSOLUTE)

target_include_directories(telemetry_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${SHARED_HEADERS_DIR}
)

target_link_libraries(telemetry_client PRIVATE
    Threads::Threads
    $<$<PLATFORM_ID:Linux>:rt>
)

# Copie automatique du dossier web dans le répertoire de build pour exécution directe
add_custom_command(TARGET telemetry_client POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/web
    $<TARGET_FILE_DIR:telemetry_client>/web
)

# Test d'intégration automatisé
add_executable(test_client
    tests/test_client.cpp
    src/TcpClient.cpp
    src/ProtocolHandler.cpp
    src/CsvLogger.cpp
    src/WebServer.cpp
)
target_include_directories(test_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${SHARED_HEADERS_DIR}
)
target_link_libraries(test_client PRIVATE
    Threads::Threads
    $<$<PLATFORM_ID:Linux>:rt>
)

# Test : l'arret d'urgence doit atteindre le FSM et les moteurs
add_executable(test_emergency_stop
    tests/test_emergency_stop.cpp
    src/TcpClient.cpp
    src/ProtocolHandler.cpp
)
target_include_directories(test_emergency_stop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${SHARED_HEADERS_DIR}
)
target_link_libraries(test_emergency_stop PRIVATE
    Threads::Threads
    $<$<PLATFORM_ID:Linux>:rt>
)
