set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Look for available aarch64 cross-compiler
find_program(AARCH64_CXX NAMES aarch64-gnu-g++ aarch64-linux-gnu-g++)
if(AARCH64_CXX)
    set(CMAKE_CXX_COMPILER ${AARCH64_CXX})
else()
    set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
endif()

find_program(AARCH64_C NAMES aarch64-gnu-gcc aarch64-linux-gnu-gcc)
if(AARCH64_C)
    set(CMAKE_C_COMPILER ${AARCH64_C})
else()
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
