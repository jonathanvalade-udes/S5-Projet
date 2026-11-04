# PeripheralsService

## Description
Service managing peripherals and hardware interfaces, providing a POSIX UNIX domain socket file (`/tmp/peripherals.sock` by default) for inter-process communication (IPC).

## Directory Structure
```
PeripheralsService/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Documentation
├── inc/
│   └── PeripheralsService.hpp    # Service header
└── src/
    ├── PeripheralsService.cpp    # Service implementation
    └── main.cpp            # Entry point & signal handling
```

## Shared Libraries Integration
This service automatically links and includes libraries found in `../libs` (e.g. `libs/shared-headers`).
You can directly include shared headers in your code:
```cpp
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "car/full_packet.hpp"
```

## Build Instructions

### Host Build (Dynamic / Default)
```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```

### Embedded Build (Dynamic / Default)
```bash
mkdir -p build && cd build
cmake ..
cmake --build . --target embedded
```

### Static Build (When Needed)
To build statically linked binary, pass `-DSTATIC=ON` to CMake:
```bash
mkdir -p build && cd build
cmake .. -DSTATIC=ON
cmake --build .
```
Or build the dedicated static target:
```bash
cmake --build . --target static
```

## Run
```bash
./PeripheralsService
```
