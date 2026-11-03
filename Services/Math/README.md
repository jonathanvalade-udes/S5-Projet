# Math

## Description
Service created based on the `ServiceManager` template with modern CMake build system and shared `libs` integration.

## Directory Structure
```
Math/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Documentation
├── inc/
│   └── Math.hpp    # Service header
└── src/
    ├── Math.cpp    # Service implementation
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
./Math
```
