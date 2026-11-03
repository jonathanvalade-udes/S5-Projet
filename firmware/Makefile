# ==============================================================================
# Makefile - PiCar Robotics Platform (C++17 Standalone Services)
# Compatible hôte macOS (Tests locaux & Mock) et Cross-Compilation Buildroot ARM
# ==============================================================================

# Détection de l'hôte
UNAME_S := $(shell uname -s)

# Compilateur natif hôte pour les tests unitaires et exécutables locaux
HOST_CXX ?= clang++
HOST_CXXFLAGS ?= -O2 -Wall -Wextra -std=c++17 -Iinclude -pthread

# Cross-compilateur Buildroot pour Raspberry Pi 3
BUILDROOT_PATH ?= /chemin/vers/buildroot/output/host/bin
CROSS_CXX ?= $(BUILDROOT_PATH)/arm-buildroot-linux-gnueabihf-g++
CROSS_CXXFLAGS ?= -O2 -Wall -Wextra -std=c++17 -march=armv8-a -mfpu=crypto-neon-fp-armv8 -mfloat-abi=hard -Iinclude
CROSS_LDFLAGS ?= -lpthread -lrt

# Cible IP par défaut
PI_IP ?= 192.168.1.100

.PHONY: all test test_ipc test_control test_sim bins clean cross deploy help

all: test bins

# ==============================================================================
# Binaires Exécutables des Services (Compilables sur macOS en mode Mock/Live)
# ==============================================================================
bins: bin/picar_sensors bin/picar_motor bin/picar_brain bin/picar_sim_bridge

bin/picar_sensors: src/picar_sensors.cpp include/ipc/*.hpp include/filters/*.hpp include/hal/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) src/picar_sensors.cpp -o bin/picar_sensors

bin/picar_motor: src/picar_motor.cpp include/ipc/*.hpp include/hal/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) src/picar_motor.cpp -o bin/picar_motor

bin/picar_brain: src/picar_brain.cpp include/ipc/*.hpp include/control/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) src/picar_brain.cpp -o bin/picar_brain

bin/picar_sim_bridge: src/picar_sim_bridge.cpp include/ipc/*.hpp include/utils/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) src/picar_sim_bridge.cpp -o bin/picar_sim_bridge

# ==============================================================================
# Tests Unitaires et d'Intégration Locaux (macOS)
# ==============================================================================
test: bin/test_filters bin/test_ipc bin/test_control bin/test_sim_bridge
	@echo "--- 1. Exécution des tests unitaires filtres sur macOS ---"
	@./bin/test_filters
	@echo "\n--- 2. Exécution des tests d'intégration IPC & Failsafe ---"
	@./bin/test_ipc
	@echo "\n--- 3. Exécution des tests de contrôle Motion Profiler & Line Follower ---"
	@./bin/test_control
	@echo "\n--- 4. Exécution des tests du Pont Virtuel TCP Godot 4 ---"
	@./bin/test_sim_bridge

bin/test_filters: tests/test_filters.cpp include/filters/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) tests/test_filters.cpp -o bin/test_filters

bin/test_ipc: tests/test_ipc_services.cpp include/ipc/*.hpp include/hal/*.hpp include/filters/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) tests/test_ipc_services.cpp -o bin/test_ipc

bin/test_control: tests/test_control.cpp include/control/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) tests/test_control.cpp -o bin/test_control

bin/test_sim_bridge: tests/test_sim_bridge.cpp include/ipc/*.hpp include/utils/*.hpp
	@mkdir -p bin
	$(HOST_CXX) $(HOST_CXXFLAGS) tests/test_sim_bridge.cpp -o bin/test_sim_bridge

# ==============================================================================
# Cross-compilation pour Raspberry Pi 3 (Buildroot)
# ==============================================================================
cross:
	@mkdir -p bin/arm
	@echo "--- Cross-compilation Buildroot ARM pour Raspberry Pi 3 ---"
	$(CROSS_CXX) $(CROSS_CXXFLAGS) src/picar_sensors.cpp -o bin/arm/picar_sensors $(CROSS_LDFLAGS)
	$(CROSS_CXX) $(CROSS_CXXFLAGS) src/picar_motor.cpp -o bin/arm/picar_motor $(CROSS_LDFLAGS)
	$(CROSS_CXX) $(CROSS_CXXFLAGS) src/picar_brain.cpp -o bin/arm/picar_brain $(CROSS_LDFLAGS)

deploy: cross
	@echo "--- Déploiement à chaud des services vers root@$(PI_IP):/root/ ---"
	scp bin/arm/picar_sensors bin/arm/picar_motor bin/arm/picar_brain root@$(PI_IP):/root/
	scp init.d/S80picar_sensors init.d/S85picar_brain init.d/S90picar_motor root@$(PI_IP):/etc/init.d/

clean:
	rm -rf bin *.o
