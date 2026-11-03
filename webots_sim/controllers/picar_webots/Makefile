# ==============================================================================
# Makefile Robuste pour le contrôleur C++ picar_webots
# Compatible macOS natif et Webots IDE
# ==============================================================================

WEBOTS_HOME ?= /Applications/Webots.app/Contents

CXX ?= clang++
CXXFLAGS = -std=c++17 -O2 \
  -I$(WEBOTS_HOME)/include/controller/cpp \
  -I$(WEBOTS_HOME)/include/controller/c \
  -I../../../firmware/include

LDFLAGS = -L$(WEBOTS_HOME)/lib/controller \
  -lCppController -lController \
  -Wl,-rpath,/Applications/Webots.app \
  -Wl,-rpath,$(WEBOTS_HOME)/lib/controller

all: picar_webots

picar_webots: picar_webots.cpp
	@echo "Compilation du contrôleur C++ natif Webots..."
	$(CXX) $(CXXFLAGS) picar_webots.cpp -o picar_webots $(LDFLAGS)
	@echo "✓ Binaire picar_webots généré avec succès."

clean:
	rm -f picar_webots *.o
