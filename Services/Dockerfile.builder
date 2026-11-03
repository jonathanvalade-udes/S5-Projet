# ==============================================================================
# Dockerfile pour l'environnement de build ARM64 statique (Raspberry Pi / PiCar)
# ==============================================================================
FROM alpine:latest

# Installation des paquets nécessaires pour compiler C++17 en binaire statique
RUN apk add --no-cache \
    build-base \
    cmake \
    linux-headers \
    file

WORKDIR /workspace

# Commande par défaut : compilation statique de l'ensemble des services
CMD ["sh", "-c", "cmake -B build-embedded -S . -DSTATIC=ON && cmake --build build-embedded --parallel"]
