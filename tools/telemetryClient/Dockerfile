# ==============================================================================
# Dockerfile multi-stage pour TelemetryClient (C++17 + Dashboard Web)
# Compatible Mac (Apple Silicon / Intel), Linux et Windows
# ==============================================================================
FROM alpine:latest AS builder

RUN apk add --no-cache \
    build-base \
    cmake \
    linux-headers

WORKDIR /app

# Copie des en-têtes partagés (lecture seule) et du client
COPY services/libs/shared-headers /app/services/libs/shared-headers
COPY tools/telemetryClient /app/tools/telemetryClient

WORKDIR /app/tools/telemetryClient

# Compilation statique pour portabilité absolue
RUN rm -rf build build-docker && \
    cmake -B build-docker -S . -DSTATIC=ON && \
    cmake --build build-docker --parallel

# ==============================================================================
# Image finale ultra-légère (~10 Mo)
# ==============================================================================
FROM alpine:latest

WORKDIR /app

# Binaire compilé et assets web
COPY --from=builder /app/tools/telemetryClient/build-docker/telemetry_client /app/telemetry_client
COPY --from=builder /app/tools/telemetryClient/web /app/web

EXPOSE 8080

ENV PICAR_HOST=host.docker.internal \
    PICAR_PORT=91 \
    WEB_PORT=8080 \
    WEB_ROOT=/app/web

ENTRYPOINT ["/app/telemetry_client"]
