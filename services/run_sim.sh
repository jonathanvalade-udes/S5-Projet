#!/usr/bin/env bash
# Lance la pile C++ en mode simulation Godot (a executer sous Linux / WSL2).
#
#   SimBridge (TCP :9000)  <->  SHM /picar_shm  <->  FSM
#
# Sensors et Motors NE sont PAS lances : en simulation, Godot fournit les
# capteurs (via SimBridge) et la voiture est le moteur. Les lancer ecraserait
# les donnees Godot avec les valeurs de repli des drivers PiCar.
#
# Variables d'environnement :
#   BUILD_DIR   repertoire de build (defaut: $HOME/picar-build, hors /mnt/c : plus rapide)
#   PORT        port TCP du SimBridge (defaut: 9000)
#   TELEMETRY=1 lance aussi TelemetryService (--no-ui)
#   REBUILD=1   force la recompilation
#   AVOID_SIDE  cote de contournement des obstacles : gauche | droite (defaut: droite)
#
# Options :
#   --direction gauche|droite   (ou -d) cote par lequel la voiture contourne les obstacles
#   --rebuild                   force la recompilation
set -u

while [ $# -gt 0 ]; do
    case "$1" in
        --direction|-d) AVOID_SIDE="${2:-}"; shift 2 ;;
        --direction=*)  AVOID_SIDE="${1#*=}"; shift ;;
        --rebuild)      REBUILD=1; shift ;;
        -h|--help)      sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//' | head -n -1; exit 0 ;;
        *) echo "[run_sim] option inconnue : $1 (voir --help)"; exit 1 ;;
    esac
done
export AVOID_SIDE="${AVOID_SIDE:-droite}"

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$HOME/picar-build}"
PORT="${PORT:-9000}"

# cmake peut vivre dans un venv utilisateur si apt n'est pas disponible
[ -d "$HOME/.picar-venv/bin" ] && export PATH="$HOME/.picar-venv/bin:$PATH"

if [ "${REBUILD:-0}" = "1" ] || [ ! -x "$BUILD_DIR/SimBridge/SimBridge" ] || [ ! -x "$BUILD_DIR/FSM/FSM" ]; then
    echo "[run_sim] Build dans $BUILD_DIR ..."
    command -v cmake >/dev/null || { echo "[run_sim] cmake introuvable"; exit 1; }
    cmake -S "$SRC_DIR" -B "$BUILD_DIR" >/dev/null && cmake --build "$BUILD_DIR" -j"$(nproc)" || exit 1
fi

# Un segment SHM perime (processus tue) serait reutilise avec un mutex possiblement verrouille.
rm -f /dev/shm/picar_shm
rm -rf /dev/shm/car

PIDS=()
cleanup() {
    trap - INT TERM EXIT
    echo; echo "[run_sim] Arret des services..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
    wait 2>/dev/null
    rm -f /dev/shm/picar_shm
    rm -rf /dev/shm/car
}
trap cleanup INT TERM EXIT

# Ordre important : SimBridge cree la SHM (et la supprime a sa fermeture).
"$BUILD_DIR/SimBridge/SimBridge" --sim --port "$PORT" & PIDS+=($!)
sleep 0.5
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo "[run_sim] SimBridge a echoue au demarrage (port $PORT deja utilise ?)"
    exit 1
fi

"$BUILD_DIR/FSM/FSM" & PIDS+=($!)

if [ "${TELEMETRY:-0}" = "1" ]; then
    "$BUILD_DIR/TelemetryService/TelemetryService" --no-ui & PIDS+=($!)
fi

echo "[run_sim] Pile C++ prete. Dans Godot : cocher 'Localhost' (ou 127.0.0.1:$PORT). Ctrl+C pour arreter."
# Quitte des qu'un des services s'arrete
wait -n "${PIDS[@]}"
echo "[run_sim] Un service s'est arrete."
