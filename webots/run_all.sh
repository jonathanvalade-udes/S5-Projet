#!/usr/bin/env bash
# ==============================================================================
# Lance toute la simulation en une commande : pile C++ (SimBridge + FSM, Docker) puis Webots.
# ==============================================================================
#   ./webots/run_all.sh                    # circuit Godot sans obstacles, depart automatique
#   ./webots/run_all.sh --obstacles        # circuit Godot avec les 2 obstacles
#   ./webots/run_all.sh --oval             # ovale simple (sans bille)
#   ./webots/run_all.sh --manual           # pas de depart automatique (touche S dans Webots)
#   ./webots/run_all.sh --rebuild          # recompile la pile C++ (apres une modification du code C++)
#   ./webots/run_all.sh -- --mode=fast     # options supplementaires passees a run_webots.sh / Webots
#
# La pile est (re)demarree a chaque lancement : le FSM repart de l'etat initial. Fermer Webots arrete la pile.
# Variables : KEEP_STACK=1 laisse la pile tourner apres la fermeture de Webots ; WEBOTS_DEBUG=1, WEBOTS_TRUTH_LOG=...
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICES_DIR="$(cd "${SCRIPT_DIR}/../services" && pwd)"
WORLD="${SCRIPT_DIR}/worlds/picar_s_course_no_obstacles.wbt"
AUTO_START="--auto-start"
REBUILD=0
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --obstacles) WORLD="${SCRIPT_DIR}/worlds/picar_s_course.wbt"; shift ;;
        --oval)      WORLD="${SCRIPT_DIR}/worlds/picar_s_track.wbt"; shift ;;
        --world)     WORLD="$2"; shift 2 ;;
        --manual)    AUTO_START=""; shift ;;
        --rebuild)   REBUILD=1; shift ;;
        -h|--help)   sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//' | sed '$d'; exit 0 ;;
        --)          shift; EXTRA+=("$@"); break ;;
        *)           EXTRA+=("$1"); shift ;;
    esac
done

command -v docker >/dev/null || { echo "[run_all] docker introuvable"; exit 1; }
docker info >/dev/null 2>&1 || { echo "[run_all] Docker n'est pas demarre (ouvrir Docker Desktop)"; exit 1; }

stop_stack() {
    if [[ "${KEEP_STACK:-0}" != "1" ]]; then
        echo "[run_all] Arret de la pile C++..."
        (cd "$SERVICES_DIR" && docker compose stop simulation >/dev/null 2>&1) || true
    fi
}
trap stop_stack EXIT

echo "[run_all] Demarrage de la pile C++ (SimBridge + FSM) dans Docker..."
STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
# Conteneur existant : simple redemarrage, rapide (il garde sa compilation ; le code C++ modifie n'est PAS recompile).
# --rebuild ou premier lancement : conteneur recree, la pile est recompilee (une ou deux minutes).
if [[ "$REBUILD" != "1" ]] && docker ps -a --format '{{.Names}}' | grep -x picar-sim >/dev/null; then
    (cd "$SERVICES_DIR" && docker compose restart simulation >/dev/null)
else
    (cd "$SERVICES_DIR" && docker compose up -d --build --force-recreate simulation >/dev/null)
fi

# Attente de la pile (premier lancement ou --rebuild : compilation dans le conteneur, une ou deux minutes).
for _ in $(seq 1 300); do
    if docker logs --since "$STARTED_AT" picar-sim 2>&1 | grep "Pile C++ prete" >/dev/null; then
        break
    fi
    if ! docker ps --format '{{.Names}}' | grep -x picar-sim >/dev/null; then
        echo "[run_all] La pile s'est arretee :"; docker logs --since "$STARTED_AT" picar-sim 2>&1 | tail -20; exit 1
    fi
    sleep 1
done
docker logs --since "$STARTED_AT" picar-sim 2>&1 | grep "Pile C++ prete" >/dev/null || { echo "[run_all] Pile pas prete apres 5 min"; exit 1; }
docker logs --since "$STARTED_AT" picar-sim 2>&1 | grep "surcharges" || true
echo "[run_all] Pile prete. Journal en direct : docker logs -f picar-sim"

# Webots au premier plan : fermer sa fenetre termine ce script (et arrete la pile).
"${SCRIPT_DIR}/run_webots.sh" --world "$WORLD" ${AUTO_START} ${EXTRA[@]+"${EXTRA[@]}"}
