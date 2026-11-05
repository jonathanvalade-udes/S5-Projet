#!/usr/bin/env bash
# ==============================================================================
# Compile les services pour le robot (ARM64, statique) et les envoie au Raspberry Pi par SSH.
# A lancer depuis WSL ou Git Bash (pas PowerShell). Docker doit etre actif pour la compilation.
#
# Usage :  ./services/deploy.sh [options]
#   --host <ip>    adresse du robot (defaut : $PI_HOST ou 10.155.175.111)
#   --no-build     n'effectue pas la compilation : envoie les binaires deja presents dans build-embedded/
#   --config       envoie aussi picar.conf (celui du depot) : ECRASE celui du robot
#   --run          lance /root/init sur le robot apres l'envoi (reste attache : Ctrl+C pour arreter)
#   -h, --help     affiche cette aide
#
# Etapes : compilation -> arret des services sur le robot -> copie dans /root/ -> chmod +x -> (lancement).
# Astuce : "ssh-copy-id root@<ip>" evite de retaper le mot de passe a chaque connexion.
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-embedded"

HOST="${PI_HOST:-10.155.175.111}"
DO_BUILD=true
SEND_CONFIG=false
RUN_AFTER=false

usage() { sed -n '2,/^# =====.*$/p' "$0" | sed -n '2,$p' | sed 's/^# \{0,1\}//' | sed '$d'; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)     HOST="${2:?--host demande une adresse}"; shift 2 ;;
        --host=*)   HOST="${1#*=}"; shift ;;
        --no-build) DO_BUILD=false; shift ;;
        --config)   SEND_CONFIG=true; shift ;;
        --run)      RUN_AFTER=true; shift ;;
        -h|--help)  usage 0 ;;
        *) echo "[deploy] option inconnue : $1 (voir --help)"; exit 1 ;;
    esac
done

TARGET="root@${HOST}"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8)

# --- 1. Compilation --------------------------------------------------------------------------------------------------
if $DO_BUILD; then
    echo "[deploy] 1/4  Compilation ARM64 statique (Docker)..."
    "${SCRIPT_DIR}/build.sh" --embedded --static
else
    echo "[deploy] 1/4  Compilation ignoree (--no-build)"
fi

# Binaires a envoyer : "chemin dans build-embedded" -> nom sur le robot (ServiceManager/init devient /root/init)
FILES=(
    "${BUILD_DIR}/ServiceManager/init"
    "${BUILD_DIR}/FSM/FSM"
    "${BUILD_DIR}/Motors/Motors"
    "${BUILD_DIR}/Sensors/Sensors"
    "${BUILD_DIR}/TelemetryService/TelemetryService"
)
# PeripheralsService : envoye seulement s'il a ete compile (le robot en a deja un sinon)
[ -f "${BUILD_DIR}/PeripheralsService/PeripheralsService" ] && FILES+=("${BUILD_DIR}/PeripheralsService/PeripheralsService")
$SEND_CONFIG && FILES+=("${REPO_DIR}/picar.conf")

for f in "${FILES[@]}"; do
    [ -f "$f" ] || { echo "[deploy] ERREUR : fichier introuvable : $f"; echo "          (compile d'abord, ou verifie le nom du service)"; exit 1; }
done

# --- 2. Arret des services sur le robot ------------------------------------------------------------------------------
# Un binaire en cours d'execution ne peut pas etre ecrase ("Text file busy").
echo "[deploy] 2/4  Arret des services sur ${TARGET} (et sauvegarde des anciens binaires dans /root/backup_prev)..."
ssh "${SSH_OPTS[@]}" "${TARGET}" \
    'killall -2 init FSM Sensors Motors TelemetryService PeripheralsService 2>/dev/null; sleep 0.5;
     killall -9 init FSM Sensors Motors TelemetryService PeripheralsService 2>/dev/null;
     mkdir -p /root/backup_prev && cp -p /root/init /root/FSM /root/Motors /root/Sensors /root/TelemetryService /root/PeripheralsService /root/picar.conf /root/backup_prev/ 2>/dev/null; true'

# --- 3. Copie --------------------------------------------------------------------------------------------------------
echo "[deploy] 3/4  Envoi de ${#FILES[@]} fichier(s) vers ${TARGET}:/root/ ..."
scp "${SSH_OPTS[@]}" "${FILES[@]}" "${TARGET}:/root/"

ssh "${SSH_OPTS[@]}" "${TARGET}" 'chmod +x /root/init /root/FSM /root/Motors /root/Sensors /root/TelemetryService; ls -l /root'

# --- 4. Lancement ----------------------------------------------------------------------------------------------------
if $RUN_AFTER; then
    echo "[deploy] 4/4  Lancement de /root/init (Ctrl+C pour arreter)..."
    exec ssh -t "${SSH_OPTS[@]}" "${TARGET}" '/root/init'
else
    echo "[deploy] 4/4  Termine. Pour lancer sur le robot :  ssh ${TARGET} /root/init"
    echo "         (ou relance avec --run)"
fi
