#!/usr/bin/env bash
# ==============================================================================
# Script de lancement du simulateur Webots pour PiCar-S (Projet S5)
# ==============================================================================
# Détecte automatiquement l'installation de Webots (macOS et Linux),
# configure les variables d'environnement pour le contrôleur Python et lance
# le monde de simulation picar_s_track.wbt.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORLD_FILE="${SCRIPT_DIR}/worlds/picar_s_track.wbt"
TEXTURE_FILE="${SCRIPT_DIR}/worlds/textures/line_track.png"
TEXTURE_SCRIPT="${SCRIPT_DIR}/tools/generate_track_texture.py"

# Variables par défaut
SIMBRIDGE_HOST="${SIMBRIDGE_HOST:-127.0.0.1}"
SIMBRIDGE_PORT="${SIMBRIDGE_PORT:-9000}"
WEBOTS_AUTO_START="${WEBOTS_AUTO_START:-0}"
WEBOTS_DYNAMICS="${WEBOTS_DYNAMICS:-1}"   # modèles moteur/direction/bille (0 = consigne appliquée telle quelle)
WEBOTS_MODE="${WEBOTS_MODE:-realtime}"

# Couleurs pour le terminal
RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
BLUE=$'\033[0;34m'
YELLOW=$'\033[1;33m'
BOLD=$'\033[1m'
NC=$'\033[0m' # No Color

print_usage() {
    cat << EOF
${BOLD}Usage:${NC} $0 [options] [-- [webots-options]]

${BOLD}Options:${NC}
  -h, --help               Affiche ce message d'aide et quitte.
  -s, --auto-start         Active l'auto-démarrage du robot après 2 secondes (WEBOTS_AUTO_START=1).
  -H, --host <IP>          Adresse IP du service SimBridge (défaut: 127.0.0.1).
  -p, --port <PORT>        Port TCP du service SimBridge (défaut: 9000).
  -m, --mode <MODE>        Mode de démarrage Webots: realtime, pause, fast (défaut: realtime).
  --world <FILE>           Spécifie un fichier .wbt alternatif.

${BOLD}Commandes clavier interactives une fois dans Webots:${NC}
  - ${BOLD}'S'${NC}     : Démarre le suivi de ligne (trigger_start -> FSM).
  - ${BOLD}'Espace'${NC}: Arrêt d'urgence immédiat (emergency_stop -> FSM).

${BOLD}Exemples:${NC}
  $0                       # Lancement standard en mode temps réel
  $0 --auto-start          # Lancement avec démarrage autonome
  $0 -H 192.168.1.50       # Connexion vers SimBridge distant
  $0 --mode=pause          # Démarrer en pause
  $0 -- --batch            # Passer des arguments natifs à Webots
EOF
}

# 1. Analyse des arguments de ligne de commande
EXTRA_WEBOTS_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        -s|--auto-start)
            WEBOTS_AUTO_START=1
            shift
            ;;
        -H|--host)
            SIMBRIDGE_HOST="$2"
            shift 2
            ;;
        -p|--port)
            SIMBRIDGE_PORT="$2"
            shift 2
            ;;
        -m|--mode)
            WEBOTS_MODE="$2"
            shift 2
            ;;
        --mode=*)
            WEBOTS_MODE="${1#*=}"
            shift
            ;;
        --world)
            WORLD_FILE="$2"
            shift 2
            ;;
        --)
            shift
            EXTRA_WEBOTS_ARGS+=("$@")
            break
            ;;
        *)
            EXTRA_WEBOTS_ARGS+=("$1")
            shift
            ;;
    esac
done

# 2. Détection du binaire Webots selon le système d'exploitation
WEBOTS_BIN=""

detect_webots() {
    # 1. Vérification de la variable d'environnement WEBOTS_HOME
    if [[ -n "${WEBOTS_HOME:-}" && -x "${WEBOTS_HOME}/webots" ]]; then
        WEBOTS_BIN="${WEBOTS_HOME}/webots"
        return 0
    fi

    # 2. Détection sous macOS
    if [[ "$(uname)" == "Darwin" ]]; then
        local mac_paths=(
            "/Applications/Webots.app/Contents/MacOS/webots"
            "${HOME}/Applications/Webots.app/Contents/MacOS/webots"
        )
        for p in "${mac_paths[@]}"; do
            if [[ -x "$p" ]]; then
                WEBOTS_BIN="$p"
                return 0
            fi
        done
    fi

    # 3. Détection dans le PATH standard (Linux / macOS)
    if command -v webots >/dev/null 2>&1; then
        WEBOTS_BIN="$(command -v webots)"
        return 0
    fi

    # 4. Chemins standards sous Linux
    local linux_paths=(
        "/usr/local/bin/webots"
        "/usr/bin/webots"
        "/snap/bin/webots"
    )
    for p in "${linux_paths[@]}"; do
        if [[ -x "$p" ]]; then
            WEBOTS_BIN="$p"
            return 0
        fi
    done

    return 1
}

echo -e "${BLUE}==>${NC} Recherche de l'exécutable Webots..."
if ! detect_webots; then
    echo -e "${RED}[ERREUR]${NC} Impossible de trouver l'exécutable Webots sur cette machine."
    echo -e "Veuillez vous assurer que Webots est installé dans /Applications/Webots.app (macOS)"
    echo -e "ou que la commande 'webots' est accessible dans le PATH (Linux)."
    echo -e "Vous pouvez également définir la variable d'environnement WEBOTS_HOME."
    exit 1
fi
echo -e "${GREEN}==>${NC} Exécutable Webots trouvé : ${BOLD}${WEBOTS_BIN}${NC}"

# 3. Vérification des fichiers du monde et de la texture
if [[ ! -f "$WORLD_FILE" ]]; then
    echo -e "${RED}[ERREUR]${NC} Le fichier du monde est introuvable : ${WORLD_FILE}"
    exit 1
fi

# Régénération de la texture de circuit si elle est absente
if [[ ! -f "$TEXTURE_FILE" ]]; then
    echo -e "${YELLOW}==>${NC} Texture du circuit manquante, génération automatique via ${TEXTURE_SCRIPT}..."
    if command -v python3 >/dev/null 2>&1 && [[ -f "$TEXTURE_SCRIPT" ]]; then
        python3 "$TEXTURE_SCRIPT"
    else
        echo -e "${RED}[AVERTISSEMENT]${NC} Impossible de générer la texture automatiquement."
    fi
fi

# 4. Configuration de l'environnement pour le contrôleur
export SIMBRIDGE_HOST
export SIMBRIDGE_PORT
export WEBOTS_AUTO_START
export WEBOTS_DYNAMICS

echo -e "${BLUE}==>${NC} Configuration de la liaison SimBridge :"
echo -e "    - Adresse IP : ${BOLD}${SIMBRIDGE_HOST}${NC}"
echo -e "    - Port TCP   : ${BOLD}${SIMBRIDGE_PORT}${NC}"
echo -e "    - Auto-start : ${BOLD}${WEBOTS_AUTO_START}${NC}"
echo -e "    - Mode       : ${BOLD}${WEBOTS_MODE}${NC}"
echo -e "    - Monde      : ${BOLD}${WORLD_FILE}${NC}"
echo -e ""
echo -e "${GREEN}==>${NC} Commandes interactives dans la fenêtre Webots :"
echo -e "    - Touche ${BOLD}'S'${NC}      : Lancer le suivi de ligne (trigger_start)"
echo -e "    - Touche ${BOLD}'Espace'${NC} : Arrêt d'urgence (emergency_stop)"
echo -e ""

# 5. Lancement de Webots
WEBOTS_CMD=(
    "$WEBOTS_BIN"
    "--mode=${WEBOTS_MODE}"
    "$WORLD_FILE"
)

if [[ ${#EXTRA_WEBOTS_ARGS[@]} -gt 0 ]]; then
    WEBOTS_CMD+=("${EXTRA_WEBOTS_ARGS[@]}")
fi

echo -e "${BLUE}==>${NC} Démarrage de Webots..."
exec "${WEBOTS_CMD[@]}"
