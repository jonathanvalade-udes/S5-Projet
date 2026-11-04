#!/usr/bin/env bash
# ==============================================================================
# Script de lancement de l'optimiseur Optuna pour PiCar-S
# Usage :
#   ./tools/optimizer/run_optimization.sh           # Mode rapide Python (~5s, 300 essais)
#   ./tools/optimizer/run_optimization.sh --godot   # Mode Godot 3D Headless (~2m, 100 essais)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Vérifier et créer le venv si absent
if [ ! -d "${SCRIPT_DIR}/.venv" ]; then
    echo "==> Création de l'environnement virtuel Python (.venv)..."
    python3 -m venv "${SCRIPT_DIR}/.venv"
    echo "==> Installation d'Optuna et CMA-ES..."
    "${SCRIPT_DIR}/.venv/bin/pip" install --quiet optuna cmaes
fi

PYTHON="${SCRIPT_DIR}/.venv/bin/python3"
MODE="fast"
TRIALS=300

if [ "${1:-}" = "--godot" ]; then
    MODE="godot"
    TRIALS="${2:-100}"
elif [ -n "${1:-}" ]; then
    TRIALS="$1"
fi

cd "${ROOT_DIR}"
exec "${PYTHON}" "${SCRIPT_DIR}/optimize_godot.py" --mode "${MODE}" --trials "${TRIALS}"
