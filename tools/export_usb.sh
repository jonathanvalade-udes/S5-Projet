#!/bin/bash
# Script d'export de télémétrie vers clé USB
# Usage: ./tools/export_usb.sh [chemin_optionnel_usb]

SRC="/tmp/telemetry.csv"

if [ ! -f "$SRC" ]; then
    echo "❌ Erreur : Aucun fichier de télémétrie trouvé dans $SRC"
    exit 1
fi

DEST_DIR="$1"

if [ -z "$DEST_DIR" ]; then
    # Auto-détection de la clé USB dans /media ou /mnt
    DEST_DIR=$(grep -E '/media/|/mnt/' /proc/mounts | awk '{print $2}' | grep -v -E '/mnt/wsl|/mnt/wslg' | head -n 1)
fi

if [ -z "$DEST_DIR" ] || [ ! -d "$DEST_DIR" ]; then
    echo "⚠️ Aucune clé USB détectée dans /media ou /mnt."
    echo "Veuillez spécifier le point de montage manuellement :"
    echo "  ./tools/export_usb.sh /media/pi/VOTRE_CLE"
    exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DEST_FILE="$DEST_DIR/telemetry_run_${TIMESTAMP}.csv"

cp "$SRC" "$DEST_FILE"
sync

echo "✅ Télémétrie exportée avec succès :"
echo "   Source : $SRC"
echo "   Cible  : $DEST_FILE"
echo "   Lignes : $(wc -l < "$DEST_FILE")"
echo "Vous pouvez maintenant retirer la clé USB en toute sécurité."

