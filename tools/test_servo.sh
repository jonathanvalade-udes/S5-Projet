#!/bin/bash
# ==============================================================================
# Script de test et calibration du servomoteur de direction (PiCar-S / PCA9685 Canal 0)
# ==============================================================================
#
# Utilisation :
#   ./test_servo.sh [angle] [offset]
#   ./test_servo.sh --calib               (Mode interactif de calibration en direct)
#
# Exemples :
#   ./test_servo.sh                       # Centre avec offset enregistré
#   ./test_servo.sh 90                    # Neutre 90° standard RC
#   ./test_servo.sh 90 +4                 # Neutre 90° avec +4° d'offset logiciel
#   ./test_servo.sh droite                # Braquage max à droite (+45°)
#   ./test_servo.sh gauche                # Braquage max à gauche (-45°)
#   ./test_servo.sh centre -3.5           # Centre avec -3.5° d'offset
#   ./test_servo.sh --calib               # Ajustez en direct avec [+] et [-] et sauvez avec [s]
#
# Fichier d'offset persistant :
#   /etc/picar_steering_offset
# ==============================================================================

I2C_BUS=1
PCA_ADDR=0x40
CH0_ON_L=0x06
CH0_ON_H=0x07
CH0_OFF_L=0x08
CH0_OFF_H=0x09
OFFSET_FILE="/etc/picar_steering_offset"

# Vérification présence i2cset
if ! command -v i2cset >/dev/null 2>&1; then
    echo "[ERREUR] i2cset est introuvable. Assurez-vous d'avoir i2c-tools installé."
    exit 1
fi

# Alerte si le daemon Motors tourne déjà
if command -v pidof >/dev/null 2>&1 && pidof Motors >/dev/null 2>&1; then
    echo "[ATTENTION] Le processus 'Motors' est actif en arrière-plan !"
    echo "            Il risque d'écraser la commande PWM à chaque cycle (50 Hz)."
    echo "            Pour l'arrêter : killall Motors"
    echo ""
fi

# Lecture de l'offset persistant si existant
SAVED_OFFSET=0.0
if [ -f "$OFFSET_FILE" ]; then
    SAVED_OFFSET=$(cat "$OFFSET_FILE" 2>/dev/null | tr -d ' \n\r' || echo 0.0)
    # Valider format numérique
    if ! awk "BEGIN { exit ($SAVED_OFFSET == $SAVED_OFFSET ? 0 : 1) }" 2>/dev/null; then
        SAVED_OFFSET=0.0
    fi
fi

# Initialisation matérielle du PCA9685 à 50 Hz
init_pca9685() {
    i2cset -y $I2C_BUS $PCA_ADDR 0x00 0x10 >/dev/null 2>&1 # Sleep
    i2cset -y $I2C_BUS $PCA_ADDR 0xFE 0x79 >/dev/null 2>&1 # Prescale 50 Hz (121)
    i2cset -y $I2C_BUS $PCA_ADDR 0x00 0x20 >/dev/null 2>&1 # Auto-increment
    sleep 0.01
    i2cset -y $I2C_BUS $PCA_ADDR 0x01 0x04 >/dev/null 2>&1 # Totem-pole Mode2
}

# Fonction appliquant une impulsion PWM sur le canal 0 (en ticks 0-4095)
apply_pwm_ticks() {
    local ticks="$1"
    # Bornes matérielles strictes de sécurité (1.0 ms ~ 205 ticks, 2.0 ms ~ 410 ticks)
    if [ "$ticks" -lt 195 ]; then ticks=195; fi
    if [ "$ticks" -gt 425 ]; then ticks=425; fi

    local off_l=$(( ticks & 0xFF ))
    local off_h=$(( (ticks >> 8) & 0x0F ))
    local hex_l=$(printf "0x%02X" $off_l)
    local hex_h=$(printf "0x%02X" $off_h)

    i2cset -y $I2C_BUS $PCA_ADDR $CH0_ON_L 0x00 >/dev/null 2>&1
    i2cset -y $I2C_BUS $PCA_ADDR $CH0_ON_H 0x00 >/dev/null 2>&1
    i2cset -y $I2C_BUS $PCA_ADDR $CH0_OFF_L $hex_l >/dev/null 2>&1
    i2cset -y $I2C_BUS $PCA_ADDR $CH0_OFF_H $hex_h >/dev/null 2>&1
}

# Calcul des ticks à partir d'un angle relatif [-45, +45] et d'un offset
calc_steering_ticks() {
    local angle="$1"
    local offset="$2"
    awk -v a="$angle" -v o="$offset" 'BEGIN {
        total = a + o;
        if (total < -45.0) total = -45.0;
        if (total > 45.0)  total = 45.0;
        ticks = 307.2 + (total / 45.0) * 102.4;
        printf "%.0f\n", ticks;
    }'
}

calc_steering_pulse_ms() {
    local angle="$1"
    local offset="$2"
    awk -v a="$angle" -v o="$offset" 'BEGIN {
        total = a + o;
        if (total < -45.0) total = -45.0;
        if (total > 45.0)  total = 45.0;
        pulse = 1.50 + (total / 45.0) * 0.50;
        printf "%.3f\n", pulse;
    }'
}

# Fonction de nettoyage appelée à la sortie (Ctrl+C, kill ou fin)
cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "[*] Arrêt détecté. Rétablissement et mise au repos..."
    
    # Recentrage avec offset pour éviter les contraintes mécaniques
    local center_ticks
    center_ticks=$(calc_steering_ticks 0.0 "$CURRENT_OFFSET")
    apply_pwm_ticks "$center_ticks"
    sleep 0.4

    # Coupure complète du signal PWM sur le canal 0 (Full OFF : bit 4 de OFF_H = 0x10)
    i2cset -y $I2C_BUS $PCA_ADDR $CH0_OFF_H 0x10 >/dev/null 2>&1
    
    echo "[✓] Servomoteur recentré et signal PWM coupé. Roues libres."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

CURRENT_OFFSET="$SAVED_OFFSET"

# ------------------------------------------------------------------------------
# MODE 1 : CALIBRATION INTERACTIVE EN DIRECT (--calib)
# ------------------------------------------------------------------------------
if [ "$1" = "--calib" ] || [ "$1" = "-c" ] || [ "$1" = "calib" ]; then
    init_pca9685
    echo "============================================================"
    echo "       MODE CALIBRATION INTERACTIVE DU SERVO PICAR-S        "
    echo "============================================================"
    echo " Offset initial sauvegardé : ${CURRENT_OFFSET}°"
    echo ""
    echo " Commandes clavier en direct :"
    echo "   [+] ou [d] : Décaler de +1.0° vers la droite"
    echo "   [-] ou [g] : Décaler de -1.0° vers la gauche"
    echo "   [D]        : Ajustement fin +0.2° vers la droite"
    echo "   [G]        : Ajustement fin -0.2° vers la gauche"
    echo "   [0]        : Réinitialiser l'offset à 0.0°"
    echo "   [s]        : Sauvegarder l'offset dans $OFFSET_FILE"
    echo "   [q]        : Quitter (sauvegarde optionnelle)"
    echo "============================================================"

    while true; do
        TICKS=$(calc_steering_ticks 0.0 "$CURRENT_OFFSET")
        PULSE=$(calc_steering_pulse_ms 0.0 "$CURRENT_OFFSET")
        apply_pwm_ticks "$TICKS"

        printf "\r-> Offset actuel : %-6s° | Impulsion : %s ms | Ticks : %d   " \
               "$CURRENT_OFFSET" "$PULSE" "$TICKS"

        read -rsn1 KEY
        case "$KEY" in
            "+"|"d")
                CURRENT_OFFSET=$(awk -v o="$CURRENT_OFFSET" 'BEGIN { printf "%.1f", o + 1.0 }')
                ;;
            "-"|"g")
                CURRENT_OFFSET=$(awk -v o="$CURRENT_OFFSET" 'BEGIN { printf "%.1f", o - 1.0 }')
                ;;
            "D")
                CURRENT_OFFSET=$(awk -v o="$CURRENT_OFFSET" 'BEGIN { printf "%.1f", o + 0.2 }')
                ;;
            "G")
                CURRENT_OFFSET=$(awk -v o="$CURRENT_OFFSET" 'BEGIN { printf "%.1f", o - 0.2 }')
                ;;
            "0")
                CURRENT_OFFSET="0.0"
                ;;
            "s"|"S")
                echo "$CURRENT_OFFSET" > "$OFFSET_FILE"
                echo ""
                echo "[✓] Offset sauvegardé avec succès dans $OFFSET_FILE : ${CURRENT_OFFSET}°"
                ;;
            "q"|"Q")
                echo ""
                cleanup
                ;;
        esac
    done
fi

# ------------------------------------------------------------------------------
# MODE 2 : POSITIONNEMENT AVEC OFFSET OPTIONNEL
# ------------------------------------------------------------------------------
ARG_ANGLE="${1:-90}"
ARG_OFFSET="${2:-}"

# Si un offset explicite est passé en argument 2, l'utiliser, sinon prendre l'offset sauvegardé
if [ -n "$ARG_OFFSET" ]; then
    CURRENT_OFFSET=$(echo "$ARG_OFFSET" | tr -d '+')
fi

ARG_LOWER=$(echo "$ARG_ANGLE" | tr '[:upper:]' '[:lower:]')
REL_ANGLE=0.0
DESC=""

case "$ARG_LOWER" in
    "center"|"centre"|"neutre"|"straight"|"0")
        REL_ANGLE=0.0
        DESC="Centre / Roues droites (0° braquage)"
        ;;
    "left"|"gauche")
        REL_ANGLE=-45.0
        DESC="Braquage max GAUCHE (-45° braquage)"
        ;;
    "right"|"droite")
        REL_ANGLE=45.0
        DESC="Braquage max DROITE (+45° braquage)"
        ;;
    *)
        VAL="$ARG_ANGLE"
        if [ "$VAL" -eq 90 ] 2>/dev/null; then
            REL_ANGLE=0.0
            DESC="90° servo standard (position neutre RC = roues droites)"
        elif [ "$VAL" -ge -45 ] 2>/dev/null && [ "$VAL" -le 45 ] 2>/dev/null; then
            REL_ANGLE="$VAL"
            DESC="Braquage relatif de ${VAL}°"
        elif [ "$VAL" -ge 0 ] 2>/dev/null && [ "$VAL" -le 180 ] 2>/dev/null; then
            # Conversion 0-180 vers relatif -45/+45 (0=gauche -45, 90=centre 0, 180=droite +45)
            REL_ANGLE=$(awk -v v="$VAL" 'BEGIN { printf "%.1f", (v - 90.0) * 0.50 }')
            DESC="Angle absolu servo ${VAL}° (équivalent braquage ${REL_ANGLE}°)"
        else
            echo "[AVERTISSEMENT] Angle '$VAL' hors limites usuelles."
            REL_ANGLE=0.0
            DESC="Valeur de sécurité : centre (0°)"
        fi
        ;;
esac

TICKS=$(calc_steering_ticks "$REL_ANGLE" "$CURRENT_OFFSET")
PULSE_MS=$(calc_steering_pulse_ms "$REL_ANGLE" "$CURRENT_OFFSET")
EFF_ANGLE=$(awk -v a="$REL_ANGLE" -v o="$CURRENT_OFFSET" 'BEGIN { printf "%.1f", a + o }')

echo "============================================================"
echo "    TEST DU SERVOMOTEUR DE DIRECTION (PCA9685 Canal 0)     "
echo "============================================================"
echo " Consigne     : $DESC"
echo " Offset soft  : ${CURRENT_OFFSET}° (appliqué au servomoteur)"
echo " Angle final  : ${EFF_ANGLE}° effectif"
echo " Impulsion    : ${PULSE_MS} ms (Ticks: ${TICKS} / 4096)"
echo "============================================================"

init_pca9685
apply_pwm_ticks "$TICKS"

echo "[+] Servomoteur activé et maintenu dans cette position."
echo "[+] Pour calibrer en direct avec les touches : test_servo --calib"
echo "[+] Pour quitter et redresser/couper le servomoteur, appuyez sur [Ctrl + C]."
echo ""

while true; do
    sleep 1 &
    wait $!
done
