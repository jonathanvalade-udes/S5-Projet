#!/bin/bash
# ==============================================================================
# Outil de test et calibration en direct du servomoteur (PiCar-S / Canal 0)
# ==============================================================================
#
# Utilisation :
#   ./test_servo.sh [angle_initial]
#
# Exemples :
#   ./test_servo.sh          # Démarre au neutre (0°)
#   ./test_servo.sh -5       # Démarre avec un angle de -5°
#
# Contrôles clavier en direct :
#   [+] ou [d] : Décaler de +1.0° vers la droite
#   [-] ou [g] : Décaler de -1.0° vers la gauche
#   [D]        : Ajustement fin +0.2° vers la droite
#   [G]        : Ajustement fin -0.2° vers la gauche
#   [0]        : Réinitialiser à 0.0° (centre théorique)
#   [q] / ^C   : Quitter (affiche l'angle final, recentre et coupe le PWM)
# ==============================================================================

I2C_BUS=1
PCA_ADDR=0x40
CH0_ON_L=0x06
CH0_ON_H=0x07
CH0_OFF_L=0x08
CH0_OFF_H=0x09

# Vérification présence i2cset
if ! command -v i2cset >/dev/null 2>&1; then
    echo "[ERREUR] i2cset est introuvable. Assurez-vous d'avoir i2c-tools installé."
    exit 1
fi

# Alerte si le daemon Motors tourne déjà
if command -v pidof >/dev/null 2>&1 && pidof Motors >/dev/null 2>&1; then
    echo "[ATTENTION] Le processus 'Motors' est actif en arrière-plan !"
    echo "            Il risque d'écraser la commande PWM à chaque cycle (50 Hz)."
    echo "            Pour l'arrêter temporairement : killall Motors"
    echo ""
fi

# Initialisation matérielle du PCA9685 à 50 Hz
init_pca9685() {
    i2cset -y $I2C_BUS $PCA_ADDR 0x00 0x10 >/dev/null 2>&1 # Sleep
    i2cset -y $I2C_BUS $PCA_ADDR 0xFE 0x79 >/dev/null 2>&1 # Prescale 50 Hz (121)
    i2cset -y $I2C_BUS $PCA_ADDR 0x00 0x20 >/dev/null 2>&1 # Auto-increment
    sleep 0.01
    i2cset -y $I2C_BUS $PCA_ADDR 0x01 0x04 >/dev/null 2>&1 # Totem-pole Mode2
}

# Fonction appliquant une impulsion PWM sur le canal 0
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

# Calcul des ticks à partir d'un angle relatif [-45, +45]
calc_ticks() {
    local deg="$1"
    awk -v a="$deg" 'BEGIN {
        if (a < -45.0) a = -45.0;
        if (a > 45.0)  a = 45.0;
        ticks = 307.2 + (a / 45.0) * 102.4;
        printf "%.0f\n", ticks;
    }'
}

calc_pulse_ms() {
    local deg="$1"
    awk -v a="$deg" 'BEGIN {
        if (a < -45.0) a = -45.0;
        if (a > 45.0)  a = 45.0;
        pulse = 1.50 + (a / 45.0) * 0.50;
        printf "%.3f\n", pulse;
    }'
}

# Nettoyage à la sortie : recentre puis coupe totalement le PWM
cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo ""
    echo "============================================================"
    echo "  Angle final mesuré : ${CURRENT_ANGLE}°"
    echo "  (Vous pouvez reporter cette valeur dans DEFAULT_STEERING_OFFSET_DEG)"
    echo "============================================================"
    echo "[*] Recentrage du servomoteur..."
    
    local center_ticks
    center_ticks=$(calc_ticks 0.0)
    apply_pwm_ticks "$center_ticks"
    sleep 0.4

    # Coupure complète du signal PWM sur le canal 0 (Full OFF)
    i2cset -y $I2C_BUS $PCA_ADDR $CH0_OFF_H 0x10 >/dev/null 2>&1
    
    echo "[✓] Servomoteur relâché et PWM désactivé. Roues au repos."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Détermination de l'angle initial
INIT_VAL="${1:-0}"
if [ "$INIT_VAL" -eq 90 ] 2>/dev/null; then
    # 90° dans la convention servo standard = 0° braquage
    CURRENT_ANGLE=0.0
elif [ "$INIT_VAL" -ge -45 ] 2>/dev/null && [ "$INIT_VAL" -le 45 ] 2>/dev/null; then
    CURRENT_ANGLE=$(awk -v v="$INIT_VAL" 'BEGIN { printf "%.1f", v * 1.0 }')
else
    CURRENT_ANGLE=0.0
fi

init_pca9685

echo "============================================================"
echo "    CALIBRATION EN DIRECT DU SERVOMOTEUR (PCA9685 Canal 0)   "
echo "============================================================"
echo " Commandes clavier :"
echo "   [+] ou [d] : Décaler de +1.0° vers la droite"
echo "   [-] ou [g] : Décaler de -1.0° vers la gauche"
echo "   [D]        : Ajustement fin +0.2° vers la droite"
echo "   [G]        : Ajustement fin -0.2° vers la gauche"
echo "   [0]        : Revenir à 0.0°"
echo "   [q] ou ^C  : Quitter et couper le moteur"
echo "============================================================"
echo ""

while true; do
    TICKS=$(calc_ticks "$CURRENT_ANGLE")
    PULSE=$(calc_pulse_ms "$CURRENT_ANGLE")
    apply_pwm_ticks "$TICKS"

    printf "\r-> Angle actuel : %-6s° | Impulsion : %s ms | Ticks : %-4d " \
           "$CURRENT_ANGLE" "$PULSE" "$TICKS"

    read -rsn1 KEY
    case "$KEY" in
        "+"|"d")
            CURRENT_ANGLE=$(awk -v a="$CURRENT_ANGLE" 'BEGIN { printf "%.1f", a + 1.0 }')
            ;;
        "-"|"g")
            CURRENT_ANGLE=$(awk -v a="$CURRENT_ANGLE" 'BEGIN { printf "%.1f", a - 1.0 }')
            ;;
        "D")
            CURRENT_ANGLE=$(awk -v a="$CURRENT_ANGLE" 'BEGIN { printf "%.1f", a + 0.2 }')
            ;;
        "G")
            CURRENT_ANGLE=$(awk -v a="$CURRENT_ANGLE" 'BEGIN { printf "%.1f", a - 0.2 }')
            ;;
        "0")
            CURRENT_ANGLE="0.0"
            ;;
        "q"|"Q")
            cleanup
            ;;
    esac
done
