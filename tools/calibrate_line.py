#!/usr/bin/env python3
# ==============================================================================
# Outil interactif de calibration du suiveur de ligne analogique PiCar-S (5 voies)
# ==============================================================================
# Cet outil calibre individuellement les 5 phototransistors TCRT5000 sur :
#  1. La surface blanche (sol / piste)
#  2. Le ruban isolant noir (18 mm)
#
# Il génère le fichier de calibration système : /etc/picar_line_calibration
# (ou $HOME/.picar_line_calibration) lu automatiquement par le service C++ Sensors.
# ==============================================================================

import os
import sys
import time

try:
    import smbus
except ImportError:
    try:
        import smbus2 as smbus
    except ImportError:
        smbus = None

I2C_BUS = 1
ADDR_PRIMARY = 0x11
ADDR_ALT = 0x0B
REG_START = 0x00

def get_i2c_bus():
    if smbus is None:
        print("[ERREUR] Module python smbus/smbus2 introuvable. Installez : pip install smbus2 ou apt install python3-smbus")
        sys.exit(1)
    try:
        return smbus.SMBus(I2C_BUS)
    except Exception as e:
        print(f"[ERREUR] Impossible d'ouvrir le bus I2C {I2C_BUS} : {e}")
        sys.exit(1)

def detect_device(bus):
    for addr in (ADDR_PRIMARY, ADDR_ALT):
        try:
            bus.read_byte(addr)
            return addr
        except Exception:
            pass
    return None

def read_raw_adc(bus, addr):
    # Lecture des 10 octets bruts (5 canaux x 2 octets, 10 bits MSB/LSB)
    data = bus.read_i2c_block_data(addr, REG_START, 10)
    adc = []
    for i in range(5):
        val = (data[i * 2] << 8) | data[i * 2 + 1]
        adc.append(val)
    return adc

def sample_readings(bus, addr, count=30, delay=0.03):
    samples = [[] for _ in range(5)]
    for _ in range(count):
        raw = read_raw_adc(bus, addr)
        for i in range(5):
            samples[i].append(raw[i])
        time.sleep(delay)
    return [sum(s) / len(s) for s in samples]

def main():
    print("=" * 68)
    print("  CALIBRATION INTERACTIVE DU SUIVEUR DE LIGNE PICAR-S (5 VOIES)")
    print("=" * 68)
    print("Cet assistant va mesurer la réponse optique réelle de vos 5 capteurs")
    print("sur votre surface pour garantir un suivi continu ultra-fluide.\n")

    bus = get_i2c_bus()
    addr = detect_device(bus)
    if addr is None:
        print(f"[ERREUR] Suiveur de ligne non détecté sur 0x{ADDR_PRIMARY:02X} ou 0x{ADDR_ALT:02X}.")
        print("Vérifiez le câblage I2C du Robot HATS.")
        sys.exit(1)

    print(f"✓ Module suiveur de ligne détecté à l'adresse 0x{addr:02X}.\n")

    # Étape 1 : Surface blanche
    print("--- ÉTAPE 1 / 2 : RÉFÉRENCE SOL BLANC ---")
    print("Placez TOUS les 5 capteurs au-dessus du sol blanc (sans ruban noir).")
    input("Appuyez sur [ENTRÉE] lorsque le robot est en position...")
    print("Mesure du blanc en cours...", end="", flush=True)
    white_adc = sample_readings(bus, addr, count=40)
    print(" Terminé.")
    for i, w in enumerate(white_adc):
        print(f"  Capteur {i} (Blanc) : {w:.1f}")
    print()

    # Étape 2 : Ruban noir
    print("--- ÉTAPE 2 / 2 : RÉFÉRENCE RUBAN NOIR ---")
    print("Faites glisser lentement le ruban noir sous chaque capteur (ou placez")
    print("successivement le noir sous chaque capteur).")
    input("Appuyez sur [ENTRÉE] pour démarrer 5 secondes d'échantillonnage dynamique...")
    print("Balayez la ligne sous les capteurs (5 secondes)...")

    min_black = list(white_adc)
    start_t = time.time()
    while time.time() - start_t < 5.0:
        raw = read_raw_adc(bus, addr)
        for i in range(5):
            if raw[i] < min_black[i]:
                min_black[i] = raw[i]
        time.sleep(0.02)

    # Sécurité si un capteur n'a pas vu de noir
    black_adc = []
    for i in range(5):
        if min_black[i] >= white_adc[i] - 15.0:
            fallback_b = max(10.0, white_adc[i] * 0.25)
            print(f"  [ATTENTION] Capteur {i} n'a pas vu assez de contraste. Défaut: {fallback_b:.1f}")
            black_adc.append(fallback_b)
        else:
            black_adc.append(float(min_black[i]))
            print(f"  Capteur {i} (Noir min détecté) : {black_adc[i]:.1f}")
    print()

    # Récapitulatif
    print("--- RÉSUMÉ DE LA CALIBRATION ---")
    print("Capteur | Blanc ADC | Noir ADC | Delta | Seuil 50%")
    print("--------------------------------------------------")
    for i in range(5):
        delta = white_adc[i] - black_adc[i]
        thresh = (white_adc[i] + black_adc[i]) / 2.0
        print(f"   {i}    |   {white_adc[i]:6.1f}  |  {black_adc[i]:6.1f}  | {delta:5.1f} |  {thresh:6.1f}")
    print()

    # Sauvegarde
    target_path = "/etc/picar_line_calibration"
    save_path = target_path
    try:
        with open(target_path, "w") as f:
            pass
    except PermissionError:
        save_path = os.path.expanduser("~/.picar_line_calibration")
        print(f"[NOTE] Pas d'accès root en écriture sur {target_path}.")
        print(f"       Sauvegarde dans le dossier utilisateur : {save_path}")

    rep = input(f"Enregistrer cette calibration dans {save_path} ? [O/n] : ").strip().lower()
    if rep in ("", "o", "oui", "y", "yes"):
        with open(save_path, "w") as f:
            f.write("# Calibration suiveur de ligne PiCar-S (généré par tools/calibrate_line.py)\n")
            f.write("# Format: <capteur_idx> <noir_adc> <blanc_adc>\n")
            for i in range(5):
                f.write(f"{i} {black_adc[i]:.1f} {white_adc[i]:.1f}\n")
        print(f"✓ Calibration enregistrée avec succès dans {save_path} !")
    else:
        print("Annulé, aucun fichier enregistré.")

    # Mode test en direct
    rep = input("\nLancer un test interactif en direct de la noirceur normalisée ? [O/n] : ").strip().lower()
    if rep in ("", "o", "oui", "y", "yes"):
        print("\nAffichage en direct (Ctrl+C pour quitter) :")
        weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
        try:
            while True:
                raw = read_raw_adc(bus, addr)
                darkness = []
                for i in range(5):
                    span = white_adc[i] - black_adc[i]
                    d = 0.0 if span <= 1.0 else (white_adc[i] - raw[i]) / span
                    d = max(0.0, min(1.0, d))
                    darkness.append(d)

                tot = sum(darkness)
                w_sum = sum(w * d for w, d in zip(weights, darkness))
                err = (w_sum / tot) if tot >= 0.20 else 0.0
                steer_deg = err * (45.0 / 2.0)

                bars = "".join(f"[{'#' * int(d * 10):<10}] " for d in darkness)
                status = "T-STOP" if tot >= 3.8 else ("LIGNE" if tot >= 0.20 else "PERDUE")
                sys.stdout.write(f"\r{bars} | Err: {err:+5.2f} ({steer_deg:+5.1f}°) | {status}   ")
                sys.stdout.flush()
                time.sleep(0.04)
        except KeyboardInterrupt:
            print("\nTest terminé.")

if __name__ == "__main__":
    main()
