#!/usr/bin/env python3
"""Calibre le modèle dynamique à partir de mesures du vrai PiCar-S et écrit calibration.json.

Mesures à faire sur le robot (roues au sol, piste dégagée, vitesse relevée à l'ultrason ou à l'odométrie) :

  1. ECHELON : puissance 0 -> 50 % d'un coup, enregistrer la vitesse toutes les 20 ms pendant 2 s.
     CSV `step.csv`, colonnes : time_s,speed_mps   (time_s = 0 à l'envoi de l'ordre)
     -> retard pur (latence) et constante de temps du moteur.
  2. PALIERS : puissance fixe 20, 25, 30, 40, 50, 60, 75 %, vitesse stabilisée relevée pour chacune.
     CSV `plateaus.csv`, colonnes : power_pct,speed_mps
     -> zone morte (puissance minimale qui fait avancer) et gain vitesse/puissance.
  3. DIRECTION : braquage -45° -> +45° d'un coup, angle relevé (vidéo ou capteur) toutes les 20 ms.
     CSV `steer.csv`, colonnes : time_s,angle_deg
     -> vitesse maximale du servo (°/s).

Usage : calibrate_dynamics.py --step step.csv --plateaus plateaus.csv --steer steer.csv
        (chaque fichier est facultatif ; les paramètres non mesurés gardent leur valeur actuelle)
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

CONTROLLER_DIR = Path(__file__).resolve().parent.parent / "controllers" / "picar_s_controller"
sys.path.insert(0, str(CONTROLLER_DIR))
from picar_dynamics import Calibration  # noqa: E402

REFERENCE_POWER_PCT = 50.0       # puissance a laquelle le FSM suppose 0,32 m/s
SIM_SPEED_AT_REFERENCE = 0.32


def read_csv(path, *cols):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return [[float(r[c]) for c in cols] for r in rows]


def fit_step(rows):
    """Retard = instant où la vitesse dépasse 5 % de sa valeur finale ; tau = temps de 5 % à 63 % de la valeur finale."""
    t = [r[0] for r in rows]
    v = [r[1] for r in rows]
    final = sum(v[-max(3, len(v) // 10):]) / max(3, len(v) // 10)
    if final <= 0:
        raise ValueError("vitesse finale nulle : echelon inexploitable")
    t5 = next(ti for ti, vi in zip(t, v) if vi >= 0.05 * final)
    t63 = next(ti for ti, vi in zip(t, v) if vi >= 0.632 * final)
    return max(0.0, t5), max(0.01, t63 - t5), final


def fit_plateaus(rows):
    """Zone morte = plus petite vitesse mesurée qui démarre vraiment ; gain = vitesse réelle / vitesse supposée par le FSM."""
    moving = [(p, v) for p, v in rows if v > 0.02]
    if not moving:
        raise ValueError("aucun palier ne fait avancer le robot")
    p_min, v_min = min(moving)
    # le FSM convertit une puissance P en consigne v_fsm = P/50*0.32 ; le gain est mesuré a la reference
    ref = min(rows, key=lambda r: abs(r[0] - REFERENCE_POWER_PCT))
    gain = ref[1] / (ref[0] / REFERENCE_POWER_PCT * SIM_SPEED_AT_REFERENCE)
    deadband_cmd = p_min / REFERENCE_POWER_PCT * SIM_SPEED_AT_REFERENCE   # en m/s de consigne FSM
    return deadband_cmd, gain


def fit_steer(rows):
    """Vitesse du servo = pente maximale de l'angle (lissée sur 3 points)."""
    best = 0.0
    for i in range(3, len(rows)):
        dt = rows[i][0] - rows[i - 3][0]
        if dt > 0:
            best = max(best, abs(rows[i][1] - rows[i - 3][1]) / dt)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step"), ap.add_argument("--plateaus"), ap.add_argument("--steer")
    ap.add_argument("--out", default=str(CONTROLLER_DIR / "calibration.json"))
    a = ap.parse_args()
    cal = Calibration.load(a.out)
    if a.step:
        cal.latency_s, cal.motor_tau_s, _ = fit_step(read_csv(a.step, "time_s", "speed_mps"))
    if a.plateaus:
        cal.motor_deadband_mps, cal.motor_gain = fit_plateaus(read_csv(a.plateaus, "power_pct", "speed_mps"))
    if a.steer:
        cal.steer_slew_dps = fit_steer(read_csv(a.steer, "time_s", "angle_deg"))
    cal.measured = bool(a.step and a.plateaus and a.steer)
    cal.save(a.out)
    print(f"calibration ecrite dans {a.out} (measured={cal.measured})")
    print(f"  latence {cal.latency_s:.3f} s | tau moteur {cal.motor_tau_s:.3f} s | zone morte {cal.motor_deadband_mps:.3f} m/s "
          f"| gain {cal.motor_gain:.2f} | servo {cal.steer_slew_dps:.0f} deg/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
