#!/usr/bin/env python3
"""
Outil CLI d'analyse statistique et de diagnostic pour fichiers CSV de télémétrie PiCar-S.
Permet d'analyser rapidement un log de roulage sans interface graphique.
"""

import sys
import os
import csv
import math

FSM_STATES = {
    0: "Init",
    1: "Unarmed",
    2: "Armed",
    3: "LineFollowingForward",
    4: "CollisionRisk",
    5: "Stopped",
    6: "CollisionOccured",
    7: "RightDeviation",
    8: "LeftDeviation",
    9: "Reverse",
    10: "LineSearching",
    11: "LineDetected",
    12: "LineSearchingTimedOut",
    13: "Abort",
    14: "Failure",
    255: "Undefined"
}

MOTOR_STATES = {
    0: "Init",
    1: "Idle",
    2: "Running",
    3: "Abort"
}

def analyze_csv(filepath):
    if not os.path.isfile(filepath):
        print(f"Erreur: Fichier introuvable '{filepath}'")
        sys.exit(1)

    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("Erreur: Fichier CSV vide.")
        sys.exit(1)

    total_records = len(rows)
    first_ts = int(rows[0]["timestamp_ms"])
    last_ts = int(rows[-1]["timestamp_ms"])
    duration_s = (last_ts - first_ts) / 1000.0
    avg_hz = total_records / duration_s if duration_s > 0 else 0.0

    print("=" * 65)
    print(f"  RAPPORT D'ANALYSE TÉLÉMÉTRIE FASTDRINK / PICAR-S")
    print("=" * 65)
    print(f"Fichier          : {os.path.basename(filepath)}")
    print(f"Échantillons     : {total_records} trames")
    print(f"Durée totale     : {duration_s:.2f} s ({duration_s/60.0:.2f} min)")
    print(f"Fréquence moy.   : {avg_hz:.1f} Hz")
    print("-" * 65)

    # 1. Analyse FSM
    fsm_counts = {}
    fsm_transitions = []
    prev_fsm = None
    for r in rows:
        st = int(r.get("fsm_current_state", -1))
        fsm_counts[st] = fsm_counts.get(st, 0) + 1
        if prev_fsm is not None and st != prev_fsm:
            fsm_transitions.append((r["timestamp_ms"], prev_fsm, st))
        prev_fsm = st

    print("ÉTATS FSM (Machine à États) :")
    for st, count in sorted(fsm_counts.items()):
        name = FSM_STATES.get(st, f"Unknown({st})")
        pct = (count / total_records) * 100.0
        print(f"  • {name:<22} : {count:4d} trames ({pct:5.1f}%)")
    print(f"  Total transitions : {len(fsm_transitions)}")

    # 2. Moteurs & Déplacements
    left_encs = [int(r.get("m_left_encoder", 0)) for r in rows]
    right_encs = [int(r.get("m_right_encoder", 0)) for r in rows]
    delta_left = left_encs[-1] - left_encs[0]
    delta_right = right_encs[-1] - right_encs[0]
    avg_ticks = (delta_left + delta_right) / 2.0

    left_throttles = [float(r.get("m_left_applied_throttle", 0.0)) for r in rows]
    right_throttles = [float(r.get("m_right_applied_throttle", 0.0)) for r in rows]
    max_throttle = max(max(left_throttles), max(right_throttles))
    avg_throttle = sum(left_throttles + right_throttles) / (2.0 * total_records)

    print("-" * 65)
    print("MOTEURS & DÉPLACEMENT :")
    print(f"  • Encodeur gauche (ticks) : {left_encs[0]} -> {left_encs[-1]} (Δ = {delta_left})")
    print(f"  • Encodeur droit (ticks)  : {right_encs[0]} -> {right_encs[-1]} (Δ = {delta_right})")
    print(f"  • Throttle max appliqué   : {max_throttle:.2f}")
    print(f"  • Throttle moyen          : {avg_throttle:.2f}")

    # 3. Direction & Servos
    steerings = [float(r.get("steering_current_angle", 0.0)) for r in rows]
    min_steer = min(steerings)
    max_steer = max(steerings)
    avg_steer = sum(steerings) / total_records
    print("-" * 65)
    print("DIRECTION (Servomoteur) :")
    print(f"  • Angle braquage min / max: {min_steer:.1f}° / {max_steer:.1f}°")
    print(f"  • Angle braquage moyen    : {avg_steer:.1f}°")

    # 4. Capteurs : Sonar Ultrason
    sonars = [float(r.get("sonar_calc_dist_cm", 0.0)) for r in rows]
    min_sonar = min(sonars)
    max_sonar = max(sonars)
    alerts_sonar = sum(1 for s in sonars if s < 25.0)
    print("-" * 65)
    print("TÉLÉMÉTRIE ULTRASON (Sonar) :")
    print(f"  • Distance min / max      : {min_sonar:.1f} cm / {max_sonar:.1f} cm")
    print(f"  • Trames en zone alerte   : {alerts_sonar} trames (< 25 cm)")

    # 5. Capteurs : Suivi de ligne TCRT5000 (S0..S4)
    print("-" * 65)
    print("SUIVEUR DE LIGNE (5 capteurs normalisés 0.0 - 1.0) :")
    for i in range(5):
        vals = [float(r.get(f"s{i}_norm", 0.0)) for r in rows]
        print(f"  • S{i} : moy={sum(vals)/total_records:.3f}, min={min(vals):.3f}, max={max(vals):.3f}")

    # 6. Diagnostic & Erreurs
    fsm_errs = [int(r.get("fsm_error_code", 0)) for r in rows if int(r.get("fsm_error_code", 0)) != 0]
    motor_errs = [int(r.get("motors_error_code", 0)) for r in rows if int(r.get("motors_error_code", 0)) != 0]
    sensors_errs = [int(r.get("sensors_comp_err", 0)) for r in rows if int(r.get("sensors_comp_err", 0)) != 0]

    print("-" * 65)
    print("DIAGNOSTIC SYSTÈME & ERREURS :")
    print(f"  • FSM erreurs actives     : {len(fsm_errs)} occurrences")
    print(f"  • Motors erreurs actives  : {len(motor_errs)} occurrences")
    print(f"  • Sensors erreurs actives : {len(sensors_errs)} occurrences")
    print("=" * 65)

if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "sample_telemetry.csv"
    analyze_csv(target)
