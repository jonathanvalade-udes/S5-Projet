#!/usr/bin/env python3
"""
tools/optimizer/optimize_params.py
Optimiseur cinématique bayésien basé sur Optuna pour le robot FastDrink / PiCar.
Recherche les hyperparamètres optimaux (Kp, Kd, Vitesse de croisière, Ralentissement en virage)
afin de minimiser le temps de parcours tout en garantissant :
1. Accélération totale a_tot <= 0.735 m/s² (Zéro déversement de la bille).
2. Erreur latérale <= 25 mm (Zéro déviation hors du ruban noir de 18 mm).
3. Rayon minimal R >= 120 mm négocié sans décrochage.
"""

import math
import sys
import os
from typing import Tuple, List
import optuna

# Contraintes physiques absolues (PRD.md & CalculAcceleration.pdf)
MAX_ACCELERATION = 0.735  # m/s² (seuil de renversement de la bille)
TRACK_TAPE_WIDTH = 0.018  # 18 mm ruban noir
WHEELBASE = 0.14          # 140 mm empattement PiCar-S
DT = 0.02                 # Pas de simulation à 50 Hz (20 ms)

class TrackSimulator:
    """
    Génère un circuit représentatif de 2m x 2m avec lignes droites, 
    virages serrés (R = 0.12 m) et virages larges (R = 0.25 m).
    """
    def __init__(self):
        # Découpage du parcours en segments : (type, longueur_ou_angle, rayon)
        # 'straight' : longueur en mètres
        # 'curve' : angle en radians, rayon en mètres (+ = virage droite, - = virage gauche)
        self.segments = [
            ('straight', 0.8),
            ('curve', math.pi / 2, 0.12),   # Virage serré 90° R=120mm
            ('straight', 0.4),
            ('curve', math.pi / 2, 0.25),   # Virage moyen 90°
            ('straight', 0.6),
            ('curve', math.pi / 2, 0.12),   # Virage serré 90° R=120mm
            ('straight', 0.4),
            ('curve', math.pi / 2, 0.20),   # Virage de retour 90°
        ]

    def run_simulation(self, kp: float, kd: float, cruise_speed: float, 
                       min_curve_speed: float, slowdown_gain: float) -> Tuple[float, float, bool]:
        """
        Simule le parcours complet et retourne :
        (temps_total, max_accel_observée, bille_tombée_ou_décrochage)
        """
        total_time = 0.0
        current_speed = 0.0
        max_total_accel = 0.0
        ball_dropped = False
        off_track = False

        lateral_error = 0.0
        last_error = 0.0
        heading_error = 0.0

        for seg_type, param1, *optional in self.segments:
            if seg_type == 'straight':
                seg_len = param1
                progress = 0.0
                while progress < seg_len:
                    # Sur ligne droite : courbure = 0
                    curvature = 0.0
                    
                    # Dérivée de l'erreur
                    d_error = (lateral_error - last_error) / DT
                    last_error = lateral_error

                    # Commande de braquage PD
                    steer = kp * lateral_error + kd * d_error
                    steer = max(-1.0, min(1.0, steer))

                    # Régulation de vitesse
                    target_speed = cruise_speed
                    max_delta_v = MAX_ACCELERATION * DT
                    delta_v = target_speed - current_speed
                    if delta_v > max_delta_v:
                        current_speed += max_delta_v
                    elif delta_v < -max_delta_v:
                        current_speed -= max_delta_v
                    else:
                        current_speed = target_speed

                    # Accélération longitudinale & latérale
                    a_long = abs(delta_v) / DT
                    a_lat = (current_speed ** 2) * abs(steer * math.tan(math.radians(45.0)) / WHEELBASE)
                    a_total = math.sqrt(a_long**2 + a_lat**2)

                    if a_total > max_total_accel:
                        max_total_accel = a_total

                    if a_total > MAX_ACCELERATION:
                        ball_dropped = True

                    # Avancement
                    progress += current_speed * DT
                    total_time += DT

                    # Réduction résiduelle de l'erreur latérale
                    lateral_error += (steer * 0.05) - (lateral_error * 0.2)
                    if abs(lateral_error) > 0.025: # Plus de 25 mm
                        off_track = True

                    if ball_dropped or off_track:
                        return total_time, max_total_accel, True

            elif seg_type == 'curve':
                angle = param1
                radius = optional[0]
                curve_len = angle * radius
                progress = 0.0

                while progress < curve_len:
                    # Courbure théorique
                    curvature = 1.0 / radius
                    
                    # Consigne de braquage pour compenser la courbure
                    required_steer = math.atan(WHEELBASE * curvature) / math.radians(45.0)
                    
                    # Vitesse modulée par la sévérité du virage
                    speed_penalty = slowdown_gain * (abs(curvature) ** 1.3)
                    target_speed = max(min_curve_speed, cruise_speed - speed_penalty)

                    # Rampe d'accélération/décélération
                    max_delta_v = MAX_ACCELERATION * DT
                    delta_v = target_speed - current_speed
                    if delta_v > max_delta_v:
                        current_speed += max_delta_v
                    elif delta_v < -max_delta_v:
                        current_speed -= max_delta_v
                    else:
                        current_speed = target_speed

                    # Calcul de l'accélération centrifuge : a_c = v² / R
                    a_long = abs(delta_v) / DT
                    a_lat = (current_speed ** 2) / radius
                    a_total = math.sqrt(a_long**2 + a_lat**2)

                    if a_total > max_total_accel:
                        max_total_accel = a_total

                    if a_total > MAX_ACCELERATION:
                        ball_dropped = True

                    progress += current_speed * DT
                    total_time += DT

                    if ball_dropped:
                        return total_time, max_total_accel, True

        return total_time, max_total_accel, (ball_dropped or off_track)


def objective(trial: optuna.Trial) -> float:
    # Espace de recherche des hyperparamètres
    kp = trial.suggest_float("kp", 0.35, 1.20)
    kd = trial.suggest_float("kd", 0.05, 0.40)
    cruise_speed = trial.suggest_float("cruise_speed", 0.18, 0.35)
    min_curve_speed = trial.suggest_float("min_curve_speed", 0.08, 0.20)
    slowdown_gain = trial.suggest_float("slowdown_gain", 0.02, 0.15)

    sim = TrackSimulator()
    lap_time, max_accel, failed = sim.run_simulation(
        kp, kd, cruise_speed, min_curve_speed, slowdown_gain
    )

    if failed:
        # Forte pénalité si la bille tombe ou si le robot décroche
        return 999.0 + (max_accel * 10.0)

    # Fonction objectif : minimiser le temps de tour tout en pénalisant les approches trop proches de 0.735 m/s²
    safety_margin = max(0.0, max_accel - (MAX_ACCELERATION * 0.95))
    score = lap_time + (safety_margin * 50.0)
    return score


def run_optimization(n_trials: int = 500):
    print("=" * 65)
    print("   FastDrink S5-GI : Optimiseur Cinématique Bayésien (Optuna)")
    print(f"   Objectif : Minimiser le temps de tour sous a <= {MAX_ACCELERATION} m/s²")
    print("=" * 65)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=n_trials)

    print("\n" + "=" * 65)
    print("  🏆 RÉSULTATS OPTIMAUX TROUVÉS PAR OPTUNA")
    print("=" * 65)
    best = study.best_params
    print(f"  • Score (Temps de tour estimé) : {study.best_value:.2f} secondes")
    print(f"  • Kp (Gain proportionnel)      : {best['kp']:.4f}")
    print(f"  • Kd (Gain dérivé)             : {best['kd']:.4f}")
    print(f"  • Vitesse de croisière         : {best['cruise_speed']:.4f} m/s")
    print(f"  • Vitesse minimale en virage   : {best['min_curve_speed']:.4f} m/s")
    print(f"  • Facteur de ralentissement    : {best['slowdown_gain']:.4f}")
    print("=" * 65)

    # Export automatique vers un header C++ pour picar_brain
    header_path = "firmware/include/control/optimized_params.hpp"
    os.makedirs(os.path.dirname(header_path), exist_ok=True)
    with open(header_path, "w") as f:
        f.write("// Généré automatiquement par tools/optimizer/optimize_params.py via Optuna\n")
        f.write("#pragma once\n\n")
        f.write("namespace Config {\n")
        f.write(f"    constexpr float OPTIMIZED_KP = {best['kp']:.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_KD = {best['kd']:.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_CRUISE_SPEED = {best['cruise_speed']:.4f}f; // m/s\n")
        f.write(f"    constexpr float OPTIMIZED_MIN_CURVE_SPEED = {best['min_curve_speed']:.4f}f; // m/s\n")
        f.write(f"    constexpr float OPTIMIZED_SLOWDOWN_GAIN = {best['slowdown_gain']:.4f}f;\n")
        f.write("}\n")
    print(f"\n✓ Paramètres exportés avec succès dans : {header_path}")

if __name__ == "__main__":
    trials = 300
    if len(sys.argv) > 1:
        trials = int(sys.argv[1])
    run_optimization(trials)
