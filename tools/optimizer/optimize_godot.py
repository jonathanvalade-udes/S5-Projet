#!/usr/bin/env python3
"""
tools/optimizer/optimize_godot.py
=============================================================================
Optimiseur bayésien (Optuna) pour le robot FastDrink / PiCar-S.
Supporte 2 modes :
  1. --mode fast  : Moteur analytique Python pur ultra-rapide (500 essais en ~5 s).
  2. --mode godot : Co-simulation en boucle fermée via Godot 4 Headless 3D
                    (utilise le maillage Blender, raycasts réels et RigidBody).
=============================================================================
"""

import sys
import os
import math
import subprocess
import json
import argparse
import time
from typing import Tuple, Dict, Any

try:
    import optuna
except ImportError:
    print("[ERREUR] Optuna n'est pas installé dans cet environnement Python.")
    print("Veuillez utiliser : ./tools/optimizer/.venv/bin/python3 tools/optimizer/optimize_godot.py")
    sys.exit(1)

# Constantes physiques FastDrink
MAX_ACCELERATION = 0.735   # m/s² (seuil de déversement critique)
WHEELBASE = 0.14           # 140 mm
CUP_RADIUS = 0.020         # 20 mm rayon coupelle
SLOPE_TAN = 0.075          # h/R pente conique
G = 9.81
DAMPING = 6.0
DT = 0.02                  # 50 Hz

CONF_PATH = "picar.conf"
HEADER_PATH = "services/libs/shared-headers/control/optimized_params.hpp"

# ---------------------------------------------------------------------------
# MODE FAST : Moteur analytique Python vectorisé
# ---------------------------------------------------------------------------
class FastSimulator:
    """Simulateur analytique 2D avec cinématique d'Ackermann et dynamique de la bille."""
    def __init__(self):
        # Circuit type : (type, param, rayon)
        self.segments = [
            ('straight', 0.8),
            ('curve', math.pi / 2, 0.12),   # Virage serré 90° R=120 mm
            ('straight', 0.4),
            ('curve', math.pi / 2, 0.25),   # Virage moyen 90°
            ('straight', 0.6),
            ('curve', math.pi / 2, 0.12),   # Virage serré 90° R=120 mm
            ('straight', 0.4),
            ('curve', math.pi / 2, 0.20),   # Virage retour 90°
            ('straight', 0.6),
        ]

    def simulate(self, p: Dict[str, float]) -> Tuple[float, float, bool, str]:
        total_time = 0.0
        current_spd = 0.0
        max_a_tot = 0.0
        ball_pos_x = 0.0
        ball_pos_z = 0.0
        ball_vel_x = 0.0
        ball_vel_z = 0.0
        integral = 0.0
        last_error = 0.0
        filtered_error = 0.0

        for seg_type, length_or_angle, *opt in self.segments:
            if seg_type == 'straight':
                distance = length_or_angle
                radius = 999.0
            else:
                radius = opt[0]
                distance = length_or_angle * radius

            progress = 0.0
            while progress < distance:
                # 1. Erreur spatiale sur le segment
                raw_error = 0.0 if seg_type == 'straight' else (0.14 / radius) * 0.8
                # Filtre EMA
                filtered_error += p['alpha'] * (raw_error - filtered_error)
                err = filtered_error

                # Soft deadband
                if abs(err) < p['knee']:
                    err *= abs(err) / p['knee']

                # Dérivée
                d_err = (err - last_error) / DT
                d_err = max(-8.0, min(8.0, d_err))
                last_error = err

                # Intégrale anti-windup
                if abs(err) < 0.6:
                    integral += p['ki'] * err * DT
                else:
                    integral *= (1.0 - 0.5 * DT)
                integral = max(-0.15, min(0.15, integral))

                # Braquage PD + I
                steer = p['kp'] * err + p['kd'] * d_err + integral
                steer = max(-1.0, min(1.0, steer))
                steer_deg = steer * 45.0

                # Vitesse adaptative Ackermann
                abs_steer_rad = abs(steer_deg) * (math.pi / 180.0)
                if abs_steer_rad < 0.052:
                    target_spd = p['cruise']
                else:
                    tan_s = max(1e-4, math.tan(abs_steer_rad))
                    R = WHEELBASE / tan_s
                    v_curve = p['safety'] * math.sqrt(MAX_ACCELERATION * R)
                    target_spd = max(p['min_curve'], min(p['cruise'], v_curve))

                # Feedforward prédictif
                if p['brake_gain'] > 0.0:
                    reduction = min(p['brake_gain'] * abs(d_err), 0.35 * target_spd)
                    target_spd = max(p['min_curve'], target_spd - reduction)

                # Rampe d'accélération MotionProfiler
                max_dv = MAX_ACCELERATION * DT
                dv = target_spd - current_spd
                if dv > max_dv: current_spd += max_dv
                elif dv < -max_dv: current_spd -= max_dv
                else: current_spd = target_spd

                # Accélération longitudinale signée (+ = accélération, - = freinage)
                a_long = (dv) / DT
                tan_ack = math.tan(abs_steer_rad)
                R_turn = (WHEELBASE / tan_ack) if tan_ack > 1e-4 else 999.0
                a_lat = (current_spd ** 2) / R_turn
                a_tot = math.sqrt(a_long**2 + a_lat**2)
                if a_tot > max_a_tot:
                    max_a_tot = a_tot

                # Dynamique 2D de roulement de la bille
                factor = 5.0 / 7.0
                r_dist = math.hypot(ball_pos_x, ball_pos_z)
                f_grav_x = (-G * SLOPE_TAN * (ball_pos_x / r_dist)) if r_dist > 1e-4 else 0.0
                f_grav_z = (-G * SLOPE_TAN * (ball_pos_z / r_dist)) if r_dist > 1e-4 else 0.0

                acc_ball_x = factor * (a_long + f_grav_x - DAMPING * ball_vel_x)
                acc_ball_z = factor * (a_lat + f_grav_z - DAMPING * ball_vel_z)

                ball_vel_x += acc_ball_x * DT
                ball_vel_z += acc_ball_z * DT
                ball_pos_x += ball_vel_x * DT
                ball_pos_z += ball_vel_z * DT

                if math.hypot(ball_pos_x, ball_pos_z) > CUP_RADIUS:
                    return total_time, max_a_tot, True, "ball_dropped"

                progress += current_spd * DT
                total_time += DT

        return total_time, max_a_tot, False, "success"


# ---------------------------------------------------------------------------
# MODE GODOT : Exécution en boucle fermée headless
# ---------------------------------------------------------------------------
def run_godot_trial(p: Dict[str, float], time_scale: float = 5.0, timeout: float = 30.0) -> Tuple[float, float, bool, str]:
    """Exécute un tour complet dans Godot headless et extrait le résultat."""
    # 1. Mettre à jour fsm.conf et picar.conf
    write_picar_conf(p)

    # 2. Transmettre les paramètres dynamiques à SimBridge si connecté
    try:
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect(("127.0.0.1", 9000))
            s.sendall(f"STRAIGHT_CRUISE_SPEED={p['cruise']:.4f}\n".encode())
            s.sendall(f"MIN_CURVE_SPEED={p['min_curve']:.4f}\n".encode())
            s.sendall(f"CURVE_SAFETY_FACTOR={p['safety']:.4f}\n".encode())
            s.sendall(f"LINE_KP={p['kp']:.4f}\n".encode())
            s.sendall(f"LINE_KD={p['kd']:.4f}\n".encode())
    except Exception:
        pass

    cmd = [
        "godot",
        "--path", "godot",
        "--headless",
        "--",
        "--optuna",
        "--time-scale", str(time_scale),
        "--timeout", str(timeout)
    ]

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout, _ = proc.communicate(timeout=timeout / time_scale + 5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        return 999.0, 0.0, True, "timeout"
    except Exception as e:
        return 999.0, 0.0, True, f"godot_error_{str(e)}"

    for line in stdout.splitlines():
        if "OPTUNA_RESULT:" in line:
            json_str = line.split("OPTUNA_RESULT:", 1)[1].strip()
            try:
                res = json.loads(json_str)
                return (
                    float(res.get("lap_time", 999.0)),
                    float(res.get("max_accel", 0.0)),
                    bool(res.get("ball_dropped", True)),
                    str(res.get("status", "unknown"))
                )
            except:
                pass

    return 999.0, 0.0, True, "no_result_parsed"


def write_picar_conf(p: Dict[str, float]):
    """Écrit les paramètres dans fsm.conf et picar.conf."""
    content = f"""# ==========================================
# CONFIGURATION FSM - PICAR-S (OPTUNA)
# ==========================================
HZ=200.0
MAX_ACCELERATION=0.735
STRAIGHT_CRUISE_SPEED={p['cruise']:.4f}
MIN_CURVE_SPEED={p['min_curve']:.4f}
CURVE_SAFETY_FACTOR={p['safety']:.4f}
PREDICTIVE_BRAKE_GAIN=0.0000

LINE_KP={p['kp']:.4f}
LINE_KD={p['kd']:.4f}

OBSTACLE_STOP_DIST_CM=20.0
REVERSE_SPEED=0.12
AVOID_SIDE=1.0
AUTO_START_DELAY=0.0
"""
    for path in ["fsm.conf", "picar.conf"]:
        try:
            with open(path, "w") as f:
                f.write(content)
        except Exception:
            pass


def export_header(best_p: Dict[str, float], lap_time: float):
    """Exporte les constantes optimales dans optimized_params.hpp et picar_constants.hpp."""
    os.makedirs(os.path.dirname(HEADER_PATH), exist_ok=True)
    with open(HEADER_PATH, "w") as f:
        f.write(f"// Optimisé automatiquement par Optuna le {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"// Temps de tour optimal estimé : {lap_time:.2f} s\n")
        f.write("#pragma once\n\n")
        f.write("namespace Config {\n")
        f.write(f"    constexpr float OPTIMIZED_KP = {best_p['kp']:.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_KD = {best_p['kd']:.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_KI = {best_p.get('ki', 0.10):.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_CRUISE_SPEED = {best_p['cruise']:.4f}f; // m/s\n")
        f.write(f"    constexpr float OPTIMIZED_MIN_CURVE_SPEED = {best_p['min_curve']:.4f}f; // m/s\n")
        f.write(f"    constexpr float OPTIMIZED_SAFETY_FACTOR = {best_p['safety']:.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_BRAKE_GAIN = 0.0000f;\n")
        f.write(f"    constexpr float OPTIMIZED_ERROR_ALPHA = {best_p.get('alpha', 0.40):.4f}f;\n")
        f.write(f"    constexpr float OPTIMIZED_CENTER_KNEE = {best_p.get('knee', 0.15):.4f}f;\n")
        f.write("}\n")
    print(f"[Export] En-tête C++ généré dans : {HEADER_PATH}")

    # Synchronisation directe de picar_constants.hpp
    constants_path = "services/libs/shared-headers/control/picar_constants.hpp"
    if os.path.exists(constants_path):
        import re
        with open(constants_path, "r") as f:
            c = f.read()
        c = re.sub(r"constexpr float DEFAULT_STRAIGHT_CRUISE_SPEED = [0-9.]+f;",
                   f"constexpr float DEFAULT_STRAIGHT_CRUISE_SPEED = {best_p['cruise']:.2f}f;", c)
        c = re.sub(r"constexpr float DEFAULT_MIN_CURVE_SPEED       = [0-9.]+f;",
                   f"constexpr float DEFAULT_MIN_CURVE_SPEED       = {best_p['min_curve']:.2f}f;", c)
        c = re.sub(r"constexpr float DEFAULT_CURVE_SAFETY_FACTOR   = [0-9.]+f;",
                   f"constexpr float DEFAULT_CURVE_SAFETY_FACTOR   = {best_p['safety']:.2f}f;", c)
        c = re.sub(r"constexpr float DEFAULT_LINE_KP               = [0-9.]+f;",
                   f"constexpr float DEFAULT_LINE_KP               = {best_p['kp']:.2f}f;", c)
        c = re.sub(r"constexpr float DEFAULT_LINE_KD               = [0-9.]+f;",
                   f"constexpr float DEFAULT_LINE_KD               = {best_p['kd']:.4f}f;", c)
        with open(constants_path, "w") as f_const:
            f_const.write(c)
        print(f"[Export] 'picar_constants.hpp' synchronisé avec les meilleurs paramètres !")


def main():
    parser = argparse.ArgumentParser(description="Optimiseur Bayésien Optuna pour PiCar-S")
    parser.add_argument("--mode", choices=["fast", "godot"], default="fast",
                        help="fast: simulateur Python pur (~5s); godot: boucle fermée 3D headless (~2m)")
    parser.add_argument("--sampler", choices=["cmaes", "tpe"], default="cmaes",
                        help="cmaes: Covariance Matrix Adaptation (recommandé contrôle PID); tpe: Tree Parzen Estimator")
    parser.add_argument("--trials", type=int, default=300, help="Nombre d'essais (défaut: 300)")
    parser.add_argument("--time-scale", type=float, default=5.0, help="Échelle de temps Godot (défaut: 5.0)")
    parser.add_argument("--timeout", type=float, default=30.0, help="Timeout par run en secondes (défaut: 30.0)")
    args = parser.parse_args()

    sampler_name = "CMA-ES (Covariance Matrix Adaptation)" if args.sampler == "cmaes" else "TPE (Bayesian)"

    print("=" * 70)
    print("      FASTDRINK S5-GI : OPTIMISEUR MULTI-VARIABLES (OPTUNA)")
    print(f"      Mode: {args.mode.upper()} | Sampler: {sampler_name}")
    print(f"      Essais: {args.trials} | Limite stricte: a <= {MAX_ACCELERATION} m/s²")
    print("=" * 70)

    fast_sim = FastSimulator() if args.mode == "fast" else None

    def objective(trial: optuna.Trial) -> float:
        p = {
            'cruise': trial.suggest_float("cruise", 0.38, 0.52),
            'min_curve': trial.suggest_float("min_curve", 0.18, 0.28),
            'safety': trial.suggest_float("safety", 0.85, 0.98),
            'brake_gain': 0.0,
            'kp': trial.suggest_float("kp", 0.45, 0.70),
            'kd': trial.suggest_float("kd", 0.025, 0.055),
            'ki': 0.10,
            'alpha': 0.40,
            'knee': 0.15,
        }

        if args.mode == "fast":
            lap_time, max_accel, failed, reason = fast_sim.simulate(p)
        else:
            lap_time, max_accel, failed, reason = run_godot_trial(p, args.time_scale, args.timeout)

        if failed:
            penalty = 1000.0 if reason == "ball_dropped" else 500.0
            return penalty + (max_accel * 50.0)

        # Mode "ON THE EDGE" : Zéro pénalité tant que la bille ne tombe pas !
        # Le score est purement le temps de tour minimal absolu.
        return lap_time

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    # Initialisation du Sampler (CMA-ES pour paramètres corrélés ou TPE)
    if args.sampler == "cmaes":
        sampler = optuna.samplers.CmaEsSampler()
    else:
        sampler = optuna.samplers.TPESampler()

    study = optuna.create_study(direction="minimize", sampler=sampler)

    # Enqueue le jeu de paramètres actuels comme point de départ
    baseline_params = {
        'cruise': 0.38,
        'min_curve': 0.20,
        'safety': 0.90,
        'kp': 0.55,
        'kd': 0.035,
    }
    study.enqueue_trial(baseline_params)
    
    start_time = time.time()
    study.optimize(objective, n_trials=args.trials, show_progress_bar=True)
    elapsed = time.time() - start_time

    best = study.best_params
    best_score = study.best_value

    print("\n" + "=" * 70)
    print("  🏆 RÉSULTATS OPTIMAUX TROUVÉS PAR OPTUNA")
    print("=" * 70)
    print(f"  • Temps d'optimisation          : {elapsed:.2f} s ({args.trials} essais)")
    print(f"  • Score (Temps de tour estimé)  : {best_score:.2f} s")
    print(f"  • STRAIGHT_CRUISE_SPEED         : {best['cruise']:.4f} m/s")
    print(f"  • MIN_CURVE_SPEED               : {best['min_curve']:.4f} m/s")
    print(f"  • CURVE_SAFETY_FACTOR           : {best['safety']:.4f}")
    print(f"  • PREDICTIVE_BRAKE_GAIN         : {best.get('brake_gain', 0.0):.4f}")
    print(f"  • LINE_KP                       : {best['kp']:.4f}")
    print(f"  • LINE_KD                       : {best['kd']:.4f}")
    print(f"  • DEFAULT_KI                    : {best.get('ki', 0.10):.4f}")
    print(f"  • ANALOG_ERROR_ALPHA            : {best.get('alpha', 0.40):.4f}")
    print(f"  • ANALOG_CENTER_KNEE            : {best.get('knee', 0.15):.4f}")
    print("=" * 70)

    # Sauvegarde dans picar.conf et export C++
    write_picar_conf(best)
    print(f"[Succès] Fichier '{CONF_PATH}' mis à jour avec les paramètres optimaux !")
    export_header(best, best_score)


if __name__ == "__main__":
    main()
