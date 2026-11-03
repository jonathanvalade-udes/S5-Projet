#!/usr/bin/env python3
"""
optimize_webots.py
Optimiseur Bayésien Sim-to-Real Optuna directement couplé à Webots Headless.
Exécute des simulations Webots (--batch --mode=fast --no-rendering) avec le vrai moteur ODE 3D.
Trouve les gains Kp, Kd, V_cruise et V_min optimaux garantissant :
  1. Suivi de ligne complet (0 perte de ligne).
  2. Maintien parfait de la bille dans la coupelle (a <= 0.735 m/s²).
  3. Temps de tour minimal.
"""

import os
import sys
import time
import subprocess
import optuna

WEBOTS_BINARY = "/Applications/Webots.app/Contents/MacOS/webots"
WORLD_FILE = "/Users/jonathanvalade/Documents/Github/S5-Projet/webots_sim/worlds/fastdrink_track.wbt"
RESULT_FILE = "/tmp/optuna_result.txt"
EXPORT_HEADER = "/Users/jonathanvalade/Documents/Github/S5-Projet/firmware/include/control/optimized_params.hpp"

optuna.logging.set_verbosity(optuna.logging.INFO)

def objective(trial):
    # Espace de recherche des hyperparamètres
    kp = trial.suggest_float("kp", 0.6, 2.5)
    kd = trial.suggest_float("kd", 0.02, 0.35)
    cruise_speed = trial.suggest_float("cruise_speed", 0.16, 0.24)
    min_curve_speed = trial.suggest_float("min_curve_speed", 0.08, 0.14)
    slowdown_gain = trial.suggest_float("slowdown_gain", 0.06, 0.25)

    if os.path.exists(RESULT_FILE):
        try:
            os.remove(RESULT_FILE)
        except OSError:
            pass

    env = os.environ.copy()
    env["OPTUNA_RUN"] = "1"
    env["OPTUNA_KP"] = f"{kp:.4f}"
    env["OPTUNA_KD"] = f"{kd:.4f}"
    env["OPTUNA_CRUISE"] = f"{cruise_speed:.4f}"
    env["OPTUNA_MIN_SPEED"] = f"{min_curve_speed:.4f}"
    env["OPTUNA_SLOWDOWN"] = f"{slowdown_gain:.4f}"

    cmd = [
        WEBOTS_BINARY,
        "--batch",
        "--mode=fast",
        "--no-rendering",
        WORLD_FILE,
        "--stdout",
        "--stderr"
    ]

    t0 = time.time()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=20.0)
    except subprocess.TimeoutExpired:
        subprocess.run(["pkill", "-9", "webots"], capture_output=True)
        return 10000.0

    duration = time.time() - t0

    # Lecture du fichier de résultat écrit par picar_webots.cpp
    if not os.path.exists(RESULT_FILE):
        return 10000.0

    success = False
    lap_time = 40.0
    mse = 9.0
    ball_ok = False
    sectors = 0

    with open(RESULT_FILE) as f:
        line = f.read().strip()
        parts = line.split()
        for i in range(len(parts)):
            if parts[i] == "success" and i + 1 < len(parts):
                success = (parts[i+1] == "1")
            elif parts[i] == "lap_time" and i + 1 < len(parts):
                lap_time = float(parts[i+1])
            elif parts[i] == "mse" and i + 1 < len(parts):
                mse = float(parts[i+1])
            elif parts[i] == "ball_ok" and i + 1 < len(parts):
                ball_ok = (parts[i+1] == "1")
            elif parts[i] == "sectors" and i + 1 < len(parts):
                sectors = int(parts[i+1])

    # Calcul de la fonction de coût multi-critères
    if success and ball_ok:
        # SUCCÈS TOTAL : Tour complet avec bille intacte !
        cost = lap_time + 10.0 * mse
        print(f"  >>> TRIAL {trial.number} RÉUSSI ! Temps={lap_time:.2f}s | MSE={mse:.4f} | Cost={cost:.2f} (Sim: {duration:.1f}s)")
        return cost

    if not ball_ok:
        # Échec critique : Déversement de bille (interdit S6)
        cost = 5000.0 + 100.0 * (4 - sectors)
        return cost

    # Échec sortie de piste / perte de ligne
    cost = 2000.0 + 150.0 * (4 - sectors) + 20.0 * mse
    return cost

def export_to_cpp(best_params):
    content = f"""// Généré automatiquement par tools/optimizer/optimize_webots.py via Optuna sur Webots ODE 3D
#pragma once

namespace Config {{
    constexpr float OPTIMIZED_KP = {best_params['kp']:.4f}f;
    constexpr float OPTIMIZED_KD = {best_params['kd']:.4f}f;
    constexpr float OPTIMIZED_CRUISE_SPEED = {best_params['cruise_speed']:.4f}f; // m/s
    constexpr float OPTIMIZED_MIN_CURVE_SPEED = {best_params['min_curve_speed']:.4f}f; // m/s
    constexpr float OPTIMIZED_SLOWDOWN_GAIN = {best_params['slowdown_gain']:.4f}f;
}}
"""
    with open(EXPORT_HEADER, "w") as f:
        f.write(content)
    print(f"\n✓ Meilleurs hyperparamètres exportés dans : {EXPORT_HEADER}")

def main():
    trials = 25
    if len(sys.argv) > 1:
        trials = int(sys.argv[1])

    print("=" * 65)
    print("  FastDrink S5-GI : Optimiseur Sim-to-Real Optuna <-> Webots 3D")
    print(f"  Lancement de {trials} simulations physiques headless...")
    print("=" * 65)

    study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=42))
    study.optimize(objective, n_trials=trials, show_progress_bar=True)

    print("\n" + "=" * 65)
    print("  🏆 RÉSULTATS OPTIMAUX TROUVÉS SUR LE VRAI MOTEUR PHYSIQUE WEBOTS")
    print("=" * 65)
    best = study.best_params
    print(f"  • Meilleur Score          : {study.best_value:.2f}")
    print(f"  • Kp                      : {best['kp']:.4f}")
    print(f"  • Kd                      : {best['kd']:.4f}")
    print(f"  • Vitesse croisière       : {best['cruise_speed']:.4f} m/s")
    print(f"  • Vitesse minimale virage : {best['min_curve_speed']:.4f} m/s")
    print(f"  • Facteur ralentissement  : {best['slowdown_gain']:.4f}")
    print("=" * 65)

    export_to_cpp(best)

if __name__ == "__main__":
    main()
