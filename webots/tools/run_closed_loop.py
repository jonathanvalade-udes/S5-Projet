#!/usr/bin/env python3
"""Course en boucle fermee Webots + loop_bridge (vraie loi de suivi de ligne du FSM), sans interface graphique.

Mesure objective du comportement de la simulation : lance le pont de test (tools/loop_bridge.cpp), lance Webots en mode
rapide sans rendu, laisse rouler `--duration` secondes de temps simule, puis resume le journal de verite terrain
(ecart lateral au centre du ruban, oscillation du braquage, nombre d'inversions de braquage).

    python3 webots/tools/run_closed_loop.py --duration 40 --out /tmp/run1

Pre-requis : Webots installe (WEBOTS_HOME ou /Applications/Webots.app) et un compilateur C++17.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import math
import os
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

WEBOTS_DIR = Path(__file__).resolve().parent.parent
REPO = WEBOTS_DIR.parent
TRACK_STRAIGHT_M = 1.4   # doit correspondre a tools/generate_track_texture.py
TRACK_RADIUS_M = 0.5


def find_webots() -> str:
    home = os.environ.get("WEBOTS_HOME")
    candidates = [f"{home}/webots"] if home else []
    candidates += ["/Applications/Webots.app/Contents/MacOS/webots", shutil.which("webots") or ""]
    for c in candidates:
        if c and os.access(c, os.X_OK):
            return c
    sys.exit("Webots introuvable (definir WEBOTS_HOME)")


def lateral_error_m(x: float, y: float, radius: float = TRACK_RADIUS_M) -> float:
    """Distance signee (m) entre le point et l'axe du ruban du circuit ovale (positif = exterieur)."""
    xc = TRACK_STRAIGHT_M / 2.0
    ax, ay = abs(x), abs(y)
    if ax <= xc:
        return ay - radius
    return math.hypot(ax - xc, ay) - radius


def summarize(log_path: Path, sensor_x_m: float, radius: float = TRACK_RADIUS_M) -> dict:
    rows = list(csv.DictReader(log_path.open()))
    start = next((float(r["t"]) for r in rows if float(r["speed"]) > 0.05), 0.0)
    rows = [r for r in rows if float(r["t"]) >= start + 3.0 and float(r["speed"]) > 0.05]   # apres le depart
    if not rows:
        return {"samples": 0}
    lat, steer, analog_levels = [], [], set()
    for r in rows:
        yaw = math.radians(float(r["yaw_deg"]))
        # erreur mesuree au niveau de la barre de capteurs (ce que voit le robot)
        sx = float(r["x"]) + sensor_x_m * math.cos(yaw)
        sy = float(r["y"]) + sensor_x_m * math.sin(yaw)
        lat.append(lateral_error_m(sx, sy, radius) * 1000.0)
        steer.append(float(r["cmd_dir"]) * 45.0)
        for k in range(5):
            analog_levels.add(round(float(r[f"ir{k}"]), 2))
    flips = sum(1 for a, b in zip(steer, steer[1:]) if (a > 2 and b < -2) or (a < -2 and b > 2))
    big_jumps = sum(1 for a, b in zip(steer, steer[1:]) if abs(b - a) > 15.0)
    dur = float(rows[-1]["t"]) - float(rows[0]["t"])
    return {
        "samples": len(rows),
        "duration_s": round(dur, 1),
        "lateral_rms_mm": round(math.sqrt(statistics.fmean(e * e for e in lat)), 1),
        "lateral_max_mm": round(max(abs(e) for e in lat), 1),
        "steer_std_deg": round(statistics.pstdev(steer), 1),
        "steer_abs_mean_deg": round(statistics.fmean(abs(s) for s in steer), 1),
        "steer_saturated_pct": round(100.0 * sum(1 for s in steer if abs(s) > 44.0) / len(steer), 1),
        "steer_jumps_gt15deg_per_s": round(big_jumps / max(dur, 1e-3), 2),
        "steer_sign_flips_per_s": round(flips / max(dur, 1e-3), 2),
        "distinct_analog_levels": len(analog_levels),
        "mean_speed_mps": round(statistics.fmean(float(r["speed"]) for r in rows), 3),
    }


FINISH_BAR = (1.10, 1.32, 0.193, 0.213)   # barre du T d'arrivee du circuit Godot (x min, x max, y min, y max), en m


def summarize_course(log_path: Path) -> dict:
    """Course sur le circuit Godot : arrivee sur le T, bille, pertes de ligne, braquage."""
    rows = list(csv.DictReader(log_path.open()))
    start = next((float(r["t"]) for r in rows if float(r["speed"]) > 0.05), None)
    if start is None:
        return {"samples": 0}
    run = [r for r in rows if float(r["t"]) >= start]
    moving = [r for r in run if float(r["speed"]) > 0.05]
    dist = sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(run, run[1:]))
    last = run[-1]
    x, y = float(last["x"]), float(last["y"])
    fx0, fx1, fy0, fy1 = FINISH_BAR
    yaw = math.radians(float(last["yaw_deg"]))
    sx, sy = x + 0.17 * math.cos(yaw), y + 0.17 * math.sin(yaw)   # barre de capteurs
    stopped_on_t = (fx0 - 0.05 <= sx <= fx1 + 0.05 and fy0 - 0.12 <= sy <= fy1 + 0.12 and float(last["speed"]) < 0.03)
    lost = sum(1 for r in moving if sum(float(r[f"ir{k}"]) for k in range(5)) < 0.2)
    steer = [float(r["cmd_dir"]) * 45.0 for r in moving] or [0.0]
    dropped_at = next((float(r["t"]) - start for r in run if r.get("ball_dropped") == "1"), None)
    ball_r = [math.hypot(float(r["ball_dx_mm"]), float(r["ball_dy_mm"])) for r in run if r.get("ball_dropped") == "0"]
    return {
        "duree_course_s": round(float(moving[-1]["t"]) - start, 1) if moving else 0.0,
        "distance_m": round(dist, 2),
        "arrivee_sur_le_T": stopped_on_t,
        "position_finale_capteurs": [round(sx, 3), round(sy, 3)],
        "bille_tombee": dropped_at is not None,
        "bille_tombee_a_s": None if dropped_at is None else round(dropped_at, 2),
        "bille_decalage_max_mm": round(max(ball_r), 1) if ball_r else None,
        "ligne_perdue_pct": round(100.0 * lost / max(1, len(moving)), 1),
        "braquage_en_butee_pct": round(100.0 * sum(1 for s in steer if abs(s) > 44.0) / len(steer), 1),
        "vitesse_moyenne_mps": round(sum(float(r["speed"]) for r in moving) / max(1, len(moving)), 3),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=40.0, help="temps simule (s)")
    ap.add_argument("--out", type=Path, default=Path("/tmp/picar_closed_loop"))
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--dynamics", default="1", help="WEBOTS_DYNAMICS (1 = modeles moteur/servo/latence)")
    ap.add_argument("--sensor-x", type=float, default=0.17, help="position de la barre IR devant l'essieu arriere (m)")
    ap.add_argument("--wall-timeout", type=float, default=300.0)
    ap.add_argument("--track-radius", type=float, default=TRACK_RADIUS_M,
                    help="rayon des virages de l'ovale (m) ; autre que 0.5 = texture generee a part (ex. 0.15, PRD : R >= 0.12)")
    ap.add_argument("--external", action="store_true",
                    help="pas de pont de test : se connecte a la vraie pile deja lancee (cd services && ./build.sh --sim)")
    ap.add_argument("--course", action="store_true",
                    help="circuit de Godot (worlds/picar_s_course.wbt) : bille physique, barre d'arrivee en T")
    ap.add_argument("--no-obstacles", action="store_true",
                    help="avec --course : retire les 2 obstacles (le pont de test n'a pas l'evitement du FSM)")
    ap.add_argument("--proto", action="append", default=[], metavar="CHAMP=VALEUR",
                    help="surcharge d'un champ du PROTO PiCarS, ex. --proto irPitch=0.015 (repetable)")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    bridge = args.out / "loop_bridge"
    if not args.external:
        subprocess.run(["c++", "-std=c++17", "-O2", "-I", str(REPO / "services/libs/shared-headers"),
                        str(WEBOTS_DIR / "tools/loop_bridge.cpp"), "-o", str(bridge)], check=True)
    truth = args.out / "truth.csv"
    truth.unlink(missing_ok=True)
    bridge_p = None if args.external else subprocess.Popen([str(bridge), str(args.port), str(args.out / "bridge.csv")])
    env = dict(os.environ, SIMBRIDGE_PORT=str(args.port), WEBOTS_AUTO_START="1", WEBOTS_DYNAMICS=args.dynamics,
               WEBOTS_TRUTH_LOG=str(truth))
    world = WEBOTS_DIR / ("worlds/picar_s_course.wbt" if args.course else "worlds/picar_s_track.wbt")
    if args.proto or args.no_obstacles or abs(args.track_radius - TRACK_RADIUS_M) > 1e-6:
        # monde temporaire a cote de l'original (memes chemins relatifs vers le PROTO et la texture)
        text = world.read_text()
        extra = "".join(f"\n  {kv.split('=')[0]} {kv.split('=')[1]}" for kv in args.proto)
        text = text.replace('controller "picar_s_controller"', 'controller "picar_s_controller"' + extra, 1)
        if args.no_obstacles:
            text = re.sub(r"DEF OBSTACLE_\d Solid \{.*?\n\}\n", "", text, flags=re.S)
        if abs(args.track_radius - TRACK_RADIUS_M) > 1e-6 and not args.course:
            # piste a virages serres : texture mise en cache, depart sur la ligne droite du bas (y = -R)
            tex_name = f"_line_track_r{int(round(args.track_radius * 1000))}.png"
            tex = WEBOTS_DIR / "worlds/textures" / tex_name
            if not tex.exists():
                sys.path.insert(0, str(WEBOTS_DIR / "tools"))
                from generate_track_texture import generate_track_texture
                generate_track_texture(tex, 2048, 2048, 3.0, TRACK_STRAIGHT_M, args.track_radius, 0.018)
            text = text.replace('"textures/line_track.png"', f'"textures/{tex_name}"')
            text = text.replace("translation 0 -0.5 0.0225", f"translation 0 {-args.track_radius} 0.0225")
        world = WEBOTS_DIR / f"worlds/_closed_loop_{args.port}.wbt"   # nom unique : courses en parallele
        world.write_text(text)
    webots_log = (args.out / "webots.log").open("w")
    # Vraie pile (--external) : le FSM tourne en temps reel (horloge murale), Webots doit donc aussi etre en temps reel.
    # En mode "fast", le monde defilerait ~10 fois trop vite pour le FSM (rampes trop lentes, freinage trop tardif).
    mode = "--mode=realtime" if args.external else "--mode=fast"
    wb = subprocess.Popen([find_webots(), "--batch", mode, "--no-rendering", "--minimize", "--stdout",
                           "--stderr", str(world)],
                          env=env, stdout=webots_log, stderr=subprocess.STDOUT)
    t0 = time.time()
    try:
        while time.time() - t0 < args.wall_timeout:
            time.sleep(1.0)
            if wb.poll() is not None:
                break
            try:
                # duree comptee a partir du depart reel (la connexion TCP peut prendre plusieurs secondes)
                rows = [r.split(",") for r in truth.read_text().strip().splitlines()[1:]]
                start = next(float(r[0]) for r in rows if float(r[4]) > 0.05)
                if float(rows[-1][0]) - start >= args.duration + 3.0:
                    break
            except (OSError, IndexError, ValueError, StopIteration):
                pass
    finally:
        wb.terminate()
        try:
            wb.wait(10)
        except subprocess.TimeoutExpired:
            wb.kill()
        if bridge_p is not None:
            bridge_p.terminate()
        if world.name.startswith("_closed_loop"):
            world.unlink(missing_ok=True)
    if not truth.exists():
        sys.exit(f"aucun journal produit, voir {args.out / 'webots.log'}")
    summary = summarize_course(truth) if args.course else summarize(truth, args.sensor_x, args.track_radius)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))
    for k, v in summary.items():
        print(f"{k:28s} {v}")


if __name__ == "__main__":
    main()
