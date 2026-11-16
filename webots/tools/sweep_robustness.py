#!/usr/bin/env python3
"""Balayage de robustesse : gains LINE_KP / LINE_KD contre les parametres reels inconnus.

On ne connait ni le pas exact des sondes, ni la latence reelle, ni la taille de la tache des TCRT5000. Plutot que de deviner,
on cherche des gains qui restent stables dans TOUTE la plage plausible : chaque couple (KP, KD) roule dans chaque scenario
(tools/run_closed_loop.py, Webots reel) et on garde le pire cas.

    python3 webots/tools/sweep_robustness.py --grid coarse --jobs 4 --out /tmp/sweep
    python3 webots/tools/sweep_robustness.py --grid full --gains 0.45:0.08 0.58:0.035 --out /tmp/sweep_full

Resultats : <out>/results.csv (une ligne par course) et un tableau du pire cas par couple de gains.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
TAPE_M = 0.018   # largeur du ruban (worlds/textures/line_track.png)

# Plages plausibles : pas = ~ largeur du ruban (manuel SunFounder : 18-30 mm, module de 125 mm), latence ajoutee 20-80 ms,
# tache vue par un TCRT5000 a ~1 cm du sol : 5-12 mm.
GRIDS = {
    "coarse": [  # coins de la plage + nominal
        dict(pitch=0.019, latency=0.035, spot=0.007),
        dict(pitch=0.017, latency=0.020, spot=0.005),
        dict(pitch=0.017, latency=0.080, spot=0.012),
        dict(pitch=0.025, latency=0.020, spot=0.012),
        dict(pitch=0.025, latency=0.080, spot=0.012),   # (0.025 / 0.005 : angle mort, exclu, voir "full")
    ],
    # Grille complete, SANS les combinaisons a angle mort (pas > ruban + tache) : un ruban de 18 mm entre deux sondes n'y
    # touche aucune tache, la ligne disparait (30 % des trames a 25 mm / 5 mm). Le vrai robot suit la ligne, donc ces
    # combinaisons ne sont pas la realite.
    "full": [dict(pitch=p, latency=l, spot=s) for p, l, s in
             itertools.product((0.017, 0.019, 0.022, 0.025), (0.020, 0.050, 0.080), (0.005, 0.012))
             if p <= TAPE_M + s],
}
DEFAULT_GAINS = [(kp, kd) for kp in (0.35, 0.45, 0.58) for kd in (0.035, 0.08, 0.15)]


def run_one(job: dict, out_root: Path, duration: float, port: int, radius: float) -> dict:
    tag = f"kp{job['kp']}_kd{job['kd']}_p{job['pitch']}_l{job['latency']}_s{job['spot']}"
    out = out_root / tag
    out.mkdir(parents=True, exist_ok=True)
    cal = out / "calibration.json"
    cal.write_text(json.dumps({"latency_s": job["latency"]}))
    env = dict(os.environ, WEBOTS_CALIBRATION=str(cal), LOOP_BRIDGE_KP=str(job["kp"]), LOOP_BRIDGE_KD=str(job["kd"]))
    cmd = [sys.executable, str(TOOLS / "run_closed_loop.py"), "--duration", str(duration), "--port", str(port),
           "--wall-timeout", "600", "--out", str(out), "--track-radius", str(radius),
           "--proto", f"irPitch={job['pitch']}", "--proto", f"irSpotDiameter={job['spot']}"]
    subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        summary = json.loads((out / "summary.json").read_text())
    except (OSError, ValueError):
        summary = {}
    return {**job, **summary}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", choices=GRIDS, default="coarse")
    ap.add_argument("--gains", nargs="*", help="couples KP:KD (defaut : 3 x 3 autour de fsm.conf)")
    ap.add_argument("--duration", type=float, default=40.0, help="temps de roulage par course (s), ~2 tours")
    ap.add_argument("--track-radius", type=float, default=0.5, help="rayon des virages de l'ovale (m)")
    ap.add_argument("--port-base", type=int, default=9100, help="premier port TCP (un par course ; changer si 2 balayages tournent)")
    ap.add_argument("--jobs", type=int, default=4, help="courses Webots en parallele")
    ap.add_argument("--out", type=Path, default=Path("/tmp/picar_sweep"))
    args = ap.parse_args()

    gains = [tuple(map(float, g.split(":"))) for g in args.gains] if args.gains else DEFAULT_GAINS
    jobs = [dict(kp=kp, kd=kd, **sc) for (kp, kd) in gains for sc in GRIDS[args.grid]]
    args.out.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} courses ({len(gains)} couples de gains x {len(GRIDS[args.grid])} scenarios), {args.jobs} en parallele")

    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_one, job, args.out, args.duration, args.port_base + i, args.track_radius)
                   for i, job in enumerate(jobs)]
        for i, f in enumerate(futures, 1):
            r = f.result()
            results.append(r)
            print(f"[{i}/{len(jobs)}] KP={r['kp']} KD={r['kd']} pas={r['pitch']} lat={r['latency']} tache={r['spot']} -> "
                  f"butee {r.get('steer_saturated_pct', '?')} % RMS {r.get('lateral_rms_mm', '?')} mm "
                  f"roulage {r.get('duration_s', '?')} s", flush=True)

    keys = sorted({k for r in results for k in r})
    with (args.out / "results.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(results)

    print("\nPIRE CAS par couple de gains (une course qui s'arrete avant la fin = echec : ligne perdue)")
    print(f"{'KP':>5} {'KD':>6} | {'butee max %':>11} {'RMS max mm':>10} {'ecart max mm':>12} {'echecs':>6}")
    rows = []
    for (kp, kd) in gains:
        rs = [r for r in results if r["kp"] == kp and r["kd"] == kd]
        fails = sum(1 for r in rs if r.get("duration_s", 0) < args.duration - 8)
        sat = max(r.get("steer_saturated_pct", 100.0) for r in rs)
        rms = max(r.get("lateral_rms_mm", 999.0) for r in rs)
        mx = max(r.get("lateral_max_mm", 999.0) for r in rs)
        rows.append((fails, sat, rms, kp, kd, mx))
    for fails, sat, rms, kp, kd, mx in sorted(rows):
        print(f"{kp:5.2f} {kd:6.3f} | {sat:11.1f} {rms:10.1f} {mx:12.1f} {fails:6d}")


if __name__ == "__main__":
    main()
