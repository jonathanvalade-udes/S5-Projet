#!/usr/bin/env python3
"""Lance le banc d'essai de la bille (controllers/ball_lab) dans Webots, sans interface, et affiche les resultats.

    python3 webots/tools/ball_lab.py
    BALL_LAB_ACCELS=0.6,0.7,0.8 python3 webots/tools/ball_lab.py

Monde temporaire : picar_s_course.wbt, avec le controleur ball_lab a la place de picar_s_controller (la voiture accelere
en ligne droite sur la grande droite du depart).
"""

import json
import re
import os
import subprocess
import sys
from pathlib import Path

WEBOTS = Path(__file__).resolve().parent.parent


def main() -> None:
    sys.path.insert(0, str(WEBOTS / "tools"))
    from run_closed_loop import find_webots

    out_file = WEBOTS / "worlds" / "_ball_lab_result.json"
    out_file.unlink(missing_ok=True)
    world = WEBOTS / "worlds" / "_ball_lab.wbt"
    text = (WEBOTS / "worlds" / "picar_s_course.wbt").read_text()
    text = text.replace('controller "picar_s_controller"', 'controller "ball_lab"', 1)
    # sans obstacles : la voiture accelere en ligne droite sur ~1 m et percuterait l'obstacle 1 (x = 0)
    text = re.sub(r"DEF OBSTACLE_\d Solid \{.*?\n\}\n", "", text, flags=re.S)
    world.write_text(text)
    try:
        proc = subprocess.run([find_webots(), "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout",
                               "--stderr", str(world)], capture_output=True, text=True, timeout=600, env=dict(os.environ, BALL_LAB_OUT=str(out_file)))
    finally:
        world.unlink(missing_ok=True)
    line = out_file.read_text() if out_file.exists() else None
    out_file.unlink(missing_ok=True)
    if line is None:
        sys.exit("pas de resultat :\n" + proc.stdout[-3000:] + proc.stderr[-2000:])
    res = json.loads(line[len("BALL_LAB "):])
    for k, v in res.items():
        if k != "echelons":
            print(f"{k:28s} {v}")
    print("\nacceleration demandee -> obtenue | decalage max de la bille | tombee")
    for a, e in res["echelons"].items():
        print(f"  {float(a):5.2f} m/s2 -> {e['a_reelle']:5.2f} | {e['decalage_max_mm']:6.2f} mm | {e['tombee']}")


if __name__ == "__main__":
    main()
