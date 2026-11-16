"""Banc d'essai de la bille FastDrink (lance par tools/ball_lab.py, monde temporaire base sur picar_s_course.wbt).

La voiture (superviseur) pilote elle-meme ses moteurs, sans FSM, et mesure la bille physique dans la coupelle :
  1. repos       : 3 s a l'arret, derive de la bille ;
  2. oscillation : la bille est lancee lateralement, on mesure la periode et l'amortissement ;
  3. echelons    : acceleration constante a (m/s2) depuis l'arret, jusqu'a 0,6 m/s ; la bille sort-elle ?
Resultats en JSON sur la sortie standard (ligne "BALL_LAB {...}").
"""

import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "picar_s_controller"))
from ball_tracker import ball_in_cup_frame, is_dropped  # noqa: E402
from picar_protocol import PicarProtocol  # noqa: E402

from controller import Supervisor  # type: ignore  # noqa: E402

r = Supervisor()
ts = int(r.getBasicTimeStep())
dt = ts / 1000.0
ball = r.getFromDef("BALL")
me = r.getSelf()
steer = [r.getDevice(n) for n in ("left_steer", "right_steer")]
drive = [r.getDevice(n) for n in ("rear_left_motor", "rear_right_motor")]
for m in drive:
    m.setPosition(float("inf"))


def set_speed(v):
    for m in drive:
        m.setVelocity(v / PicarProtocol.WHEEL_RADIUS)
    for m in steer:
        m.setPosition(0.0)


def rel():
    return ball_in_cup_frame(me.getPosition(), me.getOrientation(), ball.getPosition())


def run(seconds, speed_fn=lambda t: 0.0):
    """Avance de `seconds`, renvoie [(t, rel, vitesse_voiture)]."""
    out, t = [], 0.0
    while t < seconds:
        set_speed(speed_fn(t))
        if r.step(ts) == -1:
            sys.exit(0)
        t += dt
        out.append((t, rel(), me.getVelocity()[0]))
    return out


CAR_T = me.getField("translation").getSFVec3f()
CAR_R = me.getField("rotation").getSFRotation()
BALL_T = ball.getField("translation").getSFVec3f()


def reset():
    """Remet voiture et bille au depart (simulationReset() arreterait ce controleur)."""
    set_speed(0.0)
    me.getField("translation").setSFVec3f(CAR_T)
    me.getField("rotation").setSFRotation(CAR_R)
    me.resetPhysics()
    ball.getField("translation").setSFVec3f(BALL_T)
    ball.resetPhysics()
    run(1.5)   # la bille se pose
    p = rel()
    log(f"  remise a zero : bille a {math.hypot(p[0], p[1]) * 1000:.2f} mm du centre, dz {p[2] * 1000:.2f} mm")


import time
_t0 = time.time()
_LOG = os.environ.get("BALL_LAB_LOG")
def log(msg):
    line = f"[ball_lab {time.time() - _t0:6.1f} s reels | t_sim={r.getTime():.2f}] {msg}"
    print(line, flush=True)
    if _LOG:
        with open(_LOG, "a") as fh:
            fh.write(line + "\n")

results = {}
log("debut")
reset()
log("pose")
trace = run(3.0)
drift = max(math.hypot(p[1][0], p[1][1]) for p in trace)
results["repos_derive_max_mm"] = round(drift * 1000, 3)
log(f"repos : derive {drift * 1000:.3f} mm")

# 2. Oscillation libre : vitesse laterale initiale de 3 cm/s
reset()
PUSH = float(os.environ.get("BALL_LAB_PUSH", "0.03"))
ball.setVelocity([0, PUSH, 0, 0, 0, 0])
trace = run(4.0)
ys = [p[1][1] for p in trace]
peaks = [(trace[i][0], ys[i]) for i in range(1, len(ys) - 1) if ys[i] > ys[i - 1] and ys[i] >= ys[i + 1] and ys[i] > 1e-4]
if len(peaks) >= 2:
    period = (peaks[-1][0] - peaks[0][0]) / (len(peaks) - 1)
    ratios = [peaks[i + 1][1] / peaks[i][1] for i in range(len(peaks) - 1) if peaks[i][1] > 0]
    dec = -math.log(sum(ratios) / len(ratios)) if ratios and min(ratios) > 0 else float("nan")
    log(f"oscillation : periode {period:.3f} s, pics {[round(p[1] * 1000, 2) for p in peaks[:6]]} mm")
    results["oscillation_periode_s"] = round(period, 3)
    results["oscillation_amplitude_mm"] = [round(p[1] * 1000, 2) for p in peaks[:6]]
    results["amortissement_zeta"] = round(dec / math.sqrt(4 * math.pi ** 2 + dec ** 2), 3)
else:
    results["oscillation_pics"] = peaks

# 3. Echelons d'acceleration
steps = {}
for a in [float(x) for x in os.environ.get("BALL_LAB_ACCELS", "0.5,0.65,0.8,1.0,1.2,1.5,2.0").split(",")]:
    reset()
    v_max = 0.6
    trace = run(v_max / a + 1.0, lambda t, a=a: min(v_max, a * t))
    dropped = any(is_dropped(p[1]) for p in trace)
    t_drop = next((p[0] for p in trace if is_dropped(p[1])), None)
    a_cmd_drop = None if t_drop is None else round(min(v_max, a * t_drop), 3)
    rmax = max(math.hypot(p[1][0], p[1][1]) for p in trace)
    # acceleration reellement obtenue par le chassis (pente de la vitesse entre 20 % et 80 % de v_max)
    sp = [(p[0], p[2]) for p in trace]
    t1 = next((t for t, v in sp if v > 0.2 * v_max), None)
    t2 = next((t for t, v in sp if v > 0.8 * v_max), None)
    a_real = 0.6 * v_max / (t2 - t1) if t1 and t2 and t2 > t1 else float("nan")
    log(f"echelon {a} : tombee={dropped} (t={t_drop}, v={a_cmd_drop}) rmax={rmax * 1000:.1f} mm")
    steps[str(a)] = {"a_reelle": round(a_real, 3), "decalage_max_mm": round(rmax * 1000, 2), "tombee": dropped}
results["echelons"] = steps
line = "BALL_LAB " + json.dumps(results)
print(line, flush=True)
if os.environ.get("BALL_LAB_OUT"):
    Path(os.environ["BALL_LAB_OUT"]).write_text(line)
r.simulationQuit(0)
