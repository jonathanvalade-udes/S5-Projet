"""Modèles dynamiques du PiCar-S : moteur, direction et bille FastDrink.

Python pur (aucune dépendance Webots) pour être testable. Le contrôleur Webots les place entre la consigne reçue du
FSM et les moteurs simulés, afin que la simulation ait les défauts du vrai robot : zone morte, retard, inertie,
servo lent, latence de la boucle.

IMPORTANT : les valeurs par défaut de `Calibration` sont PROVISOIRES. Seule la vitesse de 0,32 m/s à 50 % de puissance est
mesurée. Tant que `calibration.json` n'est pas rempli avec les mesures du vrai robot (voir tools/calibrate_dynamics.py),
la simulation sert à COMPARER des réglages entre eux, pas à fixer leurs valeurs finales.
"""

from __future__ import annotations

import json
import math
import os
from collections import deque
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Deque, Optional, Tuple

G = 9.81


@dataclass
class Calibration:
    """Paramètres physiques du robot. `measured` passe à True quand le fichier vient de vraies mesures."""

    measured: bool = False
    # --- Moteur ---
    motor_deadband_mps: float = 0.128   # sous cette consigne le moteur ne démarre pas (20 % de puissance = 0,128 m/s)
    motor_tau_s: float = 0.15           # constante de temps de la réponse en vitesse (À MESURER : échelon de puissance)
    motor_gain: float = 1.0             # vitesse réelle / vitesse demandée (la table PWM->vitesse n'est pas linéaire)
    # --- Direction ---
    # Chaine reelle : FSM -> MotorActuator (borne ±45° puis limite la vitesse de braquage) -> servo -> biellettes -> roues.
    steer_max_deg: float = 45.0         # braquage maximal commande (le HAL borne à ±45°)
    actuator_slew_straight_dps: float = 175.0   # MotorActuator::update_actuation : 175 °/s si |consigne| < 15°
    actuator_slew_curve_dps: float = 300.0      # ... et 300 °/s au-dela
    actuator_curve_threshold_deg: float = 15.0
    steer_slew_dps: float = 500.0       # servo SF0180 (manuel) : 0,12 s / 60° a 4,8 V (alimente en 5 V par le HAT)
    steer_ratio: float = 1.0            # angle roue / angle servo. Manuel : a 45° le rayon = empattement (138 mm) -> ~1
    steer_backlash_deg: float = 0.0     # jeu du servo
    steer_bias_deg: float = 0.0         # défaut d'alignement des roues
    # --- Boucle ---
    # Retard AJOUTE par le modele. La chaine simulee a deja ~20-40 ms (pas Webots de 20 ms + tick FSM asynchrone a 50 Hz) ;
    # le reel a en plus le service Motors (100 Hz, ~5 ms), la trame PWM du servo (50 Hz, ~10 ms) et le temps mort du servo.
    latency_s: float = 0.035            # délai supplementaire entre l'ordre du FSM et son effet (À MESURER : echelon)
    wheelbase_m: float = 0.14
    # --- Bille ---
    cup_radius_m: float = 0.020
    cup_depth_m: float = 0.0015         # tan(theta) = 0,075 -> a_crit = g * tan(theta) = 0,7358 m/s²
    ball_damping: float = 6.0           # hypothèse : rien n'a été mesuré sur la vraie bille

    @property
    def a_crit(self) -> float:
        return G * self.cup_depth_m / self.cup_radius_m

    @classmethod
    def load(cls, path: Optional[str] = None) -> "Calibration":
        """Lit calibration.json (ou le chemin de WEBOTS_CALIBRATION) ; valeurs provisoires si absent."""
        path = path or os.environ.get("WEBOTS_CALIBRATION") or str(Path(__file__).with_name("calibration.json"))
        cal = cls()
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return cal
        known = {f.name for f in fields(cls)}
        for key, value in data.items():
            if key in known:
                setattr(cal, key, type(getattr(cal, key))(value))
        return cal

    def save(self, path: str) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")


class DriveModel:
    """Vitesse réelle à partir de la vitesse demandée : latence pure, zone morte, gain, puis réponse du premier ordre."""

    def __init__(self, cal: Calibration):
        self.cal = cal
        self._queue: Deque[Tuple[float, float]] = deque()
        self._clock = 0.0
        self.speed = 0.0

    def reset(self) -> None:
        self._queue.clear()
        self._clock = 0.0
        self.speed = 0.0

    def _delayed_command(self, cmd: float) -> float:
        self._queue.append((self._clock, cmd))
        due = self._clock - self.cal.latency_s
        # on garde l'ordre le plus récent qui date d'au moins `latency_s`
        while len(self._queue) > 1 and self._queue[1][0] <= due:
            self._queue.popleft()
        return self._queue[0][1] if self._queue[0][0] <= due else 0.0

    def update(self, cmd_speed: float, dt: float) -> float:
        self._clock += dt
        cmd = self._delayed_command(cmd_speed)
        if abs(cmd) < self.cal.motor_deadband_mps:
            cmd = 0.0
        target = cmd * self.cal.motor_gain
        alpha = 1.0 - math.exp(-dt / max(self.cal.motor_tau_s, 1e-4))
        self.speed += alpha * (target - self.speed)
        return self.speed


class SteeringModel:
    """Angle de braquage réel des roues (rad, bicyclette) : latence, limiteur de MotorActuator, servo (jeu, vitesse), biellettes."""

    def __init__(self, cal: Calibration):
        self.cal = cal
        self._queue: Deque[Tuple[float, float]] = deque()
        self._clock = 0.0
        self._actuated = 0.0   # consigne servo apres le limiteur de MotorActuator (deg)
        self._play = 0.0       # position de l'ordre dans le jeu du servo (rad)
        self.servo = 0.0       # angle du palonnier du servo (rad)
        self.angle = 0.0       # angle réel des roues (rad)

    def reset(self) -> None:
        self._queue.clear()
        self._clock = 0.0
        self._actuated = 0.0
        self._play = 0.0
        self.servo = 0.0
        self.angle = 0.0

    def update(self, direction: float, dt: float) -> float:
        """direction : commande normalisée [-1, 1] (l'angle maximal vaut steer_max_deg)."""
        self._clock += dt
        self._queue.append((self._clock, max(-1.0, min(1.0, direction))))
        due = self._clock - self.cal.latency_s
        while len(self._queue) > 1 and self._queue[1][0] <= due:
            self._queue.popleft()
        cmd = self._queue[0][1] if self._queue[0][0] <= due else 0.0

        # 1. MotorActuator : consigne en degres, vitesse limitee selon l'amplitude de la consigne
        target_deg = cmd * self.cal.steer_max_deg
        rate = (self.cal.actuator_slew_straight_dps if abs(target_deg) < self.cal.actuator_curve_threshold_deg
                else self.cal.actuator_slew_curve_dps)
        step_deg = rate * dt
        self._actuated += max(-step_deg, min(step_deg, target_deg - self._actuated))

        # 2. Servo : biais d'alignement, jeu, vitesse mecanique
        max_rad = math.radians(self.cal.steer_max_deg)
        target = math.radians(self._actuated + self.cal.steer_bias_deg)
        half_play = math.radians(self.cal.steer_backlash_deg) / 2.0
        if target > self._play + half_play:
            self._play = target - half_play
        elif target < self._play - half_play:
            self._play = target + half_play
        step = math.radians(self.cal.steer_slew_dps) * dt
        delta = max(-step, min(step, self._play - self.servo))
        self.servo = max(-max_rad, min(max_rad, self.servo + delta))

        # 3. Biellettes : angle des roues
        self.angle = self.servo * self.cal.steer_ratio
        return self.angle

    def yaw_rate(self, speed: float) -> float:
        """Vitesse de lacet (rad/s) du modèle bicyclette d'Ackermann."""
        return speed / self.cal.wheelbase_m * math.tan(self.angle)


class BallModel:
    """Bille FastDrink dans la coupelle conique. Même modèle que godot/scripts/ball_monitor.gd.

    La coupelle est trop peu profonde (1,5 mm) pour que le moteur physique de Webots la simule : on calcule la bille
    à partir de l'accélération du châssis. Elle tombe dès qu'elle s'éloigne de plus de `cup_radius_m` du centre.
    """

    ROLLING_FACTOR = 5.0 / 7.0

    def __init__(self, cal: Calibration):
        self.cal = cal
        self.reset()

    def reset(self) -> None:
        self.pos = [0.0, 0.0]          # (x avant/arrière, z latéral) par rapport au centre (m)
        self.vel = [0.0, 0.0]
        self.dropped = False
        self.max_accel = 0.0           # accélération totale maximale vue par la bille (m/s²)
        self.peak_ratio = 0.0          # max(a_total / a_crit)
        self._last_speed: Optional[float] = None

    def update(self, speed: float, yaw_rate: float, dt: float) -> bool:
        """Avance d'un pas ; retourne True si la bille est tombée (état définitif jusqu'à reset())."""
        if self.dropped:
            return True
        dt = max(dt, 1e-4)
        if self._last_speed is None:
            self._last_speed = speed
        a_long = (speed - self._last_speed) / dt
        self._last_speed = speed
        a_lat = speed * yaw_rate
        a_total = math.hypot(a_long, a_lat)
        self.max_accel = max(self.max_accel, a_total)
        self.peak_ratio = max(self.peak_ratio, a_total / self.cal.a_crit)

        dist = math.hypot(*self.pos)
        slope = self.cal.cup_depth_m / self.cal.cup_radius_m
        pull = [0.0, 0.0]
        if dist > 1e-4:
            pull = [-G * slope * self.pos[0] / dist, -G * slope * self.pos[1] / dist]
        for axis, inertia in ((0, a_long), (1, a_lat)):
            acc = self.ROLLING_FACTOR * (inertia + pull[axis] - self.cal.ball_damping * self.vel[axis])
            self.vel[axis] += acc * dt
            self.pos[axis] += self.vel[axis] * dt
        if dist > self.cal.cup_radius_m:   # position testée = celle du pas précédent, comme dans Godot
            self.dropped = True
        return self.dropped


class PicarDynamics:
    """Assemble moteur + direction + bille. Un appel `step()` par pas de simulation."""

    def __init__(self, cal: Optional[Calibration] = None):
        self.cal = cal or Calibration.load()
        self.drive = DriveModel(self.cal)
        self.steering = SteeringModel(self.cal)
        self.ball = BallModel(self.cal)

    def reset(self) -> None:
        self.drive.reset()
        self.steering.reset()
        self.ball.reset()

    def step(self, cmd_speed: float, cmd_direction: float, dt: float) -> Tuple[float, float, bool]:
        """Retourne (vitesse réelle m/s, angle de braquage réel rad, bille tombée)."""
        speed = self.drive.update(cmd_speed, dt)
        angle = self.steering.update(cmd_direction, dt)
        dropped = self.ball.update(speed, self.steering.yaw_rate(speed), dt)
        return speed, angle, dropped
