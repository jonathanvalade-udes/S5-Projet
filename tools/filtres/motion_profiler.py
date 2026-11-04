"""
motion_profiler.py
Générateur de profils de vitesse trapézoïdal et régulateur d'accélération pour FastDrink.
Garantit le respect strict de la contrainte physique : a <= 0.735 m/s² (seuil de renversement de la bille).
"""

import math
import time
from typing import Optional


class MotionProfiler:
    """
    Régulateur cinématique bornant l'accélération et la décélération à un seuil maximal.
    Calcule également la vitesse limite en courbe pour contrer l'accélération centrifuge.
    """

    # Contrainte physique démontrée par conservation d'énergie mécanique (CalculAcceleration.pdf)
    MAX_ACCELERATION: float = 0.735  # m/s²
    
    # Rayon minimal de virage imposé par le cahier des charges S3 (120 mm)
    MIN_TURN_RADIUS: float = 0.12  # m

    # Vitesse maximale en ligne droite (m/s)
    CRUISE_SPEED: float = 5  # m/s

    def __init__(self, max_accel: float = MAX_ACCELERATION, max_speed: float = CRUISE_SPEED):
        self.max_accel = max_accel
        self.max_speed = max_speed
        self.current_speed: float = 0.0
        self.last_update_time: Optional[float] = None

    def reset(self, initial_speed: float = 0.0):
        """Réinitialise la vitesse courante."""
        self.current_speed = initial_speed
        self.last_update_time = None

    def calculate_curve_speed_limit(self, radius_m: float) -> float:
        """
        Calcule la vitesse maximale autorisée en virage pour que l'accélération centrifuge
        ne dépasse pas le seuil maximal (a_c = v² / R <= a_max).
        v_max = sqrt(a_max * R)
        """
        if radius_m <= 0:
            return 0.0
        return math.sqrt(self.max_accel * radius_m)

    def calculate_stopping_distance(self, speed_m_s: Optional[float] = None) -> float:
        """
        Calcule la distance minimale d'arrêt requise à décélération maximale autorisée :
        d = v² / (2 * a_max)
        """
        v = self.current_speed if speed_m_s is None else speed_m_s
        return (v ** 2) / (2.0 * self.max_accel)

    def calculate_max_safe_approach_speed(self, distance_to_target_m: float) -> float:
        """
        Calcule la vitesse maximale avec laquelle le véhicule peut s'approcher d'un point
        d'arrêt tout en garantissant un arrêt complet sans dépasser a_max :
        v_max = sqrt(2 * a_max * d)
        """
        if distance_to_target_m <= 0:
            return 0.0
        return math.sqrt(2.0 * self.max_accel * distance_to_target_m)

    def update(self, target_speed: float, dt: Optional[float] = None) -> float:
        """
        Met à jour la vitesse actuelle en direction de la consigne (target_speed)
        en appliquant la rampe bornée par self.max_accel.
        
        :param target_speed: Vitesse cible demandée (m/s)
        :param dt: Pas de temps écoulé (s). Si None, calculé automatiquement par horloge.
        :return: Vitesse filtrée admissible (m/s)
        """
        now = time.perf_counter()
        if dt is None:
            if self.last_update_time is None:
                dt = 0.05  # Pas par défaut (20 Hz)
            else:
                dt = max(1e-4, now - self.last_update_time)
        self.last_update_time = now

        # Plafonnement absolu de la consigne par la vitesse de croisière
        target_clamped = max(-self.max_speed, min(self.max_speed, target_speed))

        # Variation de vitesse maximale permise pendant le pas dt
        max_delta_v = self.max_accel * dt

        delta_v = target_clamped - self.current_speed

        if delta_v > max_delta_v:
            # Accélération limitée
            self.current_speed += max_delta_v
        elif delta_v < -max_delta_v:
            # Décélération limitée
            self.current_speed -= max_delta_v
        else:
            # Atteinte directe de la consigne sans saturation
            self.current_speed = target_clamped

        return self.current_speed
