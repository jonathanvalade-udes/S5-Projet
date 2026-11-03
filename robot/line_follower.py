"""
line_follower.py
Contrôleur de suivi de ligne optique et régulateur de braquage pour FastDrink.
Conforme aux spécifications S2 (ruban 18 mm), S3 (courbes R >= 120 mm) et S10 (algorithme unifié).
"""

import sys
import time
from typing import List, Tuple, Optional


class LineFollowerController:
    """
    Régulateur Proportionnel-Dérivé (PD) calculant l'angle de direction et la vitesse adaptée
    à partir des 5 capteurs optiques infrarouges [G2, G1, Centre, D1, D2].
    """

    # Poids spatiaux des capteurs : négatif = gauche, positif = droite
    SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

    # Paramètres d'asservissement PD de direction
    DEFAULT_KP: float = 0.65
    DEFAULT_KD: float = 0.20

    # Vitesses de référence (m/s)
    CRUISE_SPEED: float = 0.22  # Vitesse nominale en ligne droite
    MIN_CURVE_SPEED: float = 0.10  # Vitesse sécuritaire en virage serré (R = 120 mm)
    SPEED_REDUCTION_GAIN: float = 0.08  # Ralentissement proportionnel à la courbure

    # Temps maximal de maintien de la dernière direction en cas de perte de ligne (secondes)
    LINE_LOSS_TIMEOUT: float = 1.20

    def __init__(self, kp: float = DEFAULT_KP, kd: float = DEFAULT_KD):
        self.kp = kp
        self.kd = kd
        
        self.last_error: float = 0.0
        self.last_time: Optional[float] = None
        self.last_steer: float = 0.0
        
        self.line_lost_time: Optional[float] = None
        self.last_known_direction: float = 0.0

    def reset(self):
        """Réinitialise l'historique du régulateur."""
        self.last_error = 0.0
        self.last_time = None
        self.last_steer = 0.0
        self.line_lost_time = None
        self.last_known_direction = 0.0

    def compute_barycentric_error(self, sensor_array: List[int]) -> Optional[float]:
        """
        Calcule l'erreur spatiale barycentrique de centrage sur le ruban noir.
        :param sensor_array: Liste de 5 bits [G2, G1, Centre, D1, D2] (1 = ruban noir détecté).
        :return: Erreur dans [-2.0, +2.0], ou None si aucun capteur ne détecte la ligne.
        """
        if not sensor_array or len(sensor_array) < 5:
            return None

        total_weight = 0.0
        active_count = 0

        for i in range(5):
            if sensor_array[i] == 1:
                total_weight += self.SENSOR_WEIGHTS[i]
                active_count += 1

        if active_count == 0:
            return None

        return total_weight / float(active_count)

    def update(self, sensor_array: List[int], dt: Optional[float] = None) -> Tuple[float, float]:
        """
        Calcule la consigne de braquage normalisée et la consigne de vitesse cible.
        
        :param sensor_array: État des 5 capteurs optiques [G2, G1, C, D1, D2]
        :param dt: Pas de temps écoulé (s)
        :return: Tuple (steer_angle dans [-1.0, 1.0], target_velocity dans [MIN_SPEED, CRUISE_SPEED])
        """
        now = time.perf_counter()
        if dt is None:
            if self.last_time is None:
                dt = 0.05
            else:
                dt = max(1e-4, now - self.last_time)
        self.last_time = now

        error = self.compute_barycentric_error(sensor_array)

        # Cas 0 : Détection de la barre d'arrêt terminale en T (au moins 4 capteurs actifs)
        active_count = sum(1 for s in sensor_array if s == 1)
        if active_count >= 4:
            # Ligne d'arrivée atteinte
            return 0.0, 0.0

        # Cas 1 : Ligne détectée par au moins un capteur
        if error is not None:
            self.line_lost_time = None

            # Calcul de la dérivée de l'erreur
            d_error = (error - self.last_error) / dt
            self.last_error = error

            # Correction Proportionnelle-Dérivée (PD)
            steer = (self.kp * error) + (self.kd * d_error)
            
            # Saturation de la direction dans [-1.0, 1.0]
            steer = max(-1.0, min(1.0, steer))
            self.last_steer = steer
            self.last_known_direction = 1.0 if steer > 0 else (-1.0 if steer < 0 else 0.0)

            # Modulation de la vitesse selon la sévérité du virage (anti-déversement S6)
            curve_severity = abs(error)
            speed_penalty = self.SPEED_REDUCTION_GAIN * (curve_severity ** 1.5)
            target_velocity = max(self.MIN_CURVE_SPEED, self.CRUISE_SPEED - speed_penalty)

            return steer, target_velocity

        # Cas 2 : Perte de ligne temporaire (ex: passage rapide ou discontinuité)
        if self.line_lost_time is None:
            self.line_lost_time = now

        elapsed_lost = now - self.line_lost_time

        if elapsed_lost <= self.LINE_LOSS_TIMEOUT:
            # Conserver le braquage précédent pour recouper la ligne
            steer = self.last_steer
            target_velocity = self.MIN_CURVE_SPEED
        else:
            # Perte prolongée : arrêt sécuritaire ou recherche lente
            steer = 0.0
            target_velocity = 0.0

        return steer, target_velocity


def open_sensor():
    """Charge la librairie SunFounder si exécuté sur le Raspberry Pi physique."""
    try:
        from SunFounder_Line_Follower import Line_Follower
    except ImportError:
        sys.path.append("/home/pi/SunFounder_PiCar-S/example")
        try:
            from SunFounder_Line_Follower import Line_Follower
        except ImportError as err:
            sys.exit(
                "Librairie SunFounder_Line_Follower introuvable.\n"
                "Elle est d'habitude dans /home/pi/SunFounder_PiCar-S/example.\n"
                f"Détail : {err}"
            )
    return Line_Follower.Line_Follower()


def main():
    """Lecture des capteurs matériels en local sur le PiCar."""
    sensor = open_sensor()
    # Le défaut SunFounder est 300. Nos lectures restent sous 140, donc tout serait 1.
    # 1 = en dessous du seuil (noir), 0 = au-dessus (blanc).
    sensor.references = [50, 50, 50, 50, 50]
    print("Lecture SunFounder. 1 = noir, 0 = blanc. Ctrl+C pour arrêter.")
    try:
        while True:
            analog = sensor.read_analog()
            digital = sensor.read_digital()
            print(f"{digital}   analog {analog}", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\narrêt")


if __name__ == "__main__":
    main()
