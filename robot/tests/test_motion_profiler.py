"""
test_motion_profiler.py
Tests unitaires rigoureux validant le profilage cinématique et la limitation d'accélération.
Vérifie qu'aucun pas temporel ne viole le seuil critique a <= 0.735 m/s².
"""

import math
import unittest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from motion_profiler import MotionProfiler


class TestMotionProfiler(unittest.TestCase):
    def setUp(self):
        self.profiler = MotionProfiler(max_accel=0.735, max_speed=0.28)

    def test_acceleration_ramp_bound(self):
        """Vérifie que l'accélération ne dépasse jamais 0.735 m/s² lors d'un échelon brutal."""
        dt = 0.05  # 50 ms (20 Hz)
        target = 0.28  # m/s
        
        current_speed = 0.0
        self.profiler.reset(0.0)

        steps = int(math.ceil(target / (0.735 * dt))) + 5
        for step in range(steps):
            prev_speed = self.profiler.current_speed
            new_speed = self.profiler.update(target, dt=dt)
            accel = (new_speed - prev_speed) / dt

            # Tolérance d'arrondi numérique float
            self.assertLessEqual(
                accel, 
                0.735 + 1e-6, 
                f"Accélération violée au pas {step}: {accel:.6f} m/s² > 0.735 m/s²"
            )
            self.assertGreaterEqual(new_speed, 0.0)
            self.assertLessEqual(new_speed, target)

        self.assertAlmostEqual(self.profiler.current_speed, target, places=4)

    def test_emergency_braking_ramp_bound(self):
        """Vérifie que le freinage brutal ne dépasse jamais la décélération maximale autorisée."""
        dt = 0.05
        self.profiler.reset(0.28)
        target = 0.0

        for step in range(20):
            prev_speed = self.profiler.current_speed
            new_speed = self.profiler.update(target, dt=dt)
            decel = (prev_speed - new_speed) / dt

            self.assertLessEqual(
                decel, 
                0.735 + 1e-6, 
                f"Décélération violée au pas {step}: {decel:.6f} m/s² > 0.735 m/s²"
            )
            self.assertGreaterEqual(new_speed, 0.0)

        self.assertAlmostEqual(self.profiler.current_speed, 0.0, places=4)

    def test_curve_speed_limit(self):
        """Vérifie le calcul de la vitesse maximale pour le rayon minimal du cahier des charges (R = 120 mm)."""
        r_min = 0.12  # m
        v_curve_max = self.profiler.calculate_curve_speed_limit(r_min)
        expected = math.sqrt(0.735 * 0.12)  # ~0.29698 m/s
        self.assertAlmostEqual(v_curve_max, expected, places=4)
        
        # Vérification de l'accélération centrifuge résultante : a_c = v² / R == a_max
        a_centrifuge = (v_curve_max ** 2) / r_min
        self.assertAlmostEqual(a_centrifuge, 0.735, places=5)

    def test_stopping_distance_calculation(self):
        """Vérifie la distance physique théorique d'arrêt complet."""
        speed = 0.28  # m/s
        stop_dist = self.profiler.calculate_stopping_distance(speed)
        expected_dist = (0.28 ** 2) / (2.0 * 0.735)  # ~0.05333 m (5.33 cm)
        self.assertAlmostEqual(stop_dist, expected_dist, places=4)


if __name__ == "__main__":
    unittest.main()
