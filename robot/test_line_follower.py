"""
test_line_follower.py
Banc d'essai unitaire automatisé pour le contrôleur de suivi de ligne optique (Étape 3).
Vérifie le calcul d'erreur barycentrique, l'asservissement PD, le ralentissement en virage et l'anti-décrochage.
"""

import time
import unittest
from line_follower import LineFollowerController


class TestLineFollowerController(unittest.TestCase):
    def setUp(self):
        self.controller = LineFollowerController()

    def test_centered_line(self):
        """Vérifie le comportement en ligne droite nominale (capteur central actif)."""
        steer, speed = self.controller.update([0, 0, 1, 0, 0], dt=0.05)
        self.assertAlmostEqual(steer, 0.0, places=2, msg="Le braquage doit être nul au centre")
        self.assertAlmostEqual(speed, self.controller.CRUISE_SPEED, places=2, msg="La vitesse doit être maximale en ligne droite")

    def test_moderate_right_turn(self):
        """Vérifie la correction proportionnelle vers la droite (capteur D1 actif)."""
        steer, speed = self.controller.update([0, 0, 0, 1, 0], dt=0.05)
        self.assertGreater(steer, 0.2, msg="Le braquage doit être positif (vers la droite)")
        self.assertLess(speed, self.controller.CRUISE_SPEED, msg="La vitesse doit ralentir en amorce de virage")

    def test_moderate_left_turn(self):
        """Vérifie la correction proportionnelle vers la gauche (capteur G1 actif)."""
        steer, speed = self.controller.update([0, 1, 0, 0, 0], dt=0.05)
        self.assertLess(steer, -0.2, msg="Le braquage doit être négatif (vers la gauche)")
        self.assertLess(speed, self.controller.CRUISE_SPEED, msg="La vitesse doit ralentir en amorce de virage")

    def test_sharp_turn_saturation(self):
        """Vérifie le virage serré (R = 120 mm) avec capteur extrême D2 actif."""
        steer, speed = self.controller.update([0, 0, 0, 0, 1], dt=0.05)
        self.assertGreaterEqual(steer, 0.8, msg="Le braquage doit être proche du maximum (+1.0)")
        self.assertLessEqual(speed, 0.18, msg="La vitesse doit être bridée sous le seuil critique centrifuge (0.297 m/s)")
        self.assertGreaterEqual(speed, self.controller.MIN_CURVE_SPEED)

    def test_line_loss_memory(self):
        """Vérifie le maintien de trajectoire lors d'une perte temporaire de ligne."""
        # 1. Amorcer un virage à droite
        self.controller.update([0, 0, 0, 0, 1], dt=0.05)
        last_steer = self.controller.last_steer

        # 2. Perte immédiate de ligne
        steer_lost, speed_lost = self.controller.update([0, 0, 0, 0, 0], dt=0.05)
        self.assertAlmostEqual(steer_lost, last_steer, places=2, msg="Le braquage doit être maintenu")
        self.assertEqual(speed_lost, self.controller.MIN_CURVE_SPEED)

    def test_prolonged_line_loss_stop(self):
        """Vérifie l'arrêt de sécurité si la ligne reste perdue plus longtemps que le timeout."""
        # 1. Enregistrer le début de perte de ligne
        self.controller.update([0, 0, 0, 0, 0], dt=0.05)
        
        # 2. Attendre que le timeout soit dépassé (0.4 s > 0.35 s)
        time.sleep(0.40)
        
        # 3. La seconde mise à jour doit déclencher l'arrêt sécuritaire
        steer_stop, speed_stop = self.controller.update([0, 0, 0, 0, 0], dt=0.05)
        self.assertEqual(speed_stop, 0.0, msg="Le véhicule doit s'arrêter si la ligne est perdue durablement")


if __name__ == "__main__":
    unittest.main()
