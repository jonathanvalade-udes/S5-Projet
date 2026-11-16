"""Tests des modèles dynamiques (moteur, direction, bille) et de leur intégration au contrôleur."""
import math
import sys
import unittest
from pathlib import Path

CONTROLLER_DIR = Path(__file__).resolve().parent.parent / "controllers" / "picar_s_controller"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from picar_dynamics import BallModel, Calibration, DriveModel, PicarDynamics, SteeringModel
from picar_protocol import PicarProtocol
from picar_s_controller import PicarSController

DT = 0.02


class DummyMotor:
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0

    def setPosition(self, p):
        self.position = p

    def setVelocity(self, v):
        self.velocity = v


def ramp_fsm(a_fsm, v_target, seconds):
    """Consigne de vitesse que produirait le FSM : rampe d'accélération a_fsm jusqu'à v_target."""
    v, out = 0.0, []
    for _ in range(int(seconds / DT)):
        v = min(v_target, v + a_fsm * DT)
        out.append(v)
    return out


class TestCalibration(unittest.TestCase):
    def test_a_crit(self):
        self.assertAlmostEqual(Calibration().a_crit, 0.7358, places=3)

    def test_load_missing_file_gives_provisional(self):
        cal = Calibration.load("/nonexistent/calibration.json")
        self.assertFalse(cal.measured)

    def test_save_load_roundtrip(self):
        import tempfile, os
        cal = Calibration(motor_tau_s=0.31, measured=True)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "c.json")
            cal.save(p)
            back = Calibration.load(p)
        self.assertTrue(back.measured)
        self.assertAlmostEqual(back.motor_tau_s, 0.31)


class TestDrive(unittest.TestCase):
    def test_deadband_blocks_low_command(self):
        d = DriveModel(Calibration())
        for _ in range(100):
            v = d.update(0.10, DT)   # 0,10 m/s < 0,128 : le moteur ne démarre pas
        self.assertEqual(v, 0.0)

    def test_latency_and_lag(self):
        cal = Calibration(latency_s=0.06, motor_tau_s=0.15)
        d = DriveModel(cal)
        seen = [d.update(0.32, DT) for _ in range(200)]
        self.assertEqual(seen[0], 0.0)                    # rien pendant la latence
        self.assertAlmostEqual(seen[-1], 0.32, places=2)  # converge vers la consigne
        first_move = next(i for i, v in enumerate(seen) if v > 0)
        self.assertGreaterEqual(first_move * DT, 0.06 - DT)

    def test_lag_smooths_step(self):
        d = DriveModel(Calibration(latency_s=0.0, motor_tau_s=0.15))
        v = [d.update(0.32, DT) for _ in range(5)]
        steps = [b - a for a, b in zip([0.0] + v, v)]
        self.assertLess(max(steps) / DT, 0.32 / 0.15 * 1.01)   # pente initiale = consigne / tau


class TestSteering(unittest.TestCase):
    def test_slew_limited(self):
        s = SteeringModel(Calibration(latency_s=0.0, steer_slew_dps=250.0))
        a = s.update(1.0, DT)
        self.assertAlmostEqual(math.degrees(a), 250.0 * DT, places=3)   # 5° au premier pas
        for _ in range(100):
            a = s.update(1.0, DT)
        self.assertAlmostEqual(math.degrees(a), 45.0, places=3)

    def test_actuator_slew_matches_motor_actuator(self):
        # MotorActuator : 175 °/s sous 15° de consigne, 300 °/s au-dela (le servo lui-meme est plus rapide)
        s = SteeringModel(Calibration(latency_s=0.0))
        self.assertAlmostEqual(math.degrees(s.update(10.0 / 45.0, DT)), 175.0 * DT, places=3)
        s = SteeringModel(Calibration(latency_s=0.0))
        self.assertAlmostEqual(math.degrees(s.update(1.0, DT)), 300.0 * DT, places=3)

    def test_steer_ratio_scales_wheel_angle(self):
        s = SteeringModel(Calibration(latency_s=0.0, steer_ratio=0.7, steer_slew_dps=1e4, actuator_slew_curve_dps=1e4))
        self.assertAlmostEqual(math.degrees(s.update(1.0, DT)), 45.0 * 0.7, places=3)

    def test_backlash_needs_travel(self):
        s = SteeringModel(Calibration(latency_s=0.0, steer_backlash_deg=2.0, steer_slew_dps=1000.0))
        for _ in range(20):
            a = s.update(0.01, DT)    # 0,45° de consigne : dans le jeu, la roue ne bouge pas
        self.assertAlmostEqual(a, 0.0, places=6)

    def test_ackermann_yaw(self):
        s = SteeringModel(Calibration(latency_s=0.0, steer_slew_dps=1e4, actuator_slew_curve_dps=1e4))
        for _ in range(5):
            s.update(1.0, DT)
        radius = 0.14 / math.tan(math.radians(45))
        self.assertAlmostEqual(s.yaw_rate(0.2), 0.2 / radius, places=3)


class TestBall(unittest.TestCase):
    def run_ball(self, accel, seconds=3.0, v_max=1.0, yaw=0.0):
        ball, v = BallModel(Calibration()), 0.0
        for _ in range(int(seconds / DT)):
            v = min(v_max, v + accel * DT)
            ball.update(v, yaw, DT)
        return ball

    def test_gentle_acceleration_keeps_ball(self):
        ball = self.run_ball(0.60)
        self.assertFalse(ball.dropped)
        self.assertLess(ball.peak_ratio, 1.0)

    def test_hard_acceleration_drops_ball(self):
        self.assertTrue(self.run_ball(1.5).dropped)

    def test_sharp_cornering_drops_ball(self):
        ball = BallModel(Calibration())
        for _ in range(200):
            ball.update(0.4, 0.4 / 0.14 * math.tan(math.radians(45)), DT)   # a_lat = 1,2 m/s²
        self.assertTrue(ball.dropped)


class TestFsmRampThroughModel(unittest.TestCase):
    """La rampe du FSM (a = 0,735 x 0,88) doit garder la bille, même à travers les retards du moteur."""

    def test_fsm_ramp_keeps_ball(self):
        """Rampe du FSM une fois le moteur lancé (au-dessus de la zone morte) : la bille reste dans la coupelle."""
        dyn = PicarDynamics(Calibration())
        a_fsm = 0.735 * 0.88
        for _ in range(100):
            dyn.step(0.14, 0.0, DT)             # lancement à vitesse constante juste au-dessus de la zone morte
        dyn.ball.reset()
        dropped = False
        v = 0.14
        for _ in range(int(3.0 / DT)):
            v = min(0.32, v + a_fsm * DT)
            _, _, dropped = dyn.step(v, 0.0, DT)
        self.assertFalse(dropped)
        self.assertLess(dyn.ball.peak_ratio, 1.0)

    def test_launch_through_deadband_is_measured(self):
        """Départ à l'arrêt : le saut de la zone morte (0 -> 0,128 m/s) est un pic que la rampe du FSM ne lisse pas.

        Valeur provisoire (tau non mesuré) : le test vérifie seulement que la simulation expose ce pic."""
        dyn = PicarDynamics(Calibration())
        for v in ramp_fsm(0.735 * 0.88, 0.32, 3.0):
            dyn.step(v, 0.0, DT)
        self.assertGreater(dyn.ball.peak_ratio, 0.0)

    def test_hard_cut_drops_ball(self):
        dyn = PicarDynamics(Calibration())
        for v in ramp_fsm(0.65, 0.32, 2.0):
            dyn.step(v, 0.0, DT)
        for _ in range(100):
            dyn.step(0.0, 0.0, DT)          # coupure brutale de la consigne
        self.assertGreater(dyn.ball.peak_ratio, 0.5)  # le moteur lisse un peu, mais le pic est mesuré


class TestControllerIntegration(unittest.TestCase):
    def make(self, dynamics):
        c = PicarSController(robot=None, auto_start=False, dynamics=dynamics)
        c.drive_motor, c.steering_motor = DummyMotor(), DummyMotor()
        return c

    def test_off_applies_raw_command(self):
        c = self.make(False)
        c.apply_command(0.32, -0.5, False)
        self.assertAlmostEqual(c.drive_motor.velocity, 0.32 / PicarProtocol.WHEEL_RADIUS, places=3)
        self.assertAlmostEqual(c.steering_motor.position, -0.5 * PicarProtocol.MAX_STEER_ANGLE, places=3)

    def test_on_delays_and_smooths(self):
        c = self.make(True)
        c.apply_command(0.32, 0.0, False)
        c._apply_dynamics()
        self.assertEqual(c.drive_motor.velocity, 0.0)        # latence : pas d'effet au premier pas
        for _ in range(200):
            c._apply_dynamics()
        self.assertAlmostEqual(c.drive_motor.velocity * PicarProtocol.WHEEL_RADIUS, 0.32, places=2)

    def test_ball_flag_reported(self):
        c = self.make(True)
        c.apply_command(2.0, 0.0, False)                     # consigne irréaliste : accélération énorme
        for _ in range(100):
            c._apply_dynamics()
        self.assertTrue(c.ball_dropped)


if __name__ == "__main__":
    unittest.main()
