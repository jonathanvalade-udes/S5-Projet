"""Tests unitaires des filtres du capteur à ultrasons."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ultrasonic_filters import LowPassIIR, SlidingMedian, make_filter  # noqa: E402


class TestLowPassIIR(unittest.TestCase):
    def test_equation(self):
        f = LowPassIIR(alpha=0.5)
        self.assertEqual(f.update(100.0), 100.0)          # y[0] = x[0]
        self.assertAlmostEqual(f.update(200.0), 150.0)    # 0.5*200 + 0.5*100
        self.assertAlmostEqual(f.update(200.0), 175.0)    # 0.5*200 + 0.5*150

    def test_converges_to_constant_input(self):
        f = LowPassIIR(alpha=0.3)
        f.update(0.001)
        y = None
        for _ in range(60):
            y = f.update(500.0)
        self.assertAlmostEqual(y, 500.0, places=3)

    def test_invalid_alpha(self):
        for bad in (0.0, -0.1, 1.5):
            with self.assertRaises(ValueError):
                LowPassIIR(alpha=bad)

    def test_ignores_invalid_samples(self):
        f = LowPassIIR(alpha=0.5)
        f.update(100.0)
        self.assertEqual(f.update(-1), 100.0)             # -1 = erreur de lecture du capteur
        self.assertEqual(f.update(float("nan")), 100.0)
        self.assertEqual(f.update(None), 100.0)


class TestSlidingMedian(unittest.TestCase):
    def test_equation(self):
        f = SlidingMedian(size=3)
        out = f.filter([10, 50, 20, 30, 40])
        # fenêtres pleines : {10,50,20}->20 ; {50,20,30}->30 ; {20,30,40}->30
        self.assertEqual(out[2:], [20, 30, 30])

    def test_rejects_isolated_spike(self):
        f = SlidingMedian(size=5)
        signal = [300.0] * 10
        signal[6] = 2500.0                                # valeur aberrante isolée
        out = f.filter(signal)
        self.assertTrue(all(abs(y - 300.0) < 1e-9 for y in out))

    def test_size_must_be_odd(self):
        for bad in (0, 2, 4):
            with self.assertRaises(ValueError):
                SlidingMedian(size=bad)

    def test_ignores_invalid_samples(self):
        f = SlidingMedian(size=3)
        f.update(100.0)
        self.assertEqual(f.update(-1), 100.0)

    def test_reset(self):
        f = SlidingMedian(size=3)
        f.filter([100, 100, 100])
        f.reset()
        self.assertEqual(f.update(900.0), 900.0)          # plus d'historique


class TestFactory(unittest.TestCase):
    def test_make_filter(self):
        self.assertIsInstance(make_filter("median", size=7), SlidingMedian)
        self.assertIsInstance(make_filter("IIR", alpha=0.2), LowPassIIR)
        self.assertEqual(make_filter("none").update(42.0), 42.0)
        with self.assertRaises(ValueError):
            make_filter("kalman")


if __name__ == "__main__":
    unittest.main()
