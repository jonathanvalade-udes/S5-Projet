#!/usr/bin/env python3
"""
Test temps réel du composant suiveur de ligne, sur le Raspberry Pi.

Lit les 5 capteurs infrarouges, passe la lecture à LineFollowerController,
et affiche le braquage et la vitesse. Les moteurs ne sont pas commandés.
"""

import sys
import time

from line_follower import LineFollowerController


def open_sensor():
    """Ouvre le module SunFounder à 5 canaux déjà installé sur le Pi."""
    try:
        from SunFounder_Line_Follower import Line_Follower
    except ImportError as err:
        sys.exit(
            "Module SunFounder_Line_Follower introuvable.\n"
            "Lance ce script sur le Pi avec le Python des exemples SunFounder.\n"
            f"Détail : {err}"
        )
    return Line_Follower.Line_Follower()


def read_sensors(sensor) -> list:
    raw = sensor.read_digital()
    values = [1 if int(bit) else 0 for bit in list(raw)[:5]]
    if len(values) != 5:
        raise RuntimeError(f"lecture capteur inattendue : {raw}")
    return values


def main() -> None:
    sensor = open_sensor()
    controller = LineFollowerController()
    period_s = 0.05  # 20 Hz, même cadence que la boucle du robot

    print("Test temps réel du suiveur de ligne. Moteurs arrêtés.")
    print("Capteurs : G2 G1 C D1 D2    # = ruban vu")
    print("Passe le module au-dessus du ruban. Ctrl+C pour arrêter.\n")

    previous = time.perf_counter()
    try:
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - previous
            previous = loop_start

            sensors = read_sensors(sensor)
            steer, speed = controller.update(sensors, dt=dt)
            view = " ".join("#" if bit else "." for bit in sensors)
            print(
                f"{view}   bits {sensors}   "
                f"braquage {steer:+.2f}   vitesse {speed:.3f} m/s",
                flush=True,
            )

            remaining = period_s - (time.perf_counter() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\narrêt")


if __name__ == "__main__":
    main()
