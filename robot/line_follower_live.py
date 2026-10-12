#!/usr/bin/env python3
"""
Test temps réel du composant suiveur de ligne, sur le Raspberry Pi.

Lit les 5 capteurs infrarouges sur l'I2C (adresse 0x11), passe la lecture
à LineFollowerController, et affiche le braquage et la vitesse.
Les moteurs ne sont pas commandés.
"""

import sys
import time

from line_follower import LineFollowerController

# Seuil de l'exemple SunFounder : sous ce niveau analogique, le ruban noir est vu.
REFERENCES = [200, 200, 200, 200, 200]
I2C_ADDRESS = 0x11


class LineSensor:
    """Lecture du module 5 canaux, même protocole que SunFounder_Line_Follower."""

    def __init__(self, address=I2C_ADDRESS, references=None):
        try:
            import smbus
        except ImportError:
            try:
                import smbus2 as smbus
            except ImportError:
                sys.exit(
                    "Module smbus introuvable.\n"
                    "Sur le Pi : sudo apt install python3-smbus"
                )
        try:
            self.bus = smbus.SMBus(1)
        except FileNotFoundError:
            sys.exit("I2C absent. Active-le : sudo raspi-config → Interface Options → I2C.")
        except PermissionError:
            sys.exit("Accès /dev/i2c-1 refusé. Ajoute l'utilisateur au groupe i2c, puis reconnecte-toi.")
        self.address = address
        self.references = list(references or REFERENCES)

    def read_analog(self):
        last_error = None
        for _ in range(5):
            try:
                raw = self.bus.read_i2c_block_data(self.address, 0, 10)
            except OSError as err:
                last_error = err
                continue
            values = []
            for index in range(5):
                values.append((raw[index * 2] << 8) + raw[index * 2 + 1])
            # L'exemple SunFounder tourne à droite quand l'index 0 voit la ligne :
            # l'index 0 est donc le capteur de droite. Le contrôleur attend la gauche en premier.
            return list(reversed(values))
        raise OSError(f"Lecture I2C impossible à l'adresse {self.address:#x} : {last_error}")

    def read(self):
        analog = self.read_analog()
        digital = []
        for value, reference in zip(analog, self.references):
            digital.append(1 if value < reference else 0)
        return analog, digital


def main() -> None:
    sensor = LineSensor()
    controller = LineFollowerController()
    period_s = 0.1

    print("Test temps réel du suiveur de ligne. Moteurs arrêtés.")
    print("Capteurs : G2 G1 C D1 D2    # = ruban vu (analogique < 200)")
    print("Passe le module au-dessus du ruban. Ctrl+C pour arrêter.\n")

    previous = time.perf_counter()
    try:
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - previous
            previous = loop_start

            analog, sensors = sensor.read()
            steer, speed = controller.update(sensors, dt=dt)
            view = " ".join("#" if bit else "." for bit in sensors)
            print(
                f"{view}   bits {sensors}   analog {analog}   "
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
