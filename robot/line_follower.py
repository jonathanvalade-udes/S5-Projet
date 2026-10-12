#!/usr/bin/env python3
"""Lit les 5 capteurs avec la librairie SunFounder_Line_Follower."""

import sys
import time


def open_sensor():
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
