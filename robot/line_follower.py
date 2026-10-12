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
    print("Lecture SunFounder. Ctrl+C pour arrêter.")
    try:
        while True:
            print(sensor.read_analog(), flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\narrêt")


if __name__ == "__main__":
    main()
