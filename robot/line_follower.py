#!/usr/bin/env python3
"""Lit les 5 capteurs du suiveur de ligne et affiche les valeurs analogiques."""

import time

import smbus

I2C_BUS = 1
I2C_ADDRESS = 0x11


def read_analog(bus):
    """Cinq valeurs, gauche vers droite. Le module envoie la droite en premier."""
    raw = bus.read_i2c_block_data(I2C_ADDRESS, 0, 10)
    values = [(raw[index * 2] << 8) + raw[index * 2 + 1] for index in range(5)]
    return list(reversed(values))


def main():
    bus = smbus.SMBus(I2C_BUS)
    print("Lecture des capteurs. G2 G1 C D1 D2. Ctrl+C pour arrêter.")
    try:
        while True:
            print(read_analog(bus), flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\narrêt")


if __name__ == "__main__":
    main()
