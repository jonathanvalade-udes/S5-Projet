#!/usr/bin/env python3
"""
Suivi de ligne temps réel sur le Raspberry Pi.

Lit les 5 capteurs I2C, calcule le braquage avec LineFollowerController,
puis commande le servo de direction et les roues arrière via la librairie picar.
"""

import sys
import time

from line_follower import LineFollowerController

I2C_ADDRESS = 0x11
# 90° = droit. En dessous on tourne à droite, au-dessus à gauche.
STRAIGHT_ANGLE = 90
TURNING_MAX = 35
# Vitesse picar de 0 à 100. 30 reste lent pour un premier essai.
MAX_PWM = 30


class LineSensor:
    """Même lecture I2C que SunFounder_Line_Follower, adresse 0x11."""

    def __init__(self, address=I2C_ADDRESS):
        try:
            import smbus
        except ImportError:
            try:
                import smbus2 as smbus
            except ImportError:
                sys.exit("Module smbus introuvable. Sur le Pi : sudo apt install python3-smbus")
        try:
            self.bus = smbus.SMBus(1)
        except FileNotFoundError:
            sys.exit("I2C absent. Active-le : sudo raspi-config → Interface Options → I2C.")
        except PermissionError:
            sys.exit("Accès /dev/i2c-1 refusé. Ajoute l'utilisateur au groupe i2c, puis reconnecte-toi.")
        self.address = address

    def read_analog(self):
        last_error = None
        for _ in range(5):
            try:
                raw = self.bus.read_i2c_block_data(self.address, 0, 10)
            except OSError as err:
                last_error = err
                continue
            values = [(raw[index * 2] << 8) + raw[index * 2 + 1] for index in range(5)]
            # Index 0 du module est le capteur de droite. Le contrôleur attend la gauche en premier.
            return list(reversed(values))
        raise OSError(f"Lecture I2C impossible à l'adresse {self.address:#x} : {last_error}")

    def read(self):
        """1 = ruban. Le noir reflète peu, donc sa valeur est bien plus basse que le blanc voisin."""
        analog = self.read_analog()
        brightest = max(analog)
        if brightest < 45:
            return analog, [1, 1, 1, 1, 1]
        limit = brightest * 0.60
        digital = [1 if value < limit else 0 for value in analog]
        return analog, digital


def open_wheels():
    try:
        import picar
        from picar import back_wheels, front_wheels
    except ImportError:
        sys.path.append("/home/pi/SunFounder_PiCar")
        try:
            import picar
            from picar import back_wheels, front_wheels
        except ImportError as err:
            sys.exit(
                "Librairie picar introuvable.\n"
                "Elle se trouve en général dans /home/pi/SunFounder_PiCar.\n"
                f"Détail : {err}"
            )
    picar.setup()
    front = front_wheels.Front_Wheels()
    rear = back_wheels.Back_Wheels()
    front.turning_max = TURNING_MAX
    front.turn(STRAIGHT_ANGLE)
    rear.speed = 0
    rear.stop()
    return front, rear


def steer_to_angle(steer):
    angle = int(round(STRAIGHT_ANGLE - steer * TURNING_MAX))
    return max(STRAIGHT_ANGLE - TURNING_MAX, min(STRAIGHT_ANGLE + TURNING_MAX, angle))


def speed_to_pwm(speed_m_s):
    if speed_m_s <= 0:
        return 0
    pwm = int(round(speed_m_s / LineFollowerController.CRUISE_SPEED * MAX_PWM))
    return max(18, min(MAX_PWM, pwm))


def main() -> None:
    sensor = LineSensor()
    controller = LineFollowerController()
    front, rear = open_wheels()
    period_s = 0.05

    print("Suivi de ligne. Le robot va avancer.")
    print("Pose-le sur le ruban. Ctrl+C pour arrêter.")
    for second in range(3, 0, -1):
        print(second, flush=True)
        time.sleep(1)

    previous = time.perf_counter()
    try:
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - previous
            previous = loop_start

            analog, sensors = sensor.read()
            steer, speed = controller.update(sensors, dt=dt)
            angle = steer_to_angle(steer)
            pwm = speed_to_pwm(speed)

            front.turn(angle)
            if pwm == 0:
                rear.speed = 0
                rear.stop()
            else:
                rear.speed = pwm
                rear.forward()

            view = " ".join("#" if bit else "." for bit in sensors)
            print(
                f"{view}   bits {sensors}   analog {analog}   "
                f"braquage {steer:+.2f}   angle {angle}°   pwm {pwm}",
                flush=True,
            )

            remaining = period_s - (time.perf_counter() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\narrêt")
    finally:
        rear.speed = 0
        rear.stop()
        front.turn(STRAIGHT_ANGLE)


if __name__ == "__main__":
    main()
