#!/usr/bin/env python3
"""
Suivi de ligne sur le Raspberry Pi.

smbus lit les 5 capteurs. picar commande le servo et les roues arrière.
LineFollowerController décide du braquage et de la vitesse.
"""

import time
import picar
import smbus
from picar import back_wheels, front_wheels
from line_follower import LineFollowerController

I2C_BUS = 1
I2C_ADDRESS = 0x11
STRAIGHT_ANGLE = 90
TURNING_MAX = 40
MAX_PWM = 58
MIN_PWM = 48
MAX_ANGLE_STEP = 8


def read_analog(bus):
    """Cinq valeurs analogiques, gauche vers droite. Le module envoie la droite en premier."""
    raw = bus.read_i2c_block_data(I2C_ADDRESS, 0, 10)
    values = [(raw[index * 2] << 8) + raw[index * 2 + 1] for index in range(5)]
    return list(reversed(values))


def to_bits(analog):
    """1 = ruban noir. None si la trame I2C est invalide."""
    if any(value > 1000 for value in analog):
        return None

    brightest = max(analog)
    darkest = min(analog)
    if brightest < 45:
        return [1, 1, 1, 1, 1]

    contrast = brightest - darkest
    if contrast < 15:
        return [0, 0, 0, 0, 0]

    cutoff = darkest + contrast * 0.28
    return [1 if value <= cutoff else 0 for value in analog]


def steer_to_angle(steer):
    """Un petit écart reste presque droit. Seul un gros écart prend l'angle max."""
    softened = steer * abs(steer)
    angle = int(round(STRAIGHT_ANGLE - softened * TURNING_MAX))
    low = STRAIGHT_ANGLE - TURNING_MAX
    high = STRAIGHT_ANGLE + TURNING_MAX
    return max(low, min(high, angle))


def speed_to_pwm(speed_m_s):
    if speed_m_s <= 0:
        return 0
    ratio = min(1.0, speed_m_s / LineFollowerController.CRUISE_SPEED)
    return int(round(MIN_PWM + ratio * (MAX_PWM - MIN_PWM)))


def main():
    bus = smbus.SMBus(I2C_BUS)
    picar.setup()

    front = front_wheels.Front_Wheels()
    rear = back_wheels.Back_Wheels()
    front.turning_max = TURNING_MAX
    front.ready()
    rear.ready()

    # Kp bas et Kd nul : 0.65 / 0.20 braquent trop fort et font zigzager en ligne droite.
    controller = LineFollowerController(kp=0.50, kd=0.0)
    applied_angle = STRAIGHT_ANGLE
    period_s = 0.05
    previous = time.perf_counter()

    print("Suivi de ligne. Le robot va avancer.")
    print("Pose-le sur le ruban. Ctrl+C pour arrêter.")
    for second in range(3, 0, -1):
        print(second, flush=True)
        time.sleep(1)

    try:
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - previous
            previous = loop_start

            try:
                analog = read_analog(bus)
            except OSError:
                time.sleep(period_s)
                continue

            sensors = to_bits(analog)
            if sensors is None:
                time.sleep(period_s)
                continue

            steer, speed = controller.update(sensors, dt=dt)
            target_angle = steer_to_angle(steer)
            step = max(-MAX_ANGLE_STEP, min(MAX_ANGLE_STEP, target_angle - applied_angle))
            applied_angle += step
            pwm = speed_to_pwm(speed)

            front.turn(applied_angle)
            if pwm == 0:
                rear.stop()
            else:
                rear.speed = pwm
                rear.forward()

            view = " ".join("#" if bit else "." for bit in sensors)
            print(
                f"{view}   bits {sensors}   analog {analog}   "
                f"braquage {steer:+.2f}   angle {applied_angle}°   pwm {pwm}",
                flush=True,
            )

            remaining = period_s - (time.perf_counter() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\narrêt")
    finally:
        rear.stop()
        front.turn_straight()


if __name__ == "__main__":
    main()
