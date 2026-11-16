"""PicarProtocol: Pure Python logic for PiCar-S simulation protocol handler.

Encapsulates:
- Formatting telemetry frames for SimBridge (JSON ending with newline).
- Parsing motion commands from SimBridge.
- Normalizing IR sensor readings (raw distance sensor values -> analog [0.0, 1.0] and binary 0/1).
- Key event processing with edge-triggering for 'S' (start) and ' ' (emergency stop).
- Stream buffer splitting on newline delimiters.
"""

from __future__ import annotations

import json
import math
from typing import Any, Tuple, List, Set, Optional


class PicarProtocol:
    """Protocol serializer/deserializer and helper for PiCar-S <-> SimBridge communication."""

    CMD_TRIGGER_START: str = "trigger_start\n"
    CMD_EMERGENCY_STOP: str = "emergency_stop\n"

    DEFAULT_IR_MIN: float = 100.0   # White floor raw reflectance
    DEFAULT_IR_MAX: float = 950.0   # Black tape raw reflectance
    DEFAULT_IR_THRESHOLD: float = 0.5

    DEFAULT_HOST: str = "127.0.0.1"
    DEFAULT_PORT: int = 9000
    WHEEL_RADIUS: float = 0.0335    # meters (33.5 mm)
    MAX_STEER_ANGLE: float = 0.7854  # radians (45 degrees : meme plage que le HAL MotorActuator / FSM, direction = braquage / 45)
    WHEELBASE: float = 0.14          # m, essieu arriere -> pivots avant (= WHEELBASE_M du MotionProfiler / VehicleModel)
    TRACK: float = 0.13              # m, entre les centres des roues avant (pivots CAD a ±50 mm + 15 mm de deport)
    CONTROL_STEP_MS: int = 20        # boucle de controle et de telemetrie : 50 Hz, quel que soit le pas physique du monde

    @classmethod
    def ackermann_angles(cls, steer_angle: float) -> Tuple[float, float]:
        """Angles (rad) des roues avant (gauche, droite) pour un braquage equivalent bicyclette `steer_angle` (+ = gauche).

        Les deux roues visent le meme centre de rotation, sur l'axe de l'essieu arriere, a R = L / tan(steer) : la roue
        interieure braque plus que l'exterieure. C'est l'angle bicyclette que le FSM commande (R = L / tan(delta)).
        """
        t = math.tan(steer_angle)
        half = cls.TRACK / 2.0
        left = math.atan(cls.WHEELBASE * t / (cls.WHEELBASE - half * t))
        right = math.atan(cls.WHEELBASE * t / (cls.WHEELBASE + half * t))
        return left, right

    @classmethod
    def differential_speeds(cls, velocity: float, steer_angle: float) -> Tuple[float, float]:
        """Vitesses lineaires (m/s) des roues arriere (gauche, droite) : chacune roule sur son propre rayon, sans riper."""
        k = cls.TRACK / 2.0 * math.tan(steer_angle) / cls.WHEELBASE
        return velocity * (1.0 - k), velocity * (1.0 + k)

    @classmethod
    def format_trigger_start(cls) -> str:
        """Returns the start trigger command line."""
        return cls.CMD_TRIGGER_START

    @classmethod
    def format_emergency_stop(cls) -> str:
        """Returns the emergency stop command line."""
        return cls.CMD_EMERGENCY_STOP

    @classmethod
    def normalize_ir(
        cls,
        raw_val: float,
        min_val: float = DEFAULT_IR_MIN,
        max_val: float = DEFAULT_IR_MAX,
        threshold: float = DEFAULT_IR_THRESHOLD,
    ) -> Tuple[float, int]:
        """Normalizes a raw IR reflectance reading.

        Args:
            raw_val: Raw sensor value from Webots DistanceSensor.
            min_val: Reading on white surface (default 100.0).
            max_val: Reading on black tape (default 950.0).
            threshold: Binary threshold for darkness (default 0.5).

        Returns:
            Tuple of (analog_darkness [0.0, 1.0], binary_val {0, 1}).
        """
        if max_val <= min_val:
            analog = 0.0
        else:
            clamped_raw = max(min_val, min(max_val, float(raw_val)))
            analog = (clamped_raw - min_val) / (max_val - min_val)

        analog = max(0.0, min(1.0, analog))
        binary = 1 if analog > threshold else 0
        return round(analog, 4), binary

    @classmethod
    def format_telemetry(
        cls,
        distance_m: float,
        raw_ir_values: List[float],
        velocity: float = 0.0,
        ball_dropped: bool = False,
    ) -> str:
        """Formats telemetry data into a JSON string ending with a newline.

        Args:
            distance_m: Sonar distance in meters (will be converted to centimeters).
            raw_ir_values: List of raw IR sensor readings (typically 5 sensors).
            velocity: Current linear vehicle velocity in m/s.
            ball_dropped: Flag indicating ball drop state.

        Returns:
            JSON formatted telemetry line terminated with '\\n'.
        """
        line_follower: List[int] = []
        line_analog: List[float] = []

        for raw_val in raw_ir_values:
            analog, binary = cls.normalize_ir(raw_val)
            line_analog.append(analog)
            line_follower.append(binary)

        # Distance must be in centimeters for SimBridge / FSM
        distance_cm = round(float(distance_m) * 100.0, 2)

        data = {
            "distance": distance_cm,
            "velocity": round(float(velocity), 4),
            "line_follower": line_follower,
            "line_analog": line_analog,
            "ball_dropped": bool(ball_dropped),
        }
        return json.dumps(data) + "\n"

    @classmethod
    def parse_command(cls, json_line: str) -> Optional[Tuple[float, float, bool]]:
        """Parses a command JSON string from SimBridge.

        Args:
            json_line: Single line JSON string received from SimBridge.

        Returns:
            Tuple of (velocity, direction, brake):
            - velocity: float in m/s
            - direction: float clamped in [-1.0, 1.0]
            - brake: bool
            Returns None if parsing fails or if the message is a status/ACK frame without motion targets.
        """
        line = json_line.strip()
        if not line:
            return None

        try:
            data: Any = json.loads(line)
            if not isinstance(data, dict):
                return None

            # Ignore status/ACK frames without velocity/direction
            if "velocity" not in data and "direction" not in data:
                return None

            raw_vel = data.get("velocity", 0.0)
            raw_dir = data.get("direction", 0.0)
            raw_brake = data.get("brake", False)

            velocity = float(raw_vel)
            direction = float(raw_dir)
            direction = max(-1.0, min(1.0, direction))
            brake = bool(raw_brake)

            return round(velocity, 4), round(direction, 4), brake
        except (json.JSONDecodeError, ValueError, TypeError):
            return None

    @classmethod
    def process_key_events(
        cls,
        current_keys: Set[int],
        previous_keys: Set[int],
    ) -> Tuple[Optional[str], Set[int]]:
        """Detects key-press transitions (edge-triggered) for keyboard interaction.

        Supported keys:
        - ' ' (Space / 32) -> emergency_stop (Safety stop highest priority)
        - 'S' / 's' -> trigger_start

        Args:
            current_keys: Set of key codes currently held down.
            previous_keys: Set of key codes held down in previous step.

        Returns:
            Tuple of (command_string_or_None, updated_current_keys).
        """
        new_pressed = current_keys - previous_keys

        # Check Space (32) first for safety stop priority
        if ord(' ') in new_pressed or 32 in new_pressed:
            return cls.CMD_EMERGENCY_STOP, set(current_keys)

        # Check 'S' / 's'
        if ord('S') in new_pressed or ord('s') in new_pressed:
            return cls.CMD_TRIGGER_START, set(current_keys)

        return None, set(current_keys)

    @classmethod
    def extract_lines(cls, buffer: str) -> Tuple[List[str], str]:
        """Extracts complete lines from an incoming string buffer.

        Args:
            buffer: Current stream buffer content.

        Returns:
            Tuple of (list_of_complete_lines, remaining_partial_buffer).
        """
        if "\n" not in buffer:
            return [], buffer

        parts = buffer.split("\n")
        complete_lines = parts[:-1]
        remaining = parts[-1]
        return complete_lines, remaining
