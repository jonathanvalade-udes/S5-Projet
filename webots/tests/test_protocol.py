import json
import socket
import sys
import threading
import unittest
from pathlib import Path

CONTROLLER_DIR = Path(__file__).resolve().parent.parent / "controllers" / "picar_s_controller"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from picar_protocol import PicarProtocol
from picar_s_controller import PicarSController


class DummyMotor:
    """Mock Webots motor for testing controller actuation."""
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0

    def setPosition(self, pos):
        self.position = pos

    def setVelocity(self, vel):
        self.velocity = vel


class DummyDistanceSensor:
    """Mock Webots DistanceSensor."""
    def __init__(self, value=0.0):
        self.value = value

    def getValue(self):
        return self.value

    def enable(self, timestep):
        pass


class DummyKeyboard:
    """Mock Webots Keyboard."""
    def __init__(self, keys=None):
        self.keys = keys or []
        self._index = 0

    def getKey(self):
        if self._index < len(self.keys):
            k = self.keys[self._index]
            self._index += 1
            return k
        return -1

    def enable(self, timestep):
        pass


class DummyRobot:
    """Mock Webots Robot."""
    def __init__(self):
        self.devices = {}
        self.keyboard = DummyKeyboard()

    def getBasicTimeStep(self):
        return 20.0

    def getKeyboard(self):
        return self.keyboard

    def getDevice(self, name):
        return self.devices.get(name, None)

    def step(self, timestep):
        return 0


class TestPicarProtocol(unittest.TestCase):
    def test_normalize_ir_boundaries(self):
        # White ground: raw value around 100 -> darkness ~0.0, binary 0
        analog_w, binary_w = PicarProtocol.normalize_ir(100.0)
        self.assertAlmostEqual(analog_w, 0.0, places=3)
        self.assertEqual(binary_w, 0)

        # Black tape: raw value around 950 -> darkness ~1.0, binary 1
        analog_b, binary_b = PicarProtocol.normalize_ir(950.0)
        self.assertAlmostEqual(analog_b, 1.0, places=3)
        self.assertEqual(binary_b, 1)

    def test_normalize_ir_intermediate_and_clamping(self):
        # Midpoint: (100 + 950) / 2 = 525 -> darkness 0.5
        analog_mid, binary_mid = PicarProtocol.normalize_ir(525.0)
        self.assertAlmostEqual(analog_mid, 0.5, places=3)
        self.assertEqual(binary_mid, 0)  # threshold is > 0.5

        # Slightly above threshold
        analog_hi, binary_hi = PicarProtocol.normalize_ir(530.0)
        self.assertGreater(analog_hi, 0.5)
        self.assertEqual(binary_hi, 1)

        # Values outside [100, 950] must clamp to [0.0, 1.0]
        analog_low, binary_low = PicarProtocol.normalize_ir(20.0)
        self.assertAlmostEqual(analog_low, 0.0, places=3)
        self.assertEqual(binary_low, 0)

        analog_high, binary_high = PicarProtocol.normalize_ir(1200.0)
        self.assertAlmostEqual(analog_high, 1.0, places=3)
        self.assertEqual(binary_high, 1)

    def test_format_telemetry(self):
        raw_irs = [100.0, 100.0, 950.0, 100.0, 100.0]
        distance_m = 0.35  # 35 cm
        velocity = 0.20
        ball_dropped = False

        telem_str = PicarProtocol.format_telemetry(
            distance_m=distance_m,
            raw_ir_values=raw_irs,
            velocity=velocity,
            ball_dropped=ball_dropped
        )

        self.assertTrue(telem_str.endswith("\n"), "Telemetry frame must end with newline")
        data = json.loads(telem_str)

        # Distance must be converted from meters to centimeters
        self.assertAlmostEqual(data["distance"], 35.0, places=2)
        self.assertAlmostEqual(data["velocity"], 0.20, places=2)
        self.assertEqual(data["ball_dropped"], False)
        self.assertEqual(data["line_follower"], [0, 0, 1, 0, 0])
        self.assertEqual(len(data["line_analog"]), 5)
        self.assertAlmostEqual(data["line_analog"][2], 1.0, places=2)
        self.assertAlmostEqual(data["line_analog"][0], 0.0, places=2)

    def test_parse_command_valid(self):
        line = '{"velocity": 0.25, "direction": -0.35, "brake": false}\n'
        vel, direction, brake = PicarProtocol.parse_command(line)

        self.assertAlmostEqual(vel, 0.25, places=3)
        self.assertAlmostEqual(direction, -0.35, places=3)
        self.assertFalse(brake)

    def test_parse_command_with_brake(self):
        line = '{"velocity": 0.0, "direction": 0.1, "brake": true}\n'
        vel, direction, brake = PicarProtocol.parse_command(line)

        self.assertAlmostEqual(vel, 0.0, places=3)
        self.assertAlmostEqual(direction, 0.1, places=3)
        self.assertTrue(brake)

    def test_parse_command_direction_clamping(self):
        # Normalized direction must be clamped to [-1.0, 1.0]
        line_over = '{"velocity": 0.1, "direction": 2.5, "brake": false}\n'
        _, dir_over, _ = PicarProtocol.parse_command(line_over)
        self.assertAlmostEqual(dir_over, 1.0, places=3)

        line_under = '{"velocity": 0.1, "direction": -3.0, "brake": false}\n'
        _, dir_under, _ = PicarProtocol.parse_command(line_under)
        self.assertAlmostEqual(dir_under, -1.0, places=3)

    def test_parse_command_corrupted_or_partial(self):
        # Empty string
        self.assertIsNone(PicarProtocol.parse_command(""))

        # Whitespace only
        self.assertIsNone(PicarProtocol.parse_command("   \n"))

        # Truncated JSON
        self.assertIsNone(PicarProtocol.parse_command('{"velocity": 0.2, "dir'))

        # Malformed non-JSON
        self.assertIsNone(PicarProtocol.parse_command("INVALID_GARBAGE"))

        # Status/ACK frame without velocity/direction must return None
        self.assertIsNone(PicarProtocol.parse_command('{"status":"ok","event":"trigger_start"}\n'))

        # Wrong data types
        self.assertIsNone(
            PicarProtocol.parse_command('{"velocity": "fast", "direction": null, "brake": "no"}')
        )

    def test_text_commands(self):
        self.assertEqual(PicarProtocol.CMD_TRIGGER_START, "trigger_start\n")
        self.assertEqual(PicarProtocol.CMD_EMERGENCY_STOP, "emergency_stop\n")
        self.assertEqual(PicarProtocol.format_trigger_start(), "trigger_start\n")
        self.assertEqual(PicarProtocol.format_emergency_stop(), "emergency_stop\n")

    def test_key_handling_edge_detection(self):
        # Key 'S' pressed (ASCII 83 / 'S')
        cmd, active_keys = PicarProtocol.process_key_events(current_keys={ord('S')}, previous_keys=set())
        self.assertEqual(cmd, "trigger_start\n")
        self.assertEqual(active_keys, {ord('S')})

        # Key 'S' held down -> no duplicate command
        cmd_held, active_keys2 = PicarProtocol.process_key_events(current_keys={ord('S')}, previous_keys={ord('S')})
        self.assertIsNone(cmd_held)
        self.assertEqual(active_keys2, {ord('S')})

        # Key 'S' released -> no command
        cmd_rel, active_keys3 = PicarProtocol.process_key_events(current_keys=set(), previous_keys={ord('S')})
        self.assertIsNone(cmd_rel)
        self.assertEqual(active_keys3, set())

        # Space key pressed (ASCII 32 / ' ')
        cmd_space, active_keys4 = PicarProtocol.process_key_events(current_keys={ord(' ')}, previous_keys=set())
        self.assertEqual(cmd_space, "emergency_stop\n")
        self.assertEqual(active_keys4, {ord(' ')})

        # Lowercase 's' also triggers start
        cmd_lower, _ = PicarProtocol.process_key_events(current_keys={ord('s')}, previous_keys=set())
        self.assertEqual(cmd_lower, "trigger_start\n")

        # Other keys (e.g. arrow keys) return None
        cmd_other, _ = PicarProtocol.process_key_events(current_keys={314}, previous_keys=set())
        self.assertIsNone(cmd_other)

    def test_extract_lines(self):
        # Complete single line
        lines, rem = PicarProtocol.extract_lines("hello\n")
        self.assertEqual(lines, ["hello"])
        self.assertEqual(rem, "")

        # Multiple lines with partial remainder
        lines, rem = PicarProtocol.extract_lines("line1\nline2\npartial")
        self.assertEqual(lines, ["line1", "line2"])
        self.assertEqual(rem, "partial")

        # Partial buffer without newline
        lines, rem = PicarProtocol.extract_lines("incomplete")
        self.assertEqual(lines, [])
        self.assertEqual(rem, "incomplete")


class TestPicarSController(unittest.TestCase):
    def test_controller_init_without_webots(self):
        controller = PicarSController(robot=None, host="127.0.0.1", port=9000, auto_start=False)
        self.assertIsNone(controller.robot)
        self.assertEqual(controller.host, "127.0.0.1")
        self.assertEqual(controller.port, 9000)
        self.assertFalse(controller.auto_start)
        self.assertEqual(controller.time_step, 20)

    def test_ackermann_wheels_share_turn_center(self):
        """Les deux roues avant visent le meme centre de rotation que le modele bicyclette du FSM (R = L / tan delta)."""
        import math
        L, half = PicarProtocol.WHEELBASE, PicarProtocol.TRACK / 2.0
        for deg in (5.0, 15.6, 30.0, 45.0, -20.0):
            delta = math.radians(deg)
            left, right = PicarProtocol.ackermann_angles(delta)
            radius = L / math.tan(delta)                     # rayon au centre de l'essieu arriere (+ = gauche)
            self.assertAlmostEqual(L / math.tan(left), radius - half, places=9)
            self.assertAlmostEqual(L / math.tan(right), radius + half, places=9)
            # roue interieure plus braquee que l'exterieure
            inner, outer = (left, right) if deg > 0 else (right, left)
            self.assertGreater(abs(inner), abs(delta))
            self.assertLess(abs(outer), abs(delta))
        self.assertEqual(PicarProtocol.ackermann_angles(0.0), (0.0, 0.0))

    def test_differential_speeds_match_radii(self):
        import math
        delta = math.radians(30.0)
        v_left, v_right = PicarProtocol.differential_speeds(0.3, delta)
        radius = PicarProtocol.WHEELBASE / math.tan(delta)
        half = PicarProtocol.TRACK / 2.0
        self.assertAlmostEqual(v_left, 0.3 * (radius - half) / radius, places=9)
        self.assertAlmostEqual(v_right, 0.3 * (radius + half) / radius, places=9)
        self.assertAlmostEqual((v_left + v_right) / 2.0, 0.3, places=9)

    def test_controller_drives_ackermann_proto(self):
        """Avec le PROTO actuel (2 pivots, 2 moteurs arriere), chaque roue recoit son angle et sa vitesse."""
        robot = DummyRobot()
        names = ("left_steer", "right_steer", "rear_left_motor", "rear_right_motor")
        motors = {n: DummyMotor() for n in names}
        robot.devices.update(motors)
        controller = PicarSController(robot=robot, dynamics=False)
        controller.apply_command(velocity=0.3, direction=0.5, brake=False)
        delta = 0.5 * PicarProtocol.MAX_STEER_ANGLE
        left, right = PicarProtocol.ackermann_angles(delta)
        v_left, v_right = PicarProtocol.differential_speeds(0.3, delta)
        self.assertAlmostEqual(motors["left_steer"].position, left, places=6)
        self.assertAlmostEqual(motors["right_steer"].position, right, places=6)
        self.assertAlmostEqual(motors["rear_left_motor"].velocity, v_left / PicarProtocol.WHEEL_RADIUS, places=6)
        self.assertAlmostEqual(motors["rear_right_motor"].velocity, v_right / PicarProtocol.WHEEL_RADIUS, places=6)
        controller.apply_command(velocity=0.3, direction=0.0, brake=True)
        self.assertEqual(motors["rear_left_motor"].velocity, 0.0)

    def test_auto_start_retries_until_fsm_moves(self):
        """trigger_start perdu (Webots connecte avant le FSM) : renvoye toutes les 3 s, plus jamais une fois la voiture partie."""
        controller = PicarSController(robot=None, auto_start=True, dynamics=False)
        sent = []
        controller.send_message = lambda msg: sent.append(msg) or True
        controller.receive_latest_command = lambda: None
        starts = lambda: sum(1 for m in sent if m == PicarProtocol.CMD_TRIGGER_START)
        for _ in range(100):            # 2 s : premier envoi
            controller.step()
        self.assertEqual(starts(), 1)
        for _ in range(150):            # 3 s sans vitesse commandee : renvoi
            controller.step()
        self.assertEqual(starts(), 2)
        controller.apply_command(0.32, 0.0, False)   # le FSM a demarre
        for _ in range(1000):
            controller.step()
        self.assertEqual(starts(), 2)

    def test_auto_start_gives_up_after_max_attempts(self):
        controller = PicarSController(robot=None, auto_start=True, dynamics=False)
        sent = []
        controller.send_message = lambda msg: sent.append(msg) or True
        controller.receive_latest_command = lambda: None
        for _ in range(5000):
            controller.step()
        self.assertEqual(sent.count(PicarProtocol.CMD_TRIGGER_START), PicarSController.AUTO_START_MAX_ATTEMPTS)

    def test_control_step_is_20ms_with_fine_physics(self):
        robot = DummyRobot()
        robot.getBasicTimeStep = lambda: 4.0
        self.assertEqual(PicarSController(robot=robot).time_step, 20)

    def test_controller_apply_command(self):
        robot = DummyRobot()
        steer_m = DummyMotor()
        drive_m = DummyMotor()
        robot.devices["steering_motor"] = steer_m
        robot.devices["drive_motor"] = drive_m

        controller = PicarSController(robot=robot)

        # Drive forward at 0.33 m/s with 0.5 direction
        controller.apply_command(velocity=0.33, direction=0.5, brake=False)
        self.assertAlmostEqual(controller.current_velocity, 0.33, places=3)
        # Expected steer: 0.5 * PicarProtocol.MAX_STEER_ANGLE rad = 0.225 rad
        self.assertAlmostEqual(steer_m.position, 0.5 * PicarProtocol.MAX_STEER_ANGLE, places=3)
        # Expected wheel omega: 0.33 / WHEEL_RADIUS
        self.assertAlmostEqual(drive_m.velocity, 0.33 / PicarProtocol.WHEEL_RADIUS, places=2)

        # Brake engaged -> omega = 0.0
        controller.apply_command(velocity=0.33, direction=0.0, brake=True)
        self.assertEqual(drive_m.velocity, 0.0)

        # Direction saturation test
        controller.apply_command(velocity=0.1, direction=2.0, brake=False)
        self.assertAlmostEqual(steer_m.position, PicarProtocol.MAX_STEER_ANGLE, places=3)

    def test_controller_socket_loop_and_telemetry(self):
        # Setup a mock TCP server to simulate SimBridge
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server_port = server.getsockname()[1]
        server.listen(1)

        server_received = []

        def handle_client():
            conn, _ = server.accept()
            conn.settimeout(1.0)
            try:
                # Read incoming telemetry frame
                data = b""
                while b"\n" not in data:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                server_received.append(data.decode("utf-8"))
                # Reply with a command frame
                cmd_reply = b'{"velocity": 0.15, "direction": -0.2, "brake": false}\n'
                conn.sendall(cmd_reply)
            finally:
                conn.close()

        server_thread = threading.Thread(target=handle_client, daemon=True)
        server_thread.start()

        try:
            robot = DummyRobot()
            steer_m = DummyMotor()
            drive_m = DummyMotor()
            sonar = DummyDistanceSensor(0.50)  # 50 cm
            robot.devices["steering_motor"] = steer_m
            robot.devices["drive_motor"] = drive_m
            robot.devices["sonar"] = sonar

            controller = PicarSController(robot=robot, host="127.0.0.1", port=server_port)
            connected = controller.try_connect()
            self.assertTrue(connected)

            # Perform step
            status = controller.step()
            self.assertEqual(status, 0)

            # Verify server got telemetry
            server_thread.join(timeout=1.0)
            self.assertTrue(len(server_received) > 0)
            telem_data = json.loads(server_received[0])
            self.assertAlmostEqual(telem_data["distance"], 50.0, places=1)

            # Receive command from server and verify motors updated
            cmd = controller.receive_latest_command()
            if cmd is not None:
                controller.apply_command(*cmd)
                self.assertAlmostEqual(controller.current_velocity, 0.15, places=2)
                self.assertAlmostEqual(steer_m.position, -0.2 * PicarProtocol.MAX_STEER_ANGLE, places=3)
        finally:
            server.close()
            controller.disconnect_socket()


if __name__ == "__main__":
    unittest.main()
