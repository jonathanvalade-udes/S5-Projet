"""End-to-End integration tests for Webots PiCar-S controller and SimBridge TCP mock.

Validates the full communication loop:
1. Bidirectional 50 Hz telemetry stream and motor actuation.
2. Keyboard event detection and transmission:
   - 'S' / 's' -> trigger_start
   - ' ' (Space) -> emergency_stop
3. Server disconnection and automatic reconnection handling.
4. Autonomous start sequence after stabilization (WEBOTS_AUTO_START).
5. Robustness against segmented TCP streams and rapid step execution.
"""

from __future__ import annotations

import json
import os
import queue
import socket
import sys
import threading
import time
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

CONTROLLER_DIR = Path(__file__).resolve().parent.parent / "controllers" / "picar_s_controller"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from picar_protocol import PicarProtocol
from picar_s_controller import PicarSController


class MockMotor:
    """Mock Webots Motor device tracking position and velocity."""

    def __init__(self, name: str = "motor"):
        self.name = name
        self.position: float = 0.0
        self.velocity: float = 0.0

    def setPosition(self, pos: float) -> None:
        self.position = float(pos)

    def setVelocity(self, vel: float) -> None:
        self.velocity = float(vel)


class MockDistanceSensor:
    """Mock Webots DistanceSensor device returning specified value."""

    def __init__(self, name: str = "sensor", value: float = 0.0):
        self.name = name
        self.value: float = float(value)
        self.enabled: bool = False
        self.timestep: int = 0

    def getValue(self) -> float:
        return self.value

    def setValue(self, val: float) -> None:
        self.value = float(val)

    def enable(self, timestep: int) -> None:
        self.enabled = True
        self.timestep = timestep


class MockKeyboard:
    """Mock Webots Keyboard queueing and delivering key presses."""

    def __init__(self):
        self._key_queue: List[int] = []
        self.enabled: bool = False
        self.timestep: int = 0

    def push_keys(self, keys: List[int]) -> None:
        """Enqueues key codes to be read on subsequent getKey calls."""
        self._key_queue.extend(keys)

    def getKey(self) -> int:
        if self._key_queue:
            return self._key_queue.pop(0)
        return -1

    def enable(self, timestep: int) -> None:
        self.enabled = True
        self.timestep = timestep


class MockRobot:
    """Mock Webots Robot managing devices, keyboard and simulation steps."""

    def __init__(self, timestep: int = 20):
        self.basic_time_step: float = float(timestep)
        self.keyboard = MockKeyboard()
        self.devices: Dict[str, Any] = {}
        self.step_count: int = 0

    def getBasicTimeStep(self) -> float:
        return self.basic_time_step

    def getKeyboard(self) -> MockKeyboard:
        return self.keyboard

    def getDevice(self, name: str) -> Optional[Any]:
        return self.devices.get(name, None)

    def add_device(self, name: str, device: Any) -> None:
        self.devices[name] = device

    def step(self, timestep: int) -> int:
        self.step_count += 1
        return 0


class MockSimBridgeServer:
    """Threaded TCP server faithfully simulating SimBridge behavior.

    - Listens on 127.0.0.1 on an OS-assigned ephemeral port.
    - Handles incoming telemetry JSON lines and text commands.
    - Responds with ACKs or command frames matching SimProtocolHandler.cpp.
    - Allows simulating disconnects and inspection of received messages.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        default_reply: Optional[str] = '{"velocity": 0.32, "direction": 0.0, "brake": false}\n',
    ):
        self.host = host
        self.default_reply = default_reply
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, 0))
        self.port: int = self.server_sock.getsockname()[1]
        self.server_sock.listen(5)
        self.server_sock.settimeout(0.2)

        self.running: bool = False
        self.thread: Optional[threading.Thread] = None

        self._client_sock: Optional[socket.socket] = None
        self._client_lock = threading.Lock()

        self._received_lines: List[str] = []
        self._lines_lock = threading.Lock()
        self._line_event = threading.Event()

        self._out_queue: queue.Queue[str] = queue.Queue()

    def start(self) -> None:
        """Starts the server thread."""
        self.running = True
        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        """Stops the server and terminates all active connections."""
        self.running = False
        self.disconnect_client()
        try:
            self.server_sock.close()
        except OSError:
            pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def disconnect_client(self) -> None:
        """Forcibly disconnects the currently connected client."""
        with self._client_lock:
            if self._client_sock is not None:
                try:
                    self._client_sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self._client_sock.close()
                except OSError:
                    pass
                self._client_sock = None

    def send_command(self, cmd_line: str) -> None:
        """Enqueues a command string (with trailing newline) to send to client."""
        if not cmd_line.endswith("\n"):
            cmd_line += "\n"
        self._out_queue.put(cmd_line)

    def get_received_lines(self) -> List[str]:
        """Returns a snapshot copy of all received lines."""
        with self._lines_lock:
            return list(self._received_lines)

    def wait_for_line(
        self, predicate: Callable[[str], bool], timeout: float = 1.5
    ) -> bool:
        """Blocks until a received line satisfies the predicate or timeout occurs."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lines_lock:
                for line in self._received_lines:
                    if predicate(line):
                        return True
            self._line_event.wait(timeout=0.05)
            self._line_event.clear()
        return False

    def clear_received_lines(self) -> None:
        """Clears the buffer of received lines."""
        with self._lines_lock:
            self._received_lines.clear()
        self._line_event.clear()

    def _server_loop(self) -> None:
        while self.running:
            try:
                client, _ = self.server_sock.accept()
            except (socket.timeout, OSError):
                continue

            with self._client_lock:
                self._client_sock = client

            client.settimeout(0.05)
            rx_buf = ""

            while self.running:
                # 1. Send queued outgoing commands
                try:
                    while True:
                        msg = self._out_queue.get_nowait()
                        client.sendall(msg.encode("utf-8"))
                except queue.Empty:
                    pass
                except OSError:
                    break

                # 2. Receive incoming data
                try:
                    data = client.recv(4096)
                    if not data:
                        break
                    rx_buf += data.decode("utf-8", errors="ignore")
                    while "\n" in rx_buf:
                        line, rx_buf = rx_buf.split("\n", 1)
                        line_str = line.strip()
                        if not line_str:
                            continue

                        with self._lines_lock:
                            self._received_lines.append(line_str)
                        self._line_event.set()

                        # Emulate SimProtocolHandler ACKs / responses
                        if "trigger_start" in line_str or line_str == "start":
                            ack = '{"status":"ok","event":"trigger_start"}\n'
                            client.sendall(ack.encode("utf-8"))
                        elif "emergency_stop" in line_str or line_str == "stop":
                            ack = '{"status":"ok","event":"emergency_stop"}\n'
                            client.sendall(ack.encode("utf-8"))
                        elif "distance" in line_str or "line_follower" in line_str:
                            if self.default_reply:
                                client.sendall(self.default_reply.encode("utf-8"))
                except socket.timeout:
                    continue
                except OSError:
                    break

            with self._client_lock:
                if self._client_sock is client:
                    try:
                        client.close()
                    except OSError:
                        pass
                    self._client_sock = None


class TestE2EMockBridge(unittest.TestCase):
    """E2E integration test suite for PiCar-S controller talking to mock SimBridge."""

    def setUp(self):
        self.server = MockSimBridgeServer()
        self.server.start()

        # Build mock PiCar-S robot
        self.robot = MockRobot(timestep=20)
        self.steer_motor = MockMotor("steering_motor")
        self.drive_motor = MockMotor("drive_motor")
        self.sonar = MockDistanceSensor("SONAR", value=0.42)  # 42 cm
        self.ir_sensors = [
            MockDistanceSensor(f"IR_{i}", value=PicarProtocol.DEFAULT_IR_MIN)
            for i in range(5)
        ]
        # Middle sensor on black tape
        self.ir_sensors[2].setValue(PicarProtocol.DEFAULT_IR_MAX)

        self.robot.add_device("steering_motor", self.steer_motor)
        self.robot.add_device("drive_motor", self.drive_motor)
        self.robot.add_device("SONAR", self.sonar)
        for i, ir in enumerate(self.ir_sensors):
            self.robot.add_device(f"IR_{i}", ir)

        self.controller = PicarSController(
            robot=self.robot,
            host=self.server.host,
            port=self.server.port,
            auto_start=False,
        )

    def tearDown(self):
        self.controller.disconnect_socket()
        self.server.stop()

    def test_e2e_telemetry_and_command_exchange(self):
        """Validates 50 Hz telemetry reporting and command reception / actuation."""
        # Custom command reply from bridge: 0.25 m/s, -0.4 direction (turn right), brake=False
        cmd_reply = '{"velocity": 0.25, "direction": -0.4, "brake": false}\n'
        self.server.default_reply = cmd_reply

        connected = self.controller.try_connect()
        self.assertTrue(connected, "Controller must successfully connect to mock SimBridge")

        # Step 1: Telemetry sent, server queues command response
        status = self.controller.step()
        self.assertEqual(status, 0)

        # Wait for server to receive at least one telemetry frame
        received = self.server.wait_for_line(lambda l: "distance" in l, timeout=1.0)
        self.assertTrue(received, "Server should receive telemetry frame")

        lines = self.server.get_received_lines()
        telem_line = next(l for l in lines if "distance" in l)
        telem = json.loads(telem_line)

        # Verify telemetry content
        self.assertAlmostEqual(telem["distance"], 42.0, places=1)
        self.assertEqual(telem["line_follower"], [0, 0, 1, 0, 0])
        self.assertEqual(len(telem["line_analog"]), 5)
        self.assertAlmostEqual(telem["line_analog"][2], 1.0, places=2)
        self.assertAlmostEqual(telem["line_analog"][0], 0.0, places=2)
        self.assertFalse(telem["ball_dropped"])

        # Allow socket communication to settle
        time.sleep(0.05)

        # Step 2: Controller reads server reply and applies to motors
        self.controller.step()

        # Actuation checks:
        # steer_angle = -0.4 * PicarProtocol.MAX_STEER_ANGLE rad = -0.18 rad
        self.assertAlmostEqual(self.steer_motor.position, -0.4 * PicarProtocol.MAX_STEER_ANGLE, places=3)
        # target_omega = 0.25 m/s / WHEEL_RADIUS
        self.assertAlmostEqual(self.drive_motor.velocity, 0.25 / PicarProtocol.WHEEL_RADIUS, places=2)
        self.assertAlmostEqual(self.controller.current_velocity, 0.25, places=2)

    def test_e2e_keyboard_trigger_start(self):
        """Validates pressing 'S' sends trigger_start\\n and server acknowledges it."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Simulate user pressing 'S' key
        self.robot.keyboard.push_keys([ord("S")])

        self.controller.step()

        # Check server received trigger_start
        found_trigger = self.server.wait_for_line(
            lambda l: l == "trigger_start", timeout=1.0
        )
        self.assertTrue(found_trigger, "Server must receive 'trigger_start' command")

        # Step again without key press - should not send duplicate trigger_start (edge-triggered)
        self.server.clear_received_lines()
        self.controller.step()
        time.sleep(0.05)

        lines = self.server.get_received_lines()
        self.assertNotIn("trigger_start", lines, "Key event must be edge-triggered, no spam")

    def test_e2e_keyboard_emergency_stop(self):
        """Validates pressing ' ' (Space) sends emergency_stop\\n."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Simulate user pressing ' ' (Space key)
        self.robot.keyboard.push_keys([ord(" ")])

        self.controller.step()

        found_stop = self.server.wait_for_line(
            lambda l: l == "emergency_stop", timeout=1.0
        )
        self.assertTrue(found_stop, "Server must receive 'emergency_stop' command")

    def test_e2e_brake_command(self):
        """Validates that a brake command immediately zeroes motor velocity."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Initially set car moving
        self.controller.apply_command(velocity=0.3, direction=0.0, brake=False)
        self.assertGreater(self.drive_motor.velocity, 0.0)

        # Server sends brake command
        self.server.send_command('{"velocity": 0.0, "direction": 0.0, "brake": true}')
        time.sleep(0.05)

        self.controller.step()
        self.assertEqual(self.drive_motor.velocity, 0.0)

    def test_e2e_disconnect_and_reconnection(self):
        """Validates graceful recovery and automatic reconnection after TCP disconnect."""
        # Reconnection interval is 1.0s (50 steps at 20ms)
        self.assertEqual(self.controller.reconnect_interval_steps, 50)

        connected = self.controller.try_connect()
        self.assertTrue(connected)
        self.assertIsNotNone(self.controller.sock)

        # Step 1: Telemetry flows normally
        self.controller.step()
        self.assertTrue(self.server.wait_for_line(lambda l: "distance" in l, timeout=1.0))

        # Sever the connection from server side
        self.server.disconnect_client()
        time.sleep(0.05)

        # Steps drain remaining buffer and detect EOF/closed socket
        for _ in range(5):
            if self.controller.sock is None:
                break
            self.controller.step()

        self.assertIsNone(self.controller.sock, "Controller should drop closed socket")

        # Clear received telemetry history on server
        self.server.clear_received_lines()

        # Step through the remaining steps until the reconnect interval is reached
        steps_to_reconnect = self.controller.reconnect_interval_steps - (
            self.controller.step_counter % self.controller.reconnect_interval_steps
        )
        for _ in range(steps_to_reconnect):
            self.controller.step()

        # Connection should now be restored
        self.assertIsNotNone(
            self.controller.sock, "Controller should have reconnected after interval"
        )

        # Verify telemetry resumes flowing to the server
        resumed = self.server.wait_for_line(lambda l: "distance" in l, timeout=1.0)
        self.assertTrue(resumed, "Telemetry should resume flowing after reconnection")

    def test_e2e_ir_sensor_updates_reflected_in_telemetry(self):
        """Validates that real-time sensor updates reflect accurately in telemetry."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Initially middle sensor on line
        self.controller.step()
        self.server.wait_for_line(lambda l: "distance" in l, timeout=1.0)
        self.server.clear_received_lines()

        # Simulate car veering: IR_0 (leftmost) and IR_1 on line, middle sensor off line
        self.ir_sensors[0].setValue(PicarProtocol.DEFAULT_IR_MAX)
        self.ir_sensors[1].setValue(PicarProtocol.DEFAULT_IR_MAX)
        self.ir_sensors[2].setValue(PicarProtocol.DEFAULT_IR_MIN)
        self.sonar.setValue(0.18)  # 18 cm

        self.controller.step()
        self.server.wait_for_line(lambda l: "distance" in l, timeout=1.0)

        lines = self.server.get_received_lines()
        telem = json.loads(lines[-1])
        self.assertAlmostEqual(telem["distance"], 18.0, places=1)
        # L'ordre est inverse a l'envoi (SimBridge inverse a nouveau : indice 0 = droite dans la convention Godot) :
        # IR_0 (gauche) arrive en derniere position.
        self.assertEqual(telem["line_follower"], [0, 0, 0, 1, 1])
        self.assertAlmostEqual(telem["line_analog"][4], 1.0, places=2)
        self.assertAlmostEqual(telem["line_analog"][3], 1.0, places=2)
        self.assertAlmostEqual(telem["line_analog"][2], 0.0, places=2)

    def test_e2e_malformed_command_resilience(self):
        """Validates that malformed JSON from bridge does not crash controller."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Set initial motor state
        self.controller.apply_command(0.2, 0.1, False)

        # Send corrupted frames
        self.server.send_command('{"corrupted_json": true')
        self.server.send_command('not-even-json\n')
        self.server.send_command('{"velocity": "invalid_number"}\n')
        time.sleep(0.05)

        # Controller steps through without raising exceptions
        self.controller.step()

        # Controller should remain connected and functioning
        self.assertIsNotNone(self.controller.sock)

    def test_e2e_multiline_buffered_commands(self):
        """Validates that when multiple commands arrive in batch, latest is applied."""
        connected = self.controller.try_connect()
        self.assertTrue(connected)

        # Send two commands in quick succession
        cmd1 = '{"velocity": 0.10, "direction": 0.1, "brake": false}\n'
        cmd2 = '{"velocity": 0.40, "direction": -0.5, "brake": false}\n'
        self.server.send_command(cmd1 + cmd2)
        time.sleep(0.05)

        self.controller.step()

        # Latest command (cmd2) must be applied
        self.assertAlmostEqual(self.controller.current_velocity, 0.40, places=2)
        self.assertAlmostEqual(self.steer_motor.position, -0.5 * PicarProtocol.MAX_STEER_ANGLE, places=3)

    def test_e2e_auto_start_sequence(self):
        """Validates that WEBOTS_AUTO_START transmits trigger_start after 2s stabilization."""
        # Avant le depart, le vrai FSM (etat Armed) commande une vitesse nulle. Une vitesse non nulle voudrait dire que la
        # voiture roule deja, et le controleur n'enverrait alors pas de depart.
        self.server.default_reply = '{"velocity": 0.0, "direction": 0.0, "brake": false}\n'
        controller = PicarSController(
            robot=self.robot,
            host=self.server.host,
            port=self.server.port,
            auto_start=True,
        )
        self.assertTrue(controller.try_connect())

        try:
            # 2.0 seconds at 20 ms/step = 100 steps
            # Steps 1 to 99: No trigger_start sent
            for _ in range(99):
                controller.step()

            lines = self.server.get_received_lines()
            self.assertNotIn(
                "trigger_start",
                lines,
                "trigger_start must NOT be sent before 2s stabilization completes",
            )

            # Step 100: Stabilization reached -> trigger_start sent
            controller.step()
            found = self.server.wait_for_line(
                lambda l: l == "trigger_start", timeout=1.0
            )
            self.assertTrue(found, "trigger_start must be sent on step 100")
            self.assertTrue(controller.auto_start_sent)

            # Step 101: Auto-start sent flag prevents repeated triggers
            self.server.clear_received_lines()
            controller.step()
            time.sleep(0.05)
            lines_after = self.server.get_received_lines()
            self.assertNotIn("trigger_start", lines_after)
        finally:
            controller.disconnect_socket()


if __name__ == "__main__":
    unittest.main()
