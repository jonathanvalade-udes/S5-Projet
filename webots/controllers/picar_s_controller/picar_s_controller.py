#!/usr/bin/env python3
"""PiCar-S Webots Controller.

Connects the Webots simulation of PiCar-S to the SimBridge service via TCP.
Reads distance sensors (IR line tracker + Sonar), normalizes data, sends
telemetry JSON, receives movement commands, and applies them to motors.
Also handles keyboard input ('S' for trigger_start, 'Space' for emergency_stop).
"""

from __future__ import annotations

import math
import os
import sys
import socket
from pathlib import Path
from typing import List, Optional, Set, Tuple, Any

# Ensure current directory is in sys.path so picar_protocol is importable
CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from ball_tracker import ball_in_cup_frame, is_dropped
from picar_dynamics import PicarDynamics
from picar_protocol import PicarProtocol

try:
    from controller import Robot, Supervisor, Keyboard, Motor, DistanceSensor  # type: ignore
except ImportError:
    # Graceful fallback when importing module outside of Webots runtime (e.g. unit tests)
    Robot = None
    Supervisor = None
    Keyboard = None
    Motor = None
    DistanceSensor = None


class PicarSController:
    """Controller for PiCar-S in Webots simulating kinematics and IPC with SimBridge."""

    AUTO_START_RETRY_S = 3.0        # delai avant de renvoyer trigger_start si le FSM n'a pas demarre
    AUTO_START_MAX_ATTEMPTS = 5

    def __init__(
        self,
        robot: Any = None,
        host: Optional[str] = None,
        port: Optional[int] = None,
        auto_start: Optional[bool] = None,
        dynamics: Optional[bool] = None,
    ):
        """Initializes the controller and devices.

        Args:
            robot: Webots Robot instance (if None and Robot class available, instantiates Robot()).
            host: SimBridge TCP host (defaults to env SIMBRIDGE_HOST or 127.0.0.1).
            port: SimBridge TCP port (defaults to env SIMBRIDGE_PORT or 9000).
            auto_start: If True, sends trigger_start automatically after 2s stabilization.
            dynamics: If True, motor/steering/latency models and the ball model sit between the command and the motors
                (defaults to env WEBOTS_DYNAMICS, off when unset so the raw command is applied).
        """
        self.host = host or os.environ.get("SIMBRIDGE_HOST", PicarProtocol.DEFAULT_HOST)
        self.port = port or int(os.environ.get("SIMBRIDGE_PORT", PicarProtocol.DEFAULT_PORT))

        if auto_start is not None:
            self.auto_start = auto_start
        else:
            self.auto_start = os.environ.get("WEBOTS_AUTO_START", "0") in ("1", "true", "True")

        if robot is not None:
            self.robot = robot
        elif Supervisor is not None:
            # Superviseur si le monde le permet (champ supervisor TRUE) : lecture de la bille physique (Solid DEF BALL)
            self.robot = Supervisor()
        else:
            self.robot = None

        # Pas de controle (50 Hz) : multiple du pas physique du monde, qui peut etre plus fin (4 ms) pour les contacts.
        if self.robot is not None:
            basic = max(1, int(round(self.robot.getBasicTimeStep())))
            self.time_step = basic * max(1, round(PicarProtocol.CONTROL_STEP_MS / basic))
        else:
            self.time_step = PicarProtocol.CONTROL_STEP_MS

        self.keyboard: Any = None
        self.gps: Any = None
        self.imu: Any = None
        self.sonar: Any = None
        self.ir_sensors: List[Any] = []
        self.steering_motor: Any = None
        self.steering_motors: List[Any] = []
        self.drive_motor: Any = None
        self.drive_motors: List[Any] = []

        if dynamics is None:
            dynamics = os.environ.get("WEBOTS_DYNAMICS", "0") in ("1", "true", "True")
        self.dynamics: Optional[PicarDynamics] = PicarDynamics() if dynamics else None
        self._cmd: Tuple[float, float, bool] = (0.0, 0.0, False)
        self.debug: bool = os.environ.get("WEBOTS_DEBUG", "0") in ("1", "true", "True")
        self.ball_dropped: bool = False
        # Bille physique (monde avec DEF BALL et robot superviseur) : prioritaire sur le modele de bille de picar_dynamics
        self.ball_node: Any = None
        self.self_node: Any = None
        self.ball_rel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.ball_max_r: float = 0.0
        # Journal CSV par pas (verite terrain GPS/IMU + capteurs + commande) pour mesurer la fidelite ; vide = desactive.
        self._truth_log = None
        truth_path = os.environ.get("WEBOTS_TRUTH_LOG", "")
        if truth_path:
            self._truth_log = open(truth_path, "w", encoding="utf-8")
            self._truth_log.write("t,x,y,yaw_deg,speed,cmd_v,cmd_dir,ir0,ir1,ir2,ir3,ir4,ball_dx_mm,ball_dy_mm,ball_dropped\n")

        self._device_names: Optional[Set[str]] = None
        self._init_devices()
        self._init_ball()

        # Socket communication state
        self.sock: Optional[socket.socket] = None
        self.rx_buffer: str = ""
        self.step_counter: int = 0
        self.reconnect_interval_steps: int = max(1, int(1000 / self.time_step))  # ~1.0 s
        self.previous_keys: Set[int] = set()
        self.auto_start_sent: bool = False
        self.auto_start_attempts: int = 0
        self._auto_start_step: int = 0
        self._has_moved: bool = False      # une vitesse non nulle a deja ete commandee par le FSM
        self.current_velocity: float = 0.0

    def _device(self, name: str) -> Any:
        """Appareil `name`, ou None s'il n'existe pas, SANS le message d'erreur rouge de Webots pour un nom absent.

        Le controleur accepte plusieurs modeles (direction a 1 ou 2 moteurs, sonar sous plusieurs noms) : il essaie des noms
        qui n'existent pas sur le PROTO actuel, ce que robot.getDevice() signale en rouge dans la console.
        """
        if self._device_names is None:
            names: Set[str] = set()
            try:
                for i in range(self.robot.getNumberOfDevices()):
                    names.add(self.robot.getDeviceByIndex(i).getName())
            except AttributeError:   # robot simule des tests unitaires : pas d'enumeration
                names = set()
            self._device_names = names
        if self._device_names and name not in self._device_names:
            return None
        return self.robot.getDevice(name)

    def _init_devices(self) -> None:
        """Discovers and initializes Webots devices (sensors, motors, keyboard)."""
        if self.robot is None:
            return

        # Keyboard
        if hasattr(self.robot, "getKeyboard"):
            self.keyboard = self.robot.getKeyboard()
            self.keyboard.enable(self.time_step)

        # Verite terrain (debug / calibration) : GPS et centrale inertielle du proto
        self.gps = self._device("gps")
        self.imu = self._device("imu")
        for dev in (self.gps, self.imu):
            if dev is not None:
                dev.enable(self.time_step)

        # Sonar
        for name in ("sonar", "SONAR", "Sonar", "distance_sensor"):
            dev = self._device(name)
            if dev is not None:
                self.sonar = dev
                self.sonar.enable(self.time_step)
                break

        # IR Sensors (IR_0 to IR_4)
        for i in range(5):
            dev = self._device(f"IR_{i}")
            if dev is not None:
                dev.enable(self.time_step)
                self.ir_sensors.append(dev)

        # Direction : deux pivots (Ackermann, PiCarS.proto) [gauche, droite], ou un seul moteur (ancien modele)
        single_steer = self._device("steering_motor")
        if single_steer is not None:
            self.steering_motor = single_steer
            self.steering_motor.setPosition(0.0)
        else:
            for name in ("left_steer", "right_steer"):
                dev = self._device(name)
                if dev is not None:
                    dev.setPosition(0.0)
                    self.steering_motors.append(dev)

        # Propulsion : un moteur par roue arriere [gauche, droite] (differentiel), ou un essieu unique (ancien modele)
        single_drive = self._device("drive_motor")
        if single_drive is not None:
            self.drive_motor = single_drive
            self.drive_motor.setPosition(float("inf"))
            self.drive_motor.setVelocity(0.0)
        else:
            for name in ("rear_left_motor", "rear_right_motor"):
                dev = self._device(name)
                if dev is not None:
                    dev.setPosition(float("inf"))
                    dev.setVelocity(0.0)
                    self.drive_motors.append(dev)

    def try_connect(self) -> bool:
        """Attempts a non-blocking connection to SimBridge."""
        if self.sock is not None:
            return True

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.02)
            s.connect((self.host, self.port))
            s.setblocking(False)
            self.sock = s
            self.rx_buffer = ""
            print(f"[PicarSController] Connected to SimBridge at {self.host}:{self.port}")
            return True
        except (socket.error, OSError):
            self.sock = None
            return False

    def disconnect_socket(self) -> None:
        """Safely closes the socket connection."""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self.rx_buffer = ""
            print("[PicarSController] Disconnected from SimBridge.")

    def send_message(self, message: str) -> bool:
        """Sends a string message through the socket."""
        if self.sock is None:
            return False
        try:
            self.sock.sendall(message.encode("utf-8"))
            return True
        except (BrokenPipeError, ConnectionResetError, socket.error):
            self.disconnect_socket()
            return False

    def receive_latest_command(self) -> Optional[Tuple[float, float, bool]]:
        """Reads all pending lines from the socket and returns the latest parsed command."""
        if self.sock is None:
            return None

        try:
            chunk = self.sock.recv(4096)
            if not chunk:
                # EOF from server
                self.disconnect_socket()
                return None
            self.rx_buffer += chunk.decode("utf-8", errors="ignore")
            lines, self.rx_buffer = PicarProtocol.extract_lines(self.rx_buffer)

            latest_cmd = None
            for line in lines:
                line_str = line.strip()
                if not line_str:
                    continue
                cmd = PicarProtocol.parse_command(line_str)
                if cmd is not None:
                    latest_cmd = cmd
            return latest_cmd
        except (BlockingIOError, socket.error):
            return None

    def apply_command(self, velocity: float, direction: float, brake: bool) -> None:
        """Applies steering and drive commands to Webots motors.

        Args:
            velocity: Commanded linear velocity (m/s).
            direction: Commanded normalized direction [-1.0, 1.0].
            brake: Brake engagement flag.
        """
        self.current_velocity = velocity
        if abs(velocity) > 1e-3:
            self._has_moved = True
        self._cmd = (velocity, direction, brake)
        if self.dynamics is not None:
            return   # les moteurs sont pilotes a chaque pas par _apply_dynamics()
        self._drive_motors(velocity, direction, brake)

    def _init_ball(self) -> None:
        """Trouve la bille physique si le robot est superviseur et que le monde contient DEF BALL."""
        if self.robot is None or not hasattr(self.robot, "getFromDef"):
            return
        try:
            if hasattr(self.robot, "getSupervisor") and not self.robot.getSupervisor():
                return
            self.ball_node = self.robot.getFromDef("BALL")
            self.self_node = self.robot.getSelf()
        except Exception:
            self.ball_node = None
        if self.ball_node is not None:
            print("[PicarSController] Bille physique trouvee (DEF BALL) : ball_dropped vient du moteur physique.")

    def _update_physical_ball(self) -> None:
        if self.ball_node is None or self.self_node is None:
            return
        self.ball_rel = ball_in_cup_frame(self.self_node.getPosition(), self.self_node.getOrientation(),
                                          self.ball_node.getPosition())
        self.ball_max_r = max(self.ball_max_r, math.hypot(self.ball_rel[0], self.ball_rel[1]))
        if not self.ball_dropped and is_dropped(self.ball_rel):
            self.ball_dropped = True
            print(f"[PicarSController] ALERTE FastDrink : la bille physique est sortie de la coupelle "
                  f"(t = {self.step_counter * self.time_step / 1000:.2f} s, decalage max {self.ball_max_r * 1000:.1f} mm)")

    def _apply_dynamics(self) -> None:
        """Passe la derniere consigne dans les modeles moteur/direction/bille puis pilote les moteurs."""
        velocity, direction, brake = self._cmd
        dt = self.time_step / 1000.0
        speed, angle, dropped = self.dynamics.step(0.0 if brake else velocity, direction, dt)
        if self.ball_node is None:   # pas de bille physique : modele de picar_dynamics
            if dropped and not self.ball_dropped:
                print(f"[PicarSController] ALERTE FastDrink (modele) : la bille est tombee (pic a = "
                      f"{self.dynamics.ball.max_accel:.3f} m/s2)")
            self.ball_dropped = dropped
        # angle reel (rad) -> commande normalisee sur la plage du servo, puis meme chemin que sans modele
        self._drive_motors(speed, angle / max(1e-6, PicarProtocol.MAX_STEER_ANGLE), False)

    def _drive_motors(self, velocity: float, direction: float, brake: bool) -> None:
        """Pilote les moteurs. direction : braquage normalise [-1, 1] (+ = gauche) de l'angle bicyclette equivalent."""
        clamped_dir = max(-1.0, min(1.0, direction))
        steer_angle = clamped_dir * PicarProtocol.MAX_STEER_ANGLE

        if self.steering_motor is not None:
            self.steering_motor.setPosition(steer_angle)
        if len(self.steering_motors) == 2:
            left, right = PicarProtocol.ackermann_angles(steer_angle)
            self.steering_motors[0].setPosition(left)
            self.steering_motors[1].setPosition(right)
        else:
            for m in self.steering_motors:
                m.setPosition(steer_angle)

        if brake or abs(velocity) < 1e-4:
            velocity = 0.0
        radius = PicarProtocol.WHEEL_RADIUS
        if self.drive_motor is not None:
            self.drive_motor.setVelocity(velocity / radius)
        if len(self.drive_motors) == 2:
            v_left, v_right = PicarProtocol.differential_speeds(velocity, steer_angle)
            self.drive_motors[0].setVelocity(v_left / radius)
            self.drive_motors[1].setVelocity(v_right / radius)
        else:
            for m in self.drive_motors:
                m.setVelocity(velocity / radius)

    def read_sensors(self) -> Tuple[float, List[float]]:
        """Reads raw values from distance sensor and IR sensors.

        Returns:
            Tuple of (sonar_distance_meters, list_of_raw_ir_values).
        """
        if self.sonar is not None:
            distance_m = float(self.sonar.getValue())
        else:
            distance_m = 1.0  # 1.0 m = 100 cm default safe distance

        if self.ir_sensors:
            raw_irs = [float(s.getValue()) for s in self.ir_sensors]
        else:
            # Default to white ground (no line detected)
            raw_irs = [PicarProtocol.DEFAULT_IR_MIN] * 5

        # SimBridge inverse l'ordre des capteurs (convention Godot : indice 0 = droite). Le proto Webots numerote de la gauche
        # (IR_0, +Y) vers la droite (IR_4, -Y) : on envoie donc l'ordre inverse pour que le FSM voie la ligne du bon cote
        # (sans cela l'erreur de ligne est inversee et le robot s'ecarte de plus en plus).
        raw_irs.reverse()
        return distance_m, raw_irs

    def process_keyboard(self) -> Optional[str]:
        """Polls keyboard events and returns text command if triggered."""
        current_keys: Set[int] = set()
        if self.keyboard is not None:
            key = self.keyboard.getKey()
            while key > 0:
                current_keys.add(key)
                key = self.keyboard.getKey()

        cmd, self.previous_keys = PicarProtocol.process_key_events(
            current_keys, self.previous_keys
        )
        return cmd

    def step(self) -> int:
        """Executes a single control and communication step (50 Hz).

        Returns:
            Status from robot.step() or 0 if outside Webots.
        """
        self.step_counter += 1

        # 1. Webots simulation step
        status = 0
        if self.robot is not None:
            status = self.robot.step(self.time_step)
            if status == -1:
                return -1

        self._update_physical_ball()

        # 2. Socket reconnection management
        if self.sock is None:
            if self.step_counter % self.reconnect_interval_steps == 0:
                self.try_connect()

        # 3. Keyboard handling
        kb_cmd = self.process_keyboard()
        if kb_cmd:
            self.send_message(kb_cmd)

        # 4. Auto-start (apres 2 s de stabilisation). Si Webots se connecte avant que le FSM ecoute (pile lancee en meme
        #    temps), SimBridge perd le trigger_start : on le renvoie toutes les 3 s tant que le FSM n'a commande aucune
        #    vitesse, 5 fois au plus, et jamais une fois que la voiture a roule (pour ne pas relancer apres une arrivee).
        if self.auto_start and not self._has_moved and self.auto_start_attempts < self.AUTO_START_MAX_ATTEMPTS:
            stabilization_steps = max(1, int(2.0 / (self.time_step / 1000.0)))
            retry_steps = max(1, int(self.AUTO_START_RETRY_S / (self.time_step / 1000.0)))
            due = (self.step_counter >= stabilization_steps if not self.auto_start_sent
                   else self.step_counter - self._auto_start_step >= retry_steps)
            if due and self.send_message(PicarProtocol.CMD_TRIGGER_START):
                self.auto_start_sent = True
                self.auto_start_attempts += 1
                self._auto_start_step = self.step_counter
                print(f"[PicarSController] Auto-start command sent (essai {self.auto_start_attempts}).")

        # 5. Read sensors and send telemetry
        distance_m, raw_irs = self.read_sensors()
        if self.debug and self.step_counter % 25 == 0:
            analog = [PicarProtocol.normalize_ir(r)[0] for r in raw_irs]
            truth = ""
            if self.ball_node is not None:
                truth += (f" | bille physique decalage={math.hypot(*self.ball_rel[:2]) * 1000:.1f} mm "
                          f"(max {self.ball_max_r * 1000:.1f}) tombee={self.ball_dropped}")
            elif self.dynamics is not None:
                truth += f" | bille modele pic a/a_crit={self.dynamics.ball.peak_ratio:.2f} tombee={self.ball_dropped}"
            if self.gps is not None and self.imu is not None:
                x, y, z = self.gps.getValues()
                truth += (f" | GPS x={x:.3f} y={y:.3f} z={z:.4f} v={self.gps.getSpeed():.3f} m/s "
                         f"yaw={math.degrees(self.imu.getRollPitchYaw()[2]):.1f}deg")
            print(f"[PicarSController] t={self.step_counter * self.time_step / 1000:.1f}s sonar={distance_m:.2f} m "
                  f"IR raw={[round(r) for r in raw_irs]} darkness={analog} cmd(v,dir)={self._cmd[:2]}{truth}")
        if self._truth_log is not None and self.gps is not None and self.imu is not None:
            x, y, _ = self.gps.getValues()
            analog = [PicarProtocol.normalize_ir(r)[0] for r in raw_irs]
            self._truth_log.write(
                f"{self.step_counter * self.time_step / 1000:.3f},{x:.5f},{y:.5f},"
                f"{math.degrees(self.imu.getRollPitchYaw()[2]):.2f},{self.gps.getSpeed():.4f},"
                f"{self._cmd[0]:.4f},{self._cmd[1]:.4f}," + ",".join(f"{a:.3f}" for a in analog) +
                f",{self.ball_rel[0] * 1000:.2f},{self.ball_rel[1] * 1000:.2f},{int(self.ball_dropped)}\n")
            if self.step_counter % 25 == 0:   # lisible en direct (sinon ecrit seulement a la fermeture de Webots)
                self._truth_log.flush()
        telem_msg = PicarProtocol.format_telemetry(
            distance_m=distance_m,
            raw_ir_values=raw_irs,
            velocity=self.current_velocity,
            ball_dropped=self.ball_dropped,
        )
        self.send_message(telem_msg)

        # 6. Receive and apply commands
        cmd = self.receive_latest_command()
        if cmd is not None:
            vel, direction, brake = cmd
            self.apply_command(vel, direction, brake)
        if self.dynamics is not None:
            self._apply_dynamics()

        return status

    def run(self) -> None:
        """Main loop of the controller."""
        print(f"[PicarSController] Starting controller loop (timestep: {self.time_step} ms)...")
        self.try_connect()

        try:
            while self.step() != -1:
                pass
        finally:
            self.disconnect_socket()


if __name__ == "__main__":
    controller = PicarSController()
    controller.run()
