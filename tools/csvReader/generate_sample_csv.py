#!/usr/bin/env python3
"""
Générateur de données de test CSV pour la télémétrie FastDrink / PiCar-S.
Génère un fichier CSV réaliste contenant l'intégralité des 78 colonnes de CarTelemetry.
Scénario simulé :
  - Démarrage : Init -> Unarmed -> Armed
  - Roulage nominal : LineFollowingForward (suivi de ligne droite et virages)
  - Déviation / correction : RightDeviation puis retour à LineFollowingForward
  - Détection obstacle sonar : CollisionRisk puis Stopped
"""

import csv
import math
import random

HEADER = [
    "timestamp_ms", "seq_number",
    # FSM
    "fsm_current_state", "fsm_requested_state", "fsm_flags", "fsm_error_code",
    # Sonar
    "sonar_raw_dist_cm", "sonar_calc_dist_cm", "sonar_flags", "sonar_error_code",
    # Line follower
    "line_flags",
    "s0_raw", "s0_norm", "s0_corr", "s0_err",
    "s1_raw", "s1_norm", "s1_corr", "s1_err",
    "s2_raw", "s2_norm", "s2_corr", "s2_err",
    "s3_raw", "s3_norm", "s3_corr", "s3_err",
    "s4_raw", "s4_norm", "s4_corr", "s4_err",
    "sensors_comp_flags", "sensors_comp_err",
    # Motors
    "motors_state", "motors_comp_flags", "motors_error_code",
    # Left motor
    "m_left_target_throttle", "m_left_applied_throttle", "m_left_encoder",
    "m_left_flags", "m_left_pid_flags", "m_left_kp", "m_left_kd", "m_left_ki",
    # Right motor
    "m_right_target_throttle", "m_right_applied_throttle", "m_right_encoder",
    "m_right_flags", "m_right_pid_flags", "m_right_kp", "m_right_kd", "m_right_ki",
    # Steering
    "steering_target_angle", "steering_current_angle",
    "steering_target_ccr", "steering_current_ccr",
    "steering_coeff", "steering_flags", "steering_error_code",
    # Aux servo
    "aux_servo_target_angle", "aux_servo_current_angle",
    "aux_servo_target_ccr", "aux_servo_current_ccr",
    "aux_servo_coeff", "aux_servo_flags", "aux_servo_error_code",
    # Services & Daemons
    "services_flags", "services_timestamp", "services_handler_err",
    "daemons_count",
    "daemon1_status", "daemon1_err",
    "daemon2_status", "daemon2_err",
    "daemon3_status", "daemon3_err",
    "daemon4_status", "daemon4_err"
]

def generate_sample_csv(filename="sample_telemetry.csv", duration_sec=25.0, sample_rate_hz=20.0):
    total_frames = int(duration_sec * sample_rate_hz)
    dt = 1.0 / sample_rate_hz
    start_time_ms = 1730810000000

    left_encoder = 1000
    right_encoder = 1000

    cur_steering = 0.0
    cur_throttle_left = 0.0
    cur_throttle_right = 0.0
    sonar_dist = 120.0

    rows = []

    for i in range(total_frames):
        t = i * dt
        ts_ms = start_time_ms + int(t * 1000)
        seq = i + 1

        # FSM State machine timeline
        if t < 1.0:
            fsm_state = 0  # Init
            req_state = 1
            motor_st = 0   # Init
            target_throttle = 0.0
            steer_target = 0.0
            obstacle = False
        elif t < 2.5:
            fsm_state = 1  # Unarmed
            req_state = 2
            motor_st = 1   # Idle
            target_throttle = 0.0
            steer_target = 0.0
            obstacle = False
        elif t < 3.5:
            fsm_state = 2  # Armed
            req_state = 3
            motor_st = 1   # Idle
            target_throttle = 0.0
            steer_target = 0.0
            obstacle = False
        elif t < 8.0:
            # Ligne droite
            fsm_state = 3  # LineFollowingForward
            req_state = 3
            motor_st = 2   # Running
            target_throttle = 0.45
            steer_target = math.sin(t * 2.0) * 4.0  # Petites oscillations naturelles
            obstacle = False
        elif t < 11.5:
            # Virage à droite
            fsm_state = 7  # RightDeviation
            req_state = 7
            motor_st = 2
            target_throttle = 0.35
            steer_target = 25.0 + math.sin(t * 3.0) * 3.0
            obstacle = False
        elif t < 15.0:
            # Virage à gauche pour revenir
            fsm_state = 8  # LeftDeviation
            req_state = 8
            motor_st = 2
            target_throttle = 0.35
            steer_target = -22.0 + math.sin(t * 3.0) * 2.5
            obstacle = False
        elif t < 18.0:
            # Ligne droite avec approche obstacle
            fsm_state = 3  # LineFollowingForward
            req_state = 3
            motor_st = 2
            target_throttle = 0.40
            steer_target = 0.0
            obstacle = True
        elif t < 20.0:
            # Ralentissement risque collision
            fsm_state = 4  # CollisionRisk
            req_state = 5
            motor_st = 2
            target_throttle = 0.15
            steer_target = 0.0
            obstacle = True
        else:
            # Arrêt complet face obstacle
            fsm_state = 5  # Stopped
            req_state = 5
            motor_st = 1   # Idle
            target_throttle = 0.0
            steer_target = 0.0
            obstacle = True

        # Throttle dynamics (lag)
        cur_throttle_left += (target_throttle - cur_throttle_left) * 0.25
        cur_throttle_right += (target_throttle - cur_throttle_right) * 0.25

        # Steering dynamics
        cur_steering += (steer_target - cur_steering) * 0.35

        # Encoders increment
        speed_left = cur_throttle_left * 220.0 + random.uniform(-1, 1)
        speed_right = cur_throttle_right * 220.0 + random.uniform(-1, 1)
        left_encoder += int(max(0, speed_left * dt))
        right_encoder += int(max(0, speed_right * dt))

        # Sonar distance
        if obstacle:
            # Obstacle se rapproche
            dist_ideal = max(12.0, 90.0 - (t - 16.0) * 28.0)
        else:
            dist_ideal = 120.0 + random.uniform(-2, 2)
        sonar_raw = dist_ideal + random.gauss(0, 1.2)
        sonar_dist += (sonar_raw - sonar_dist) * 0.3

        # Line sensors modeling (s0 to s4)
        # Position relative à la ligne centrée (-2 to +2)
        line_offset = -cur_steering / 15.0 + random.uniform(-0.1, 0.1)
        # Distribution gaussienne de noirceur autour de la ligne
        s_norm = []
        s_raw = []
        s_corr = []
        weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
        for w in weights:
            dist_to_line = abs(w - line_offset)
            # 1.0 = ligne noire détectée, 0.0 = blanc
            val = math.exp(-0.5 * (dist_to_line / 0.8)**2)
            val = max(0.02, min(0.98, val + random.gauss(0, 0.02)))
            raw = int(val * 3800 + 200)
            corr = val * (1.0 + random.gauss(0, 0.01))
            s_norm.append(round(val, 4))
            s_raw.append(raw)
            s_corr.append(round(corr, 4))

        # Steering CCR (Center ~ 1500us, scale 10us/deg)
        steer_target_ccr = int(1500 + steer_target * 10)
        steer_cur_ccr = int(1500 + cur_steering * 10)

        # Build row
        row = [
            ts_ms, seq,
            fsm_state, req_state, 0x0200, 0,  # FSM flags (RDY)
            round(sonar_raw, 2), round(sonar_dist, 2), 1 if sonar_dist < 40 else 0, 0,
            0x0008,  # Line followed flag
            # S0
            s_raw[0], s_norm[0], s_corr[0], 0,
            # S1
            s_raw[1], s_norm[1], s_corr[1], 0,
            # S2
            s_raw[2], s_norm[2], s_corr[2], 0,
            # S3
            s_raw[3], s_norm[3], s_corr[3], 0,
            # S4
            s_raw[4], s_norm[4], s_corr[4], 0,
            0x0001, 0,  # sensors comp flags/err
            motor_st, 0x0001, 0,  # motors comp flags/err
            # Left motor
            round(target_throttle, 4), round(cur_throttle_left, 4), left_encoder,
            0x05, 0x07, 1.25, 0.08, 0.02,
            # Right motor
            round(target_throttle, 4), round(cur_throttle_right, 4), right_encoder,
            0x05, 0x07, 1.25, 0.08, 0.02,
            # Steering
            round(steer_target, 2), round(cur_steering, 2),
            steer_target_ccr, steer_cur_ccr,
            1.0, 0x0100, 0,
            # Aux servo
            0.0, 0.0, 1500, 1500, 1.0, 0x0100, 0,
            # Services & daemons
            0x000F, int(t * 1000), 0,
            4,
            1, 0,  # Daemon 1 (Running, err 0)
            1, 0,  # Daemon 2
            1, 0,  # Daemon 3
            1, 0   # Daemon 4
        ]
        rows.append(row)

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)

    print(f"✓ Échantillon généré avec succès : {filename} ({len(rows)} lignes, {duration_sec}s à {sample_rate_hz}Hz)")

if __name__ == "__main__":
    generate_sample_csv()
