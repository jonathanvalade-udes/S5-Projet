#pragma once

#include <memory>
#include "fsm/machine_component.hpp"
#include "control/line_follower.hpp"
#include "control/motion_profiler.hpp"
#include "control/picar_constants.hpp"
#include "ObstacleAvoidance.hpp"
#include "VehicleModel.hpp"
#include "SpeedPlanner.hpp"
#include "ObstacleTuning.hpp"

namespace services {

struct FsmNavOutput {
    float desired_velocity{0.0f};
    float desired_steering{0.0f};
    bool req_emergency{false};
    bool state_changed{false};
    shared::fsm::MachineStates next_state{shared::fsm::MachineStates::Init};
};

class FsmNavigator {
public:
    FsmNavigator();
    ~FsmNavigator() = default;

    void init(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel);
    void update_controllers(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel);

    FsmNavOutput step(shared::fsm::MachineStates current_state, float dt,
                      float current_dist_cm, const float line_darkness[5],
                      bool start_triggered, float avoid_side, float obstacle_stop_dist_cm,
                      float reverse_speed, float curve_safety_factor);

    /// Reglages de l'evitement d'obstacle (12 etapes : angles, durees, vitesses), lus dans fsm.conf.
    void set_avoidance(const AvoidanceSettings& settings) { m_obstacle_avoidance.set_settings(settings); }

    /// Reglages de l'approche d'un obstacle : distance de ralentissement, marge avant l'evitement, attente apres l'arret.
    void set_obstacle_tuning(const ObstacleTuning& tuning) { m_tuning = tuning; }

    void reset_controllers();
    float get_current_speed() const;
    /// Rampe de vitesse couplee (SpeedPlanner) : a_longitudinale ET a_laterale (virage) bornees ensemble sous le seuil bille.
    /// steer_deg : braquage commande ; avoiding : pendant l'evitement la puissance de reference est utilisee (pas de compensation).
    float compute_bounded_velocity(float desired_v, float dt, float steer_deg = 0.0f, bool avoiding = false);

    /// Puissance moteur (%) a la croisiere. Au-dessus de REFERENCE_POWER_PCT la voiture va plus vite pour la meme consigne en m/s
    /// du FSM : l'acceleration reelle est multipliee d'autant, donc la rampe est reduite dans le meme rapport.
    void set_power_pct(float max_power_pct);
    /// Defaillance materielle (capteurs muets, urgence SHM) : coupure immediate de la puissance. Sinon (ligne perdue, evitement
    /// echoue) l'etat Failure freine en rampe pour ne pas faire tomber la bille.
    void set_hard_failure(bool hard) { m_hard_failure = hard; }
    /// Puissance moteur (%) pour une vitesse donnee : vitesse / croisiere x power_at_cruise_pct.
    float compute_motor_power(float bounded_v, float cruise_speed, float power_at_cruise_pct = REFERENCE_POWER_PCT);

    /// Vrai si la puissance moteur doit etre coupee d'un coup : urgence demandee ou Abort (une Failure dure demande l'urgence). L'etat Stopped n'en fait PAS
    /// partie : a l'arrivee (ou en pause) le profil de vitesse freine progressivement (acceleration maximale), ce qui evite de
    /// faire tomber la bille. Une coupure brusque a pleine vitesse la ferait tomber.
    static bool must_cut_power(shared::fsm::MachineStates state, bool req_emergency);

    /// Puissance (%) a la vitesse de croisiere. 50 = reference validee (environ 0,32 m/s) sur laquelle l'evitement est calibre.
    static constexpr float REFERENCE_POWER_PCT = 50.0f;
    /// Bornes de MAX_POWER_PCT (puissance a la croisiere) et de STRAIGHT_CRUISE_SPEED : seulement des valeurs absurdes
    /// sont refusees (un moteur ne tourne pas sous quelques % de puissance, et la puissance ne depasse pas 100 %).
    static constexpr float MIN_POWER_PCT = 5.0f;
    static constexpr float MAX_POWER_PCT = 100.0f;
    static constexpr float MIN_CRUISE_SPEED = 0.05f;
    static constexpr float MAX_CRUISE_SPEED = 2.0f;
    /// Marge de securite : la rampe vise 88 % du seuil bille (0,735 -> ~0,65 m/s^2) pour absorber jitter, latence et bruit.
    static constexpr float BALL_ACCEL_MARGIN = 0.88f;

    /// Plafonne la vitesse pour pouvoir s'arreter a obstacle_stop_dist_cm sans depasser a_max (v = sqrt(2*a*d)).
    float limit_approach_speed(float desired_v, float current_dist_cm, float obstacle_stop_dist_cm) const;

    VehicleModel& get_vehicle_model() { return m_vehicle_model; }
    const VehicleModel& get_vehicle_model() const { return m_vehicle_model; }
    SpeedPlanner& get_speed_planner() { return m_speed_planner; }
    const SpeedPlanner& get_speed_planner() const { return m_speed_planner; }

private:
    std::unique_ptr<LineFollowerController> m_line_controller;
    std::unique_ptr<MotionProfiler> m_motion_profiler;
    ObstacleAvoidance m_obstacle_avoidance;
    VehicleModel m_vehicle_model;
    SpeedPlanner m_speed_planner;
    ObstacleTuning m_tuning;

    void apply_planner_limits(float accel_scale);

    float m_max_accel{0.735f};
    float m_power_accel_scale{1.0f};
    bool m_hard_failure{false};
    float m_cruise_speed{0.35f};
    float m_min_curve_speed{0.17f};
    float m_last_steer_degrees{0.0f};
    float m_state_timer{0.0f};
    float m_state_distance{0.0f};
    float m_line_lost_timer{0.0f};
    bool m_course_finished{false};
};

} // namespace services
