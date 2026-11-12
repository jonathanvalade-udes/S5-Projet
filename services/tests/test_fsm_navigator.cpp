// Tests du ralentissement a l'approche d'un obstacle (FsmNavigator::limit_approach_speed).
#include <cassert>
#include <cmath>
#include <iostream>

#include "FsmNavigator.hpp"

using services::FsmNavigator;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

int main() {
    FsmNavigator nav;
    // kp, kd, croisiere (m/s), vitesse mini en courbe, acceleration max (m/s^2)
    nav.init(0.75f, 0.2f, 0.45f, 0.22f, 0.735f);

    const float stop_cm = 20.0f;
    const float cruise = 0.45f;

    // Loin de l'obstacle (>= 45 cm) : la consigne n'est pas modifiee
    assert(near(nav.limit_approach_speed(cruise, 100.0f, stop_cm), cruise));
    assert(near(nav.limit_approach_speed(cruise, 45.0f, stop_cm), cruise));

    // Dans la zone de ralentissement : v <= sqrt(2 * a * d)
    // (consigne 1,0 m/s : superieure a la limite, c'est donc la limite qui s'applique)
    const float v35 = nav.limit_approach_speed(1.0f, 35.0f, stop_cm);
    assert(near(v35, std::sqrt(2.0f * 0.735f * 0.15f)));

    // Plus on est pres, plus la vitesse autorisee diminue
    const float v25 = nav.limit_approach_speed(1.0f, 25.0f, stop_cm);
    assert(v25 < v35);
    assert(near(v25, std::sqrt(2.0f * 0.735f * 0.05f)));

    // A 25 cm, la croisiere (0,45 m/s) depasse la limite (~0,27 m/s) : elle est bien reduite
    assert(nav.limit_approach_speed(cruise, 25.0f, stop_cm) < cruise);

    // A la distance d'arret (ou en dessous) : vitesse nulle
    assert(near(nav.limit_approach_speed(cruise, 20.0f, stop_cm), 0.0f));
    assert(near(nav.limit_approach_speed(cruise, 10.0f, stop_cm), 0.0f));

    // Une consigne deja plus lente que la limite n'est jamais augmentee
    assert(near(nav.limit_approach_speed(0.05f, 35.0f, stop_cm), 0.05f));

    // --- Evitement : manoeuvre en 12 etapes (angles, durees et vitesses reglables, cles AVOID_*) ---
    {
        using services::AvoidanceSettings;
        using services::ObstacleAvoidance;
        using Result = ObstacleAvoidance::DeviationResult;
        const float dark[5] = {0, 0, 0, 0, 0};           // aucun ruban : la derniere etape va jusqu'a sa duree maximale
        const float dt = 0.02f;

        // Deroule la manoeuvre (cote droit) : releve, pour chaque etape de mouvement (1,3,5,7,8,10,12), le premier pas, le
        // nombre de pas, le braquage et la vitesse commandes ; retourne le pas d'abandon (SearchLine)
        struct StepLog { int first = -1; int count = 0; float steer = 0.0f; float vel = 0.0f; };
        auto run = [&](const AvoidanceSettings& cfg, StepLog (&log)[13], float side) {
            ObstacleAvoidance av;
            av.set_settings(cfg);
            av.reset(0.0f, 0.2f);
            float steer = 0.0f, vel = 0.0f, v = 0.0f;
            for (int k = 0; k < 3000; ++k) {
                v += (vel * 0.9f - v) * (dt / 0.1f);                 // vitesse reelle : reponse du premier ordre (negative en recul)
                const int step = av.get_step();                      // etape active pendant ce pas
                const auto r = av.update(side, dt, dark, v, 0.2f, steer, vel);
                StepLog& l = log[step];
                if (l.first < 0) l.first = k;
                l.count++;
                l.steer = steer;
                l.vel = vel;
                if (r == Result::SearchLine) return k;
            }
            return -1;
        };

        auto about = [](int value, int expected) { return value >= expected - 1 && value <= expected + 1; };   // +-1 pas (arrondi des flottants)
        const AvoidanceSettings def;
        StepLog base[13];
        const int k_giveup = run(def, base, 1.0f);
        assert(k_giveup > 0);

        // Les 7 segments se suivent dans l'ordre des etapes, avec le braquage et la vitesse prevus (cote droit = +1)
        const int order[7] = {1, 3, 5, 7, 8, 10, 12};
        for (int i = 1; i < 7; ++i) assert(base[order[i]].first == base[order[i - 1]].first + base[order[i - 1]].count);
        assert(near(base[1].steer, 0.0f) && base[1].vel < 0.0f);                       // etape 1 : recul, roues droites
        assert(near(base[3].steer, 45.0f) && near(base[3].vel, 0.16f));                // etapes 2-3 : roues a 45 deg, cote choisi
        assert(near(base[5].steer, 0.0f) && near(base[5].vel, def.diag_speed));        // etapes 4-5 : roues droites, diagonale
        assert(near(base[7].steer, -45.0f));                                           // etapes 6-7 : l'autre cote
        assert(near(base[8].steer, 0.0f));                                             // etape 8 : roues droites
        assert(near(base[10].steer, -45.0f));                                          // etapes 9-10 : vers la ligne
        assert(near(base[12].steer, 0.0f) && near(base[12].vel, def.return_speed));    // etapes 11-12 : roues droites
        // Le cote gauche inverse les braquages
        StepLog left[13];
        run(def, left, -1.0f);
        assert(near(left[3].steer, -45.0f) && near(left[7].steer, 45.0f) && near(left[10].steer, 45.0f));

        // Duree d'une etape = nombre de pas x 20 ms ; changer une cle change seulement cette etape
        auto steps_of = [&](const AvoidanceSettings& cfg, int step) {
            StepLog l[13];
            run(cfg, l, 1.0f);
            return l[step].count;
        };
        const int n3 = steps_of(def, 3), n5 = steps_of(def, 5), n7 = steps_of(def, 7), n8 = steps_of(def, 8), n10 = steps_of(def, 10);
        assert(n3 >= 54 && n3 <= 56 && n5 >= 54 && n5 <= 56 && n7 >= 34 && n7 <= 36 && n8 >= 49 && n8 <= 51 && n10 >= 34 && n10 <= 36);

        AvoidanceSettings c = def;
        c.time1_s = 1.60f;                         // etape 3 plus longue de 0,5 s = 25 pas, les autres ne bougent pas
        assert(about(steps_of(c, 3) - n3, 25) && steps_of(c, 5) == n5 && steps_of(c, 7) == n7);
        c = def; c.diag_time_s = 2.10f;            // etape 5 : +1,0 s = 50 pas
        assert(about(steps_of(c, 5) - n5, 50) && steps_of(c, 3) == n3);
        c = def; c.time2_s = 0.90f;                // etape 7 : +0,2 s = 10 pas
        assert(about(steps_of(c, 7) - n7, 10) && steps_of(c, 10) == n10);
        c = def; c.parallel_time_s = 1.80f;        // etape 8 : +0,8 s = 40 pas
        assert(about(steps_of(c, 8) - n8, 40));
        c = def; c.time3_s = 0.50f;                // etape 10 : -0,2 s = -10 pas
        assert(about(n10 - steps_of(c, 10), 10));
        c = def; c.return_time_s = 2.0f;           // etape 12 : abandon 1,5 s plus tot = 75 pas
        StepLog tmp[13];
        assert(about(k_giveup - run(c, tmp, 1.0f), 75));

        // Angles : chaque virage a le sien
        c = def; c.angle1_deg = 30.0f; c.angle2_deg = 20.0f; c.angle3_deg = 40.0f;
        StepLog ang[13];
        run(c, ang, 1.0f);
        assert(near(ang[3].steer, 30.0f) && near(ang[7].steer, -20.0f) && near(ang[10].steer, -40.0f));

        // Vitesses : etape 5 et etape 12
        c = def; c.diag_speed = 0.22f; c.return_speed = 0.20f;
        StepLog spd[13];
        run(c, spd, 1.0f);
        assert(near(spd[5].vel, 0.22f) && near(spd[12].vel, 0.20f) && near(spd[8].vel, 0.16f));

        // Recul : la distance est mesuree (vitesse x duree) ; plus loin = plus de pas
        c = def; c.reverse_dist_m = 0.40f;
        assert(steps_of(c, 1) > steps_of(def, 1) + 40);

        // Hors bornes : ramene dans l'intervalle (via set_settings). Les bornes sont larges : seulement des valeurs absurdes
        // (duree ou vitesse nulle) ou impossibles (roues a plus de 45 deg) sont corrigees.
        c = def; c.angle1_deg = 99.0f; c.time1_s = 0.0f; c.return_speed = 9.0f; c.reverse_dist_m = 0.0f;
        ObstacleAvoidance bounded;
        bounded.set_settings(c);
        assert(near(bounded.settings().angle1_deg, 45.0f) && near(bounded.settings().time1_s, 0.05f));
        assert(near(bounded.settings().return_speed, 0.60f) && near(bounded.settings().reverse_dist_m, 0.02f));

        // La structure de reglages : 12 cles distinctes, set() borne et refuse les cles inconnues
        AvoidanceSettings st;
        assert(AvoidanceSettings::fields().size() == 12);
        assert(st.set("AVOID_PARALLEL_TIME_S", 99.0f) && near(st.parallel_time_s, 30.0f));
        assert(st.set("AVOID_ANGLE_2_DEG", -5.0f) && near(st.get("AVOID_ANGLE_2_DEG"), 0.0f));
        assert(!st.set("AVOID_INCONNUE", 1.0f));
        // Les anciennes bornes ne bloquent plus : 3,1 s, 10 deg ou 0,40 m/s sont acceptes tels quels
        assert(st.set("AVOID_TIME_1_S", 3.1f) && near(st.time1_s, 3.1f));
        assert(st.set("AVOID_ANGLE_1_DEG", 10.0f) && near(st.angle1_deg, 10.0f));
        assert(st.set("AVOID_DIAG_SPEED", 0.40f) && near(st.diag_speed, 0.40f));
        assert(st.set("AVOID_REVERSE_DIST_M", 0.80f) && near(st.reverse_dist_m, 0.80f));
        assert(near(def.parallel_time_s, 1.0f));      // etape 8 : valeur par defaut plus longue que les autres lignes droites
    }

    // --- Arret : coupure brutale seulement en urgence ; a l'arrivee (Stopped) la puissance descend progressivement ---
    {
        using shared::fsm::MachineStates;
        assert(!FsmNavigator::must_cut_power(MachineStates::Stopped, false));            // arrivee / pause : freinage progressif
        assert(!FsmNavigator::must_cut_power(MachineStates::LineFollowingForward, false));
        assert(!FsmNavigator::must_cut_power(MachineStates::Armed, false));
        assert(FsmNavigator::must_cut_power(MachineStates::Failure, false));             // echec : coupure immediate
        assert(FsmNavigator::must_cut_power(MachineStates::Abort, false));               // arret d'urgence
        assert(FsmNavigator::must_cut_power(MachineStates::LineFollowingForward, true)); // urgence demandee par la navigation

        // Du plein regime a l'arret : la puissance ne baisse jamais de plus de ~2,3 points par cycle de 20 ms
        FsmNavigator stopper;
        stopper.init(0.58f, 0.035f, 0.32f, 0.20f, 0.735f);
        float v = 0.0f;
        for (int i = 0; i < 100; ++i) v = stopper.compute_bounded_velocity(0.32f, 0.02f);    // monte a la croisiere
        assert(near(stopper.compute_motor_power(v, 0.32f), 50.0f));
        float prev_power = 50.0f;
        int steps_to_stop = 0;
        for (int i = 0; i < 200; ++i) {
            v = stopper.compute_bounded_velocity(0.0f, 0.02f);                               // consigne nulle : arrivee
            const float power = stopper.compute_motor_power(v, 0.32f);
            assert(prev_power - power <= 2.4f);                                              // jamais de chute brutale
            prev_power = power;
            if (power <= 0.0f && steps_to_stop == 0) steps_to_stop = i + 1;
        }
        assert(steps_to_stop >= 20 && steps_to_stop <= 24);                                  // ~0,43 s pour s'arreter depuis 0,32 m/s
        assert(near(prev_power, 0.0f));
    }

    // --- Puissance moteur : vitesse / croisiere x puissance a la croisiere (MAX_POWER_PCT) ---
    {
        assert(near(nav.compute_motor_power(0.32f, 0.32f), 50.0f));            // defaut : 50 % a la croisiere
        assert(near(nav.compute_motor_power(0.16f, 0.32f), 25.0f));            // la manoeuvre d'evitement : 25 %
        assert(near(nav.compute_motor_power(0.32f, 0.32f, 60.0f), 60.0f));     // MAX_POWER_PCT = 60
        assert(near(nav.compute_motor_power(0.16f, 0.32f, 60.0f), 30.0f));
        assert(near(nav.compute_motor_power(-0.16f, 0.32f, 60.0f), -30.0f));   // marche arriere
        assert(near(nav.compute_motor_power(0.64f, 0.32f, 75.0f), 100.0f));    // jamais plus de 100 %
        assert(near(nav.compute_motor_power(0.30f, 0.0f), 0.0f));              // croisiere nulle : pas de mouvement
        assert(FsmNavigator::MIN_POWER_PCT < FsmNavigator::REFERENCE_POWER_PCT);
        assert(FsmNavigator::REFERENCE_POWER_PCT < FsmNavigator::MAX_POWER_PCT);
        assert(FsmNavigator::MIN_CRUISE_SPEED > 0.0f);
        assert(near(FsmNavigator::MAX_POWER_PCT, 100.0f));                     // plus de plafond a 75 %
    }

    // --- Approche d'un obstacle : distance de ralentissement, marge avant l'evitement, attente apres l'arret ---
    {
        using shared::fsm::MachineStates;
        using services::ObstacleTuning;

        // Valeurs par defaut = comportement d'avant (45 cm, +5 cm, 0,2 s)
        ObstacleTuning def;
        assert(near(def.slowdown_dist_cm, 45.0f) && near(def.avoid_margin_cm, 5.0f) && near(def.stop_wait_s, 0.2f));

        // set() borne, refuse les cles inconnues, get() relit
        ObstacleTuning t;
        assert(ObstacleTuning::fields().size() == 3);
        assert(t.set("OBSTACLE_SLOWDOWN_DIST_CM", 80.0f) && near(t.get("OBSTACLE_SLOWDOWN_DIST_CM"), 80.0f));
        assert(t.set("OBSTACLE_STOP_WAIT_S", -3.0f) && near(t.stop_wait_s, 0.0f));
        assert(t.set("OBSTACLE_AVOID_MARGIN_CM", 5000.0f) && near(t.avoid_margin_cm, 100.0f));
        assert(!t.set("OBSTACLE_INCONNUE", 1.0f));

        // Distance de ralentissement : a 70 cm, rien par defaut ; avec 80 cm, la vitesse est plafonnee (v = sqrt(2 a d))
        FsmNavigator n;
        n.init(0.75f, 0.2f, 0.45f, 0.22f, 0.735f);
        assert(near(n.limit_approach_speed(1.0f, 70.0f, 20.0f), 1.0f));
        ObstacleTuning far_slowdown;
        far_slowdown.slowdown_dist_cm = 80.0f;
        n.set_obstacle_tuning(far_slowdown);
        assert(near(n.limit_approach_speed(1.0f, 70.0f, 20.0f), std::sqrt(2.0f * 0.735f * 0.50f)));

        // Apres l'arret (Stopped), l'obstacle a 28 cm : avec la marge par defaut (+5 = 25 cm) on reprend la ligne ;
        // avec une marge de 10 cm (30 cm) on evite, apres l'attente choisie (0,5 s = 25 pas de 20 ms)
        const float dark[5] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        auto first_change = [&](FsmNavigator& nav, MachineStates& next) {
            for (int i = 1; i <= 100; ++i) {
                auto out = nav.step(MachineStates::Stopped, 0.02f, 28.0f, dark, false, 1.0f, 20.0f, 0.12f, 0.8f);
                if (out.state_changed) { next = out.next_state; return i; }
            }
            return -1;
        };
        MachineStates next = MachineStates::Init;
        FsmNavigator keep_line;
        keep_line.init(0.75f, 0.2f, 0.45f, 0.22f, 0.735f);
        int steps = first_change(keep_line, next);
        assert(next == MachineStates::LineFollowingForward && steps >= 9 && steps <= 11);       // 0,2 s

        FsmNavigator avoid;
        avoid.init(0.75f, 0.2f, 0.45f, 0.22f, 0.735f);
        ObstacleTuning tuned;
        tuned.avoid_margin_cm = 10.0f;
        tuned.stop_wait_s = 0.5f;
        avoid.set_obstacle_tuning(tuned);
        steps = first_change(avoid, next);
        assert(next == MachineStates::RightDeviation && steps >= 24 && steps <= 26);            // 0,5 s
    }

    std::cout << "test_fsm_navigator: tous les tests passent" << std::endl;
    return 0;
}
