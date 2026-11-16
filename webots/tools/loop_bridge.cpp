// Pont de test minimal Webots <-> loi de suivi de ligne du FSM (sans Docker ni memoire partagee).
//
// Sert a mesurer la fidelite de la simulation Webots : il ecoute sur le port 9000 comme SimBridge, applique a chaque trame
// le VRAI LineFollowerController (services/libs/shared-headers/control/line_follower.hpp) et la vitesse en courbe
// d'Ackermann du MotionProfiler, puis renvoie la commande avec la meme conversion que SimBridge (direction = -braquage/45).
// Ce n'est PAS le FSM complet (pas d'evitement, pas de SpeedPlanner) : uniquement la boucle de suivi de ligne.
//
// Compilation : c++ -std=c++17 -O2 -I services/libs/shared-headers webots/tools/loop_bridge.cpp -o /tmp/loop_bridge
// Usage       : loop_bridge [port] [journal.csv]   (demarre la course a la premiere trame recue)

#include "control/line_follower.hpp"
#include "control/motion_profiler.hpp"
#include "control/picar_constants.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Gains du robot (fsm.conf), utilises aussi en simulation. LOOP_BRIDGE_KP / LOOP_BRIDGE_KD pour en essayer d'autres
// (ex. 0.35 / 0.035 : robustes sur l'ovale, voir sweep_robustness.py, mais trop faibles pour les virages serres).
constexpr float KP = 0.58f;
constexpr float KD = 0.035f;
constexpr float CRUISE = 0.32f;
constexpr float MIN_CURVE = 0.20f;
constexpr float SAFETY = 0.80f;
constexpr float MAX_ACCEL = 0.735f;
constexpr float DT = 0.02f;

bool parse_array(const std::string& s, const char* key, float out[5]) {
    size_t p = s.find(key);
    if (p == std::string::npos) return false;
    p = s.find('[', p);
    if (p == std::string::npos) return false;
    for (int i = 0; i < 5; ++i) {
        char* end = nullptr;
        out[i] = std::strtof(s.c_str() + p + 1, &end);
        p = static_cast<size_t>(end - s.c_str());
        if (i < 4) p = s.find(',', p);
        if (p == std::string::npos) return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : 9000;
    FILE* log = argc > 2 ? std::fopen(argv[2], "w") : nullptr;
    if (log) std::fprintf(log, "t,d0,d1,d2,d3,d4,error_steer_deg,velocity,direction\n");

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || listen(srv, 1) != 0) {
        std::perror("bind/listen");
        return 1;
    }
    std::fprintf(stderr, "[loop_bridge] en ecoute sur %d\n", port);
    int cli = accept(srv, nullptr, nullptr);
    setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    // LOOP_BRIDGE_KP / LOOP_BRIDGE_KD : surcharge des gains de fsm.conf (balayage de robustesse)
    const char* kp_env = std::getenv("LOOP_BRIDGE_KP");
    const char* kd_env = std::getenv("LOOP_BRIDGE_KD");
    const float kp = kp_env ? std::strtof(kp_env, nullptr) : KP;
    const float kd = kd_env ? std::strtof(kd_env, nullptr) : KD;
    std::fprintf(stderr, "[loop_bridge] KP=%.3f KD=%.3f\n", kp, kd);
    LineFollowerController line(kp, kd, CRUISE, MIN_CURVE);
    MotionProfiler profiler(MAX_ACCEL, CRUISE);
    float speed = 0.0f;
    float t = 0.0f;
    const char* gap_env = std::getenv("LOOP_BRIDGE_GAP_FIX");
    const bool gap_fix = gap_env && gap_env[0] == '1';
    std::string buf;
    char chunk[4096];

    for (;;) {
        ssize_t n = recv(cli, chunk, sizeof(chunk), 0);
        if (n <= 0) break;
        buf.append(chunk, static_cast<size_t>(n));
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            std::string msg = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            float webots[5];
            if (!parse_array(msg, "\"line_analog\"", webots)) continue;
            // Meme inversion que SimShmBridge : l'indice 0 du FSM (gauche, poids -2) est l'indice 4 de la trame.
            float dark[5];
            for (int i = 0; i < 5; ++i) dark[i] = webots[4 - i];

            // LOOP_BRIDGE_GAP_FIX=1 : emule le correctif propose pour le FSM ("ligne vue si la noirceur TOTALE >= 0,5").
            // Ruban de 18 mm entre deux capteurs au pas de 19 mm : chacun lit ~0,45 et le FSM croit la ligne perdue.
            // Mettre les valeurs a l'echelle ne change pas le barycentre (sum w.d / sum d) mais rend has_line vrai.
            if (gap_fix) {
                float total = 0.0f, max_d = 0.0f;
                for (float d : dark) { total += d; max_d = std::max(max_d, d); }
                if (max_d < 0.5f && total >= 0.5f) {
                    for (float& d : dark) d *= 0.5f / max_d;
                }
            }

            LineFollowerOutput out = line.update_analog(dark, DT);
            float target = profiler.calculate_curve_speed_ackermann(out.steer_degrees, CRUISE, MIN_CURVE, SAFETY);
            if (out.is_line_lost || out.is_finished) target = 0.0f;
            const float dv = MAX_ACCEL * 0.88f * DT;
            speed += std::clamp(target - speed, -dv, dv);

            // Conversion SimBridge : puissance = v / croisiere x 50 %, puis 50 % = 0,32 m/s.
            const float velocity = speed / CRUISE * picar::constants::PHYSICAL_SPEED_AT_HALF_THROTTLE;
            const float direction = -std::clamp(out.steer_degrees / 45.0f, -1.0f, 1.0f);
            char reply[128];
            std::snprintf(reply, sizeof(reply), "{\"velocity\":%.4f,\"direction\":%.4f,\"brake\":false}\n", velocity,
                          direction);
            send(cli, reply, std::strlen(reply), 0);
            if (log) {
                std::fprintf(log, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f\n", t, dark[0], dark[1], dark[2],
                             dark[3], dark[4], out.steer_degrees, velocity, direction);
            }
            t += DT;
        }
    }
    if (log) std::fclose(log);
    close(cli);
    close(srv);
    return 0;
}
