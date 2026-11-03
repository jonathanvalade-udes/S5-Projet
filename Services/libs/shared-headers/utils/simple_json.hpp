#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <algorithm>

struct GodotTelemetryIn {
    float distance = 100.0f;
    uint8_t line_sensors[5] = {0, 0, 1, 0, 0};
    float velocity = 0.0f;
    bool ball_dropped = false;
};

struct GodotCommandOut {
    float velocity = 0.0f;  // m/s [-0.32, +0.32]
    float direction = 0.0f; // [-1.0, +1.0] (direction normalisée pour Godot)
    bool brake = false;
    float distance = -1.0f;
    uint8_t line_sensors[5] = {0, 0, 0, 0, 0};
    bool has_sensors = false;
};

/**
 * @brief Parseur et sérialiseur JSON ultra-léger sans dépendance pour le pont TCP Godot 4.
 */
class SimpleJson {
public:
    static bool parse_godot_telemetry(const std::string& json_str, GodotTelemetryIn& out) {
        if (json_str.empty()) return false;

        // 1. Extraction de "distance"
        size_t dist_pos = json_str.find("\"distance\"");
        if (dist_pos != std::string::npos) {
            size_t colon = json_str.find(':', dist_pos);
            if (colon != std::string::npos) {
                try {
                    out.distance = std::stof(json_str.substr(colon + 1));
                } catch (...) {}
            }
        }

        // 2. Extraction de "velocity"
        size_t vel_pos = json_str.find("\"velocity\"");
        if (vel_pos != std::string::npos) {
            size_t colon = json_str.find(':', vel_pos);
            if (colon != std::string::npos) {
                try {
                    out.velocity = std::stof(json_str.substr(colon + 1));
                } catch (...) {}
            }
        }

        // 3. Extraction de "ball_dropped"
        size_t ball_pos = json_str.find("\"ball_dropped\"");
        if (ball_pos != std::string::npos) {
            size_t colon = json_str.find(':', ball_pos);
            if (colon != std::string::npos) {
                std::string val_part = json_str.substr(colon + 1, 10);
                if (val_part.find("true") != std::string::npos) {
                    out.ball_dropped = true;
                } else if (val_part.find("false") != std::string::npos) {
                    out.ball_dropped = false;
                }
            }
        }

        // 4. Extraction du tableau "line_follower": [x, x, x, x, x]
        size_t lf_pos = json_str.find("\"line_follower\"");
        if (lf_pos != std::string::npos) {
            size_t open_bracket = json_str.find('[', lf_pos);
            size_t close_bracket = json_str.find(']', open_bracket);
            if (open_bracket != std::string::npos && close_bracket != std::string::npos) {
                std::string array_str = json_str.substr(open_bracket + 1, close_bracket - open_bracket - 1);
                std::stringstream ss(array_str);
                std::string item;
                int idx = 0;
                while (std::getline(ss, item, ',') && idx < 5) {
                    // Supprime les espaces
                    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
                    if (!item.empty()) {
                        out.line_sensors[idx++] = static_cast<uint8_t>(std::atoi(item.c_str()) ? 1 : 0);
                    }
                }
            }
        }

        return true;
    }

    static std::string serialize_command(const GodotCommandOut& cmd) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        oss << "{\"velocity\":" << cmd.velocity
            << ",\"direction\":" << cmd.direction
            << ",\"brake\":" << (cmd.brake ? "true" : "false");
        if (cmd.has_sensors) {
            oss << ",\"distance\":" << std::setprecision(1) << cmd.distance;
            oss << ",\"line_follower\":["
                << static_cast<int>(cmd.line_sensors[0]) << ","
                << static_cast<int>(cmd.line_sensors[1]) << ","
                << static_cast<int>(cmd.line_sensors[2]) << ","
                << static_cast<int>(cmd.line_sensors[3]) << ","
                << static_cast<int>(cmd.line_sensors[4]) << "]";
        }
        oss << "}\n";
        return oss.str();
    }
};
