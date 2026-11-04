#include "SimProtocolHandler.hpp"
#include <algorithm>

namespace services {

bool SimProtocolHandler::parse_telemetry(const std::string& line, GodotTelemetryIn& telem) {
    if (line.find("distance") != std::string::npos || line.find("line_follower") != std::string::npos) {
        return SimpleJson::parse_godot_telemetry(line, telem);
    }
    return false;
}

std::string SimProtocolHandler::serialize_command(const GodotCommandOut& cmd) {
    return SimpleJson::serialize_command(cmd);
}

bool SimProtocolHandler::handle_text_command(const std::string& line,
                                             std::string& ack_response,
                                             bool& req_emergency,
                                             bool& req_start,
                                             bool& is_config,
                                             std::string& cfg_key,
                                             float& cfg_val) {
    req_emergency = false;
    req_start = false;
    is_config = false;
    ack_response.clear();

    if (line == "trigger_start" || line == "start" || line.find("trigger_start") != std::string::npos) {
        req_start = true;
        ack_response = "{\"status\":\"ok\",\"event\":\"trigger_start\"}\n";
        return true;
    }

    if (line == "emergency_stop" || line == "stop" || line.find("emergency_stop") != std::string::npos) {
        req_emergency = true;
        ack_response = "{\"status\":\"ok\",\"event\":\"emergency_stop\"}\n";
        return true;
    }

    if (line.find('=') != std::string::npos && line.find("line_follower") == std::string::npos && line.find("distance") == std::string::npos) {
        size_t eq = line.find('=');
        cfg_key = line.substr(0, eq);
        std::string val_str = line.substr(eq + 1);
        cfg_key.erase(std::remove_if(cfg_key.begin(), cfg_key.end(), ::isspace), cfg_key.end());
        val_str.erase(std::remove_if(val_str.begin(), val_str.end(), ::isspace), val_str.end());
        try {
            cfg_val = std::stof(val_str);
            is_config = true;
            ack_response = "{\"status\":\"ok\",\"config\":\"" + cfg_key + "\",\"value\":" + val_str + "}\n";
            return true;
        } catch (...) {}
    }

    return false;
}

} // namespace services
