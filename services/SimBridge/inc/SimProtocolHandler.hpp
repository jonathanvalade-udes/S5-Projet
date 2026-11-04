#pragma once

#include <string>
#include "utils/simple_json.hpp"

namespace services {

class SimProtocolHandler {
public:
    static bool parse_telemetry(const std::string& line, GodotTelemetryIn& telem);
    static std::string serialize_command(const GodotCommandOut& cmd);
    static bool handle_text_command(const std::string& line,
                                   std::string& ack_response,
                                   bool& req_emergency,
                                   bool& req_start,
                                   bool& is_config,
                                   std::string& cfg_key,
                                   float& cfg_val);
};

} // namespace services
