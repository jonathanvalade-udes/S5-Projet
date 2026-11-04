#include "service_config.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <unistd.h>
#include <sys/stat.h>

namespace shared::services::config {

static inline std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

ServiceConfig::ServiceConfig(const std::string& config_filename) {
    load(config_filename);
}

std::string ServiceConfig::resolve_config_path(const std::string& config_filename) {
    std::vector<std::string> search_paths = {
        "/root/" + config_filename,
        "./" + config_filename,
        "../" + config_filename,
        "../../" + config_filename,
        "/etc/" + config_filename
    };

    for (const auto& path : search_paths) {
        struct stat st{};
        if (stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
            return path;
        }
    }

    // Par défaut, retourner le chemin local
    return "./" + config_filename;
}

bool ServiceConfig::load(const std::string& config_filename) {
    std::string path = resolve_config_path(config_filename);
    return load_from_path(path);
}

bool ServiceConfig::load_from_path(const std::string& filepath) {
    m_active_filepath = filepath;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
            continue;
        }

        size_t eq_pos = trimmed.find('=');
        if (eq_pos != std::string::npos) {
            std::string key = trim(trimmed.substr(0, eq_pos));
            std::string val_str = trim(trimmed.substr(eq_pos + 1));
            try {
                float val = std::stof(val_str);
                m_values[key] = val;
            } catch (...) {
                // Ignore parsing errors for non-numeric or comments
            }
        }
    }

    return true;
}

bool ServiceConfig::has_key(const std::string& key) const {
    return m_values.find(key) != m_values.end();
}

float ServiceConfig::get_float(const std::string& key, float default_val) const {
    auto it = m_values.find(key);
    if (it != m_values.end()) {
        return it->second;
    }
    return default_val;
}

void ServiceConfig::set_value(const std::string& key, float value) {
    m_values[key] = value;
}

bool ServiceConfig::save_key(const std::string& key, float value) {
    m_values[key] = value;
    if (m_active_filepath.empty()) {
        return false;
    }
    return write_key_to_file(m_active_filepath, key, value);
}

bool ServiceConfig::write_key_to_file(const std::string& filepath, const std::string& key, float value) {
    std::vector<std::string> lines;
    bool key_found = false;

    std::ifstream in(filepath);
    if (in.is_open()) {
        std::string line;
        while (std::getline(in, line)) {
            std::string trimmed = trim(line);
            if (!trimmed.empty() && trimmed[0] != '#' && trimmed[0] != ';') {
                size_t eq_pos = trimmed.find('=');
                if (eq_pos != std::string::npos) {
                    std::string line_key = trim(trimmed.substr(0, eq_pos));
                    if (line_key == key) {
                        std::ostringstream oss;
                        oss << key << "=" << value;
                        lines.push_back(oss.str());
                        key_found = true;
                        continue;
                    }
                }
            }
            lines.push_back(line);
        }
        in.close();
    }

    if (!key_found) {
        std::ostringstream oss;
        oss << key << "=" << value;
        lines.push_back(oss.str());
    }

    std::ofstream out(filepath, std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    for (const auto& l : lines) {
        out << l << "\n";
    }
    out.close();
    return true;
}

} // namespace shared::services::config

