#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace shared::services::config {

class ServiceConfig {
public:
    ServiceConfig() = default;
    explicit ServiceConfig(const std::string& config_filename);

    static std::string resolve_config_path(const std::string& config_filename);
    bool load(const std::string& config_filename);
    bool load_from_path(const std::string& filepath);

    bool save_key(const std::string& key, float value);
    static bool write_key_to_file(const std::string& filepath, const std::string& key, float value);

    bool has_key(const std::string& key) const;
    float get_float(const std::string& key, float default_val = 0.0f) const;
    void set_value(const std::string& key, float value);

    const std::string& get_active_filepath() const { return m_active_filepath; }
    const std::unordered_map<std::string, float>& get_all() const { return m_values; }

private:
    std::string m_active_filepath;
    std::unordered_map<std::string, float> m_values;
};

} // namespace shared::services::config

