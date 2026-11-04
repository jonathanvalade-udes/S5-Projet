#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <unistd.h>

#include "picar_constants.hpp"
#include "ipc/shared_memory.hpp"

namespace config {

struct PicarConfig {
    float max_acceleration      = picar::constants::DEFAULT_MAX_ACCELERATION;
    float cruise_speed          = picar::constants::DEFAULT_STRAIGHT_CRUISE_SPEED;
    float min_curve_speed       = picar::constants::DEFAULT_MIN_CURVE_SPEED;
    float curve_safety_factor   = picar::constants::DEFAULT_CURVE_SAFETY_FACTOR;
    float predictive_brake_gain = picar::constants::DEFAULT_PREDICTIVE_BRAKE_GAIN;
    float line_kp               = picar::constants::DEFAULT_LINE_KP;
    float line_kd               = picar::constants::DEFAULT_LINE_KD;
    float obstacle_stop_dist_cm = picar::constants::DEFAULT_OBSTACLE_STOP_DIST_CM;
    float reverse_speed         = picar::constants::DEFAULT_REVERSE_SPEED;
    float auto_start_delay      = 2.0f;

    static std::string resolve_config_path(const std::string& custom = "") {
        if (!custom.empty()) return custom;
        if (access("/root/picar.conf", R_OK) == 0 || access("/root", W_OK) == 0) {
            return "/root/picar.conf";
        }
        if (access("picar.conf", R_OK) == 0) {
            return "picar.conf";
        }
        if (access("../picar.conf", R_OK) == 0) {
            return "../picar.conf";
        }
        return "picar.conf";
    }

    /**
     * @brief Lit la configuration depuis le fichier .config (ex: picar.conf).
     * Si le fichier ou une valeur est absent (null), prend la valeur de secours du .hpp !
     */
    static PicarConfig read_config(const std::string& path = "") {
        PicarConfig cfg; // Initialisé d'office avec les constantes de picar_constants.hpp !

        std::string resolved = resolve_config_path(path);
        if (access(resolved.c_str(), R_OK) != 0) {
            std::cout << "[CONFIG] Fichier .config non trouvé (" << resolved 
                      << "). Fallback sur les constantes du .hpp (picar_constants.hpp)." << std::endl;
            return cfg;
        }

        std::ifstream file(resolved);
        if (!file.is_open()) {
            std::cerr << "[CONFIG] Erreur à l'ouverture de " << resolved 
                      << ". Fallback sur les constantes du .hpp." << std::endl;
            return cfg;
        }

        std::cout << "[CONFIG] Lecture de " << resolved << " (avec fallback .hpp)..." << std::endl;
        std::string line;
        while (std::getline(file, line)) {
            size_t c_pos = line.find('#');
            if (c_pos != std::string::npos) line = line.substr(0, c_pos);
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
            if (line.empty()) continue;

            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;

            std::string key = line.substr(0, eq);
            std::string val = line.substr(eq + 1);
            if (val.empty()) continue;

            try {
                float fval = std::stof(val);
                if (key == "MAX_ACCELERATION") cfg.max_acceleration = std::max(0.1f, std::min(fval, 1.5f));
                else if (key == "STRAIGHT_CRUISE_SPEED" || key == "CRUISE_SPEED") cfg.cruise_speed = std::max(0.05f, std::min(fval, 0.60f));
                else if (key == "MIN_CURVE_SPEED") cfg.min_curve_speed = std::max(0.05f, std::min(fval, 0.40f));
                else if (key == "CURVE_SAFETY_FACTOR") cfg.curve_safety_factor = std::max(0.30f, std::min(fval, 1.20f));
                else if (key == "PREDICTIVE_BRAKE_GAIN") cfg.predictive_brake_gain = std::max(0.0f, std::min(fval, 0.30f));
                else if (key == "LINE_KP" || key == "KP") cfg.line_kp = std::max(0.0f, std::min(fval, 2.0f));
                else if (key == "LINE_KD" || key == "KD") cfg.line_kd = std::max(0.0f, std::min(fval, 1.0f));
                else if (key == "OBSTACLE_STOP_DIST_CM") cfg.obstacle_stop_dist_cm = std::max(10.0f, std::min(fval, 80.0f));
                else if (key == "REVERSE_SPEED") cfg.reverse_speed = std::max(0.05f, std::min(fval, 0.30f));
                else if (key == "AUTO_START_DELAY") cfg.auto_start_delay = std::max(0.0f, std::min(fval, 60.0f));
            } catch (...) {}
        }

        return cfg;
    }

    /**
     * @brief Écrit ou met à jour une clé dans le fichier .config (picar.conf).
     */
    static bool write_config(const std::string& param_name, float value, const std::string& path = "") {
        std::string resolved = resolve_config_path(path);

        // Lecture des lignes existantes
        std::vector<std::string> lines;
        bool found = false;
        std::ifstream infile(resolved);
        if (infile.is_open()) {
            std::string l;
            while (std::getline(infile, l)) {
                std::string stripped = l;
                size_t c_pos = stripped.find('#');
                if (c_pos != std::string::npos) stripped = stripped.substr(0, c_pos);
                stripped.erase(std::remove_if(stripped.begin(), stripped.end(), ::isspace), stripped.end());
                
                size_t eq = stripped.find('=');
                if (eq != std::string::npos) {
                    std::string key = stripped.substr(0, eq);
                    if (key == param_name) {
                        std::ostringstream oss;
                        oss << param_name << "=" << value;
                        lines.push_back(oss.str());
                        found = true;
                        continue;
                    }
                }
                lines.push_back(l);
            }
            infile.close();
        }

        if (!found) {
            std::ostringstream oss;
            oss << param_name << "=" << value;
            lines.push_back(oss.str());
        }

        std::ofstream outfile(resolved);
        if (!outfile.is_open()) {
            std::cerr << "[CONFIG] Erreur: impossible d'écrire dans " << resolved << std::endl;
            return false;
        }

        for (const auto& l : lines) {
            outfile << l << "\n";
        }
        std::cout << "[CONFIG] Écriture réussie : " << param_name << "=" << value << " dans " << resolved << std::endl;
        return true;
    }

    /**
     * @brief Sauvegarde complète de la structure dans le .config.
     */
    static bool save_all(const PicarConfig& cfg, const std::string& path = "") {
        std::string resolved = resolve_config_path(path);
        std::ofstream file(resolved);
        if (!file.is_open()) return false;

        file << "# ==========================================\n";
        file << "# CONFIGURATION FASTDRINK - PICAR-S\n";
        file << "# ==========================================\n\n";
        file << "MAX_ACCELERATION=" << cfg.max_acceleration << "\n";
        file << "STRAIGHT_CRUISE_SPEED=" << cfg.cruise_speed << "\n";
        file << "MIN_CURVE_SPEED=" << cfg.min_curve_speed << "\n";
        file << "CURVE_SAFETY_FACTOR=" << cfg.curve_safety_factor << "\n";
        file << "PREDICTIVE_BRAKE_GAIN=" << cfg.predictive_brake_gain << "\n";
        file << "LINE_KP=" << cfg.line_kp << "\n";
        file << "LINE_KD=" << cfg.line_kd << "\n";
        file << "OBSTACLE_STOP_DIST_CM=" << cfg.obstacle_stop_dist_cm << "\n";
        file << "REVERSE_SPEED=" << cfg.reverse_speed << "\n";
        return true;
    }

    /**
     * @brief Applique la configuration dans la mémoire partagée POSIX.
     */
    static void apply_to_shm(const PicarConfig& cfg, RobotTelemetry* shm) {
        if (!shm) return;
        pthread_mutex_lock(&(shm->mtx));
        shm->cfg_max_acceleration = cfg.max_acceleration;
        shm->cfg_cruise_speed     = cfg.cruise_speed;
        shm->cfg_min_curve_speed  = cfg.min_curve_speed;
        shm->cfg_curve_safety_factor = cfg.curve_safety_factor;
        shm->cfg_predictive_brake_gain = cfg.predictive_brake_gain;
        shm->cfg_line_kp          = cfg.line_kp;
        shm->cfg_line_kd          = cfg.line_kd;
        shm->cfg_version++;
        pthread_mutex_unlock(&(shm->mtx));
    }
};

} // namespace config
