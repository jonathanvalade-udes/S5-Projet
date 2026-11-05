#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <cstdint>
#include <atomic>
#include "car/full_packet.hpp"

namespace telemetry {

class CsvLogger {
public:
    CsvLogger();
    ~CsvLogger();

    // Ouvre le fichier CSV et écrit l'en-tête s'il est vide/nouveau
    bool open(const std::string& filepath);

    // Écrit une ligne avec l'ensemble des métriques de CarTelemetry
    void log_record(uint16_t seq, const shared::car::CarTelemetry& telem);

    // Force l'écriture sur le disque
    void flush();

    // Ferme proprement le fichier
    void close();

    bool is_open() const { return m_is_open.load(); }
    std::string get_filepath() const { return m_filepath; }
    size_t get_record_count() const { return m_record_count.load(); }

private:
    void write_header();

    std::string m_filepath;
    std::ofstream m_file;
    mutable std::mutex m_mutex;
    std::atomic<bool> m_is_open{false};
    std::atomic<size_t> m_record_count{0};
    size_t m_unflushed_count{0};
};

} // namespace telemetry

