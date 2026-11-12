#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

namespace services {

/**
 * @brief Résultat complet d'estimation de la position de la ligne optique.
 */
struct LineEstimate {
    float error{0.0f};              // Erreur spatiale barycentrique [-2.0f, +2.0f] (négatif = gauche, positif = droite)
    float normalized_error{0.0f};   // Erreur normalisée [-1.0f, +1.0f]
    float total_darkness{0.0f};     // Somme totale des intensités [0.0f, 5.0f]
    float confidence{0.0f};         // Indice de confiance [0.0f, 1.0f]
    bool  is_valid{false};          // true si la ligne est distinctement perçue
    bool  is_line_lost{true};       // true si aucun contraste de ligne n'est perçu
    bool  is_t_stop{false};         // true si marqueur terminal en T (>= 4 capteurs actifs)
    uint64_t timestamp_us{0};       // Horodatage de la mesure
};

class LineEstimator {
public:
    static constexpr float MIN_TOTAL_DARKNESS = 0.20f;  // Seuil minimal pour considérer la ligne présente
    static constexpr float T_STOP_DARK_THRESHOLD = 0.50f;
    static constexpr float T_STOP_TOTAL_DARKNESS = 3.60f;

    /**
     * @brief Estime la position de la ligne à partir des 5 mesures analogiques continues [0.0, 1.0].
     */
    static LineEstimate estimate_line(const float darkness[5], uint64_t timestamp_us = 0);

    /**
     * @brief Compatibilité ascendante : calcul barycentrique discret (capteurs 0/1).
     */
    static float compute_line_error(const uint8_t sensors[5], float fallback_error = 0.0f);

    /**
     * @brief Compatibilité ascendante : calcul barycentrique analogique (retourne l'erreur [-2.0, 2.0]).
     */
    static float compute_line_error_analog(const float darkness[5], float fallback_error = 0.0f);
};

} // namespace services
