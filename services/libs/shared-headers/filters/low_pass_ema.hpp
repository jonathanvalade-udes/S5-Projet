#pragma once

/**
 * @brief Filtre Passe-Bas Exponentiel (EMA - Exponential Moving Average).
 * Conforme à la Section 7 du PiCar Master Architecture & Engineering Manual.
 * Lisse les vibrations mécaniques et le bruit haute fréquence
 * sans consommer d'allocation mémoire dynamique.
 */
class LowPassEMA {
private:
    float alpha; // Facteur de lissage : 0.0 < alpha <= 1.0 (ex: 0.25 pour un lissage robuste)
    float current_val;
    bool initialized;

public:
    explicit LowPassEMA(float a = 0.25f) 
        : alpha(a), current_val(0.0f), initialized(false) {}

    float filter(float sample) {
        if (!initialized) {
            current_val = sample;
            initialized = true;
        } else {
            current_val = alpha * sample + (1.0f - alpha) * current_val;
        }
        return current_val;
    }

    void reset(float initial_value = 0.0f) {
        if (initial_value != 0.0f) {
            current_val = initial_value;
            initialized = true;
        } else {
            initialized = false;
            current_val = 0.0f;
        }
    }

    float get_value() const {
        return current_val;
    }

    bool is_initialized() const {
        return initialized;
    }

    void set_alpha(float a) {
        alpha = a;
    }
};
