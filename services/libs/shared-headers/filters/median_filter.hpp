#pragma once

#include <vector>
#include <algorithm>
#include <cstddef>

/**
 * @brief Filtre médian glissant (Anti-Glitches & Rejet d'aberrations).
 * Conforme à la Section 7 du PiCar Master Architecture & Engineering Manual.
 * Indispensable pour l'échogoniométrie ultrasonore (rejet des réflexions parasites 0 ou 400 cm).
 */
class MedianFilter {
private:
    std::vector<float> window;
    size_t max_size;

public:
    explicit MedianFilter(size_t size = 5) : max_size(size) {
        if (max_size == 0) max_size = 1;
    }

    float update(float val) {
        if (window.size() >= max_size) {
            window.erase(window.begin());
        }
        window.push_back(val);
        std::vector<float> s = window;
        std::sort(s.begin(), s.end());
        return s[s.size() / 2];
    }

    void reset() {
        window.clear();
    }

    size_t size() const {
        return window.size();
    }

    bool is_full() const {
        return window.size() >= max_size;
    }
};
