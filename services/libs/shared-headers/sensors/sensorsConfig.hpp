#pragma once

namespace shared::sensors {

struct SensorsConfig {
    float hz;
    float filter_alpha;
    float median_window;
};

} // namespace shared::sensors
