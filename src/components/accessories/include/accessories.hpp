#pragma once

#include <cstdint>

#include "esp_err.h"
#include "rgb_model.hpp"

namespace accessories {

struct RtcDateTime {
    uint16_t year{2026};
    uint8_t month{1};
    uint8_t day{1};
    uint8_t hour{0};
    uint8_t minute{0};
    uint8_t second{0};
};

struct Snapshot {
    bool bus_ready{false};
    bool fault{false};

    bool remote_enabled{false};
    bool remote_present{false};
    uint8_t oled_address{0};

    bool rtc_enabled{false};
    bool rtc_present{false};
    bool rtc_time_valid{false};
    uint16_t year{0};
    uint8_t month{0};
    uint8_t day{0};
    uint8_t hour{0};
    uint8_t minute{0};
    uint8_t second{0};

    bool als_enabled{false};
    bool als_present{false};
    rgb::AlsSensitivity als_sensitivity{rgb::AlsSensitivity::Normal};
    uint16_t als_raw{0};
    float lux{0.0f};
};

esp_err_t start();
Snapshot snapshot();
esp_err_t set_rtc_time(const RtcDateTime& value);

} // namespace accessories
