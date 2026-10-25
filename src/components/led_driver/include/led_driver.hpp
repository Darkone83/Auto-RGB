#pragma once

#include <array>
#include <cstddef>
#include "esp_err.h"
#include "rgb_model.hpp"

namespace led {

using Frame = std::array<std::array<rgb::Color, rgb::kMaxLedsPerChannel>, rgb::kChannelCount>;

esp_err_t init();
esp_err_t transmit(const Frame& frame, const rgb::DeviceConfig& config);
esp_err_t set_status(rgb::Color color);

} // namespace led
