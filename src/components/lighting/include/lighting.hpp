#pragma once

#include <cstddef>
#include "esp_err.h"

namespace lighting {

esp_err_t init();
esp_err_t start();
void identify(std::size_t channel, uint32_t duration_ms = 3000);
void stop_identify();
void preview_length(std::size_t channel, std::size_t count);
void stop_length_preview();

} // namespace lighting
