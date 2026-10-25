#pragma once

#include <cstdint>
#include "esp_err.h"

namespace ble_control {

constexpr uint8_t kProtocolVersion = 1;

// Start the Auto-RGB NimBLE peripheral. If NimBLE is not enabled in sdkconfig,
// returns ESP_ERR_NOT_SUPPORTED without affecting the rest of Auto-RGB.
esp_err_t start();

bool connected();

} // namespace ble_control
