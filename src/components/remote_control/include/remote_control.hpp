#pragma once

#include "esp_err.h"

namespace remote_control {

// Starts the lightweight Remote Control supervisor task. The optional hardware
// is only initialized when the NVS-backed Remote Control setting is enabled.
esp_err_t start();

} // namespace remote_control
