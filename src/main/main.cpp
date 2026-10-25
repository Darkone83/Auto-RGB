#include "app_core.hpp"
#include "esp_log.h"

extern "C" void app_main(void)
{
    const esp_err_t err = app::start();
    if (err != ESP_OK) {
        ESP_LOGE("main", "Startup failed: %s", esp_err_to_name(err));
    }
}
