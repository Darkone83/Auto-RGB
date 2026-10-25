#include "app_core.hpp"

#include "accessories.hpp"
#include "ble_control.hpp"
#include "config_store.hpp"
#include "esp_log.h"
#include "lighting.hpp"
#include "led_driver.hpp"
#include "remote_control.hpp"
#include "shared_i2c.hpp"
#include "web_server.hpp"
#include "wifi_ap.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char* TAG = "app";
constexpr uint32_t kI2cStartupSettleMs = 250;
}

namespace app {

esp_err_t start()
{
    ESP_LOGI(TAG, "RGB Controller firmware base starting");

    esp_err_t err = config::init();
    if (err != ESP_OK) return err;

    // Keep SoftAP/WebUI available as the fully local fallback.
    err = wifi_ap::start();
    if (err != ESP_OK) return err;
    err = webui::start();
    if (err != ESP_OK) return err;

    // BLE is an alternate transport. A missing/disabled NimBLE stack must not
    // prevent the local SoftAP controller from operating.
    err = ble_control::start();
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "BLE control unavailable: %s", esp_err_to_name(err));
    }

    // Initialize the physical I2C master BEFORE any accessory/OLED task starts.
    // The 250 ms settle interval permits the shared pull-up network and external
    // accessories to reach an idle state without any address probes or writes.
    // A low line is reported, not assumed to be fixed by the delay.
    const esp_err_t i2c_err = shared_i2c::ensure_started();
    if (i2c_err != ESP_OK) {
        ESP_LOGW(TAG, "Shared I2C master unavailable: %s", esp_err_to_name(i2c_err));
    } else {
        ESP_LOGI(TAG, "Shared I2C master initialized; settling %lu ms before accessory startup",
                 static_cast<unsigned long>(kI2cStartupSettleMs));
        vTaskDelay(pdMS_TO_TICKS(kI2cStartupSettleMs));
        if (shared_i2c::available()) {
            ESP_LOGI(TAG, "Shared I2C lines idle after %lu ms; starting accessory tasks",
                     static_cast<unsigned long>(kI2cStartupSettleMs));
        } else {
            // Do NOT hang the controller when an optional device blocks the bus.
            // shared_i2c will reject transactions and back off until the lines
            // return high; the LED and wireless systems must still start.
            ESP_LOGW(TAG, "Shared I2C not idle after %lu ms; starting accessories with bus fault backoff",
                     static_cast<unsigned long>(kI2cStartupSettleMs));
        }
    }

    err = accessories::start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Accessory manager unavailable: %s", esp_err_to_name(err));
    }

    err = remote_control::start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Remote Control task unavailable: %s", esp_err_to_name(err));
    }

    err = lighting::init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lighting init failed: %s; control interfaces remain available", esp_err_to_name(err));
        ESP_LOGI(TAG, "Ready in control-only mode");
        return ESP_OK;
    }

    err = lighting::start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lighting task failed: %s; control interfaces remain available", esp_err_to_name(err));
        ESP_LOGI(TAG, "Ready in control-only mode");
        return ESP_OK;
    }

    const esp_err_t status_err = led::set_status({0, 40, 0});
    if (status_err != ESP_OK) {
        ESP_LOGW(TAG, "Status LED update failed: %s", esp_err_to_name(status_err));
    }

    ESP_LOGI(TAG, "Ready");
    return ESP_OK;
}

} // namespace app
