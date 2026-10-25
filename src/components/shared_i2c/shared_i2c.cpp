#include "shared_i2c.hpp"

#include <atomic>

#include "board_pins.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {
constexpr const char* TAG = "shared_i2c";
// Preserve the most recent PORT1 bench baseline. Restore port 0 only after
// the complete accessory set has passed repeatable bench tests.
constexpr i2c_port_num_t kBusPort = I2C_NUM_1;
constexpr uint32_t kLockWaitMs = 250;
constexpr uint32_t kRecoverySpacingMs = 15000;
constexpr uint32_t kInitialBackoffMs = 1000;
constexpr uint32_t kMaxBackoffMs = 10000;

i2c_master_bus_handle_t s_bus = nullptr;
SemaphoreHandle_t s_mutex = nullptr;
TickType_t s_next_io_tick = 0;
TickType_t s_last_recovery_tick = 0;
uint32_t s_backoff_ms = 0;
uint8_t s_consecutive_errors = 0;
bool s_recovery_attempted = false;
bool s_bus_faulted = false;
// Increments only after a successful hardware bus recovery. Device handles
// remain registered, but peripherals may need their configuration restored.
std::atomic<uint32_t> s_recovery_generation{0};

class Lock {
public:
    Lock() {
        if (s_mutex) locked_ = xSemaphoreTake(s_mutex, pdMS_TO_TICKS(kLockWaitMs)) == pdTRUE;
    }
    ~Lock() {
        if (locked_) xSemaphoreGive(s_mutex);
    }
    bool locked() const { return locked_; }
private:
    bool locked_{false};
};

bool both_lines_high()
{
    return gpio_get_level(board::kRemoteSdaGpio) != 0 &&
           gpio_get_level(board::kRemoteSclGpio) != 0;
}

bool in_backoff(TickType_t now)
{
    // Signed, wrap-safe delta for intervals much shorter than half tick range.
    return s_backoff_ms > 0 &&
           static_cast<int32_t>(now - s_next_io_tick) < 0;
}

void note_bus_fault(const char* operation)
{
    const TickType_t now = xTaskGetTickCount();
    s_backoff_ms = s_backoff_ms == 0 ? kInitialBackoffMs :
                   (s_backoff_ms >= kMaxBackoffMs / 2 ? kMaxBackoffMs : s_backoff_ms * 2);
    s_next_io_tick = now + pdMS_TO_TICKS(s_backoff_ms);
    if (!s_bus_faulted) {
        ESP_LOGW(TAG, "I2C paused: %s SDA=%d SCL=%d; retry after %lu ms",
                 operation ? operation : "bus fault",
                 gpio_get_level(board::kRemoteSdaGpio),
                 gpio_get_level(board::kRemoteSclGpio),
                 static_cast<unsigned long>(s_backoff_ms));
    }
    s_bus_faulted = true;
}

void note_success()
{
    if (s_bus_faulted) {
        ESP_LOGI(TAG, "I2C responsive again (SDA=%d SCL=%d)",
                 gpio_get_level(board::kRemoteSdaGpio),
                 gpio_get_level(board::kRemoteSclGpio));
    }
    s_bus_faulted = false;
    s_consecutive_errors = 0;
    s_backoff_ms = 0;
    s_next_io_tick = 0;
}

esp_err_t preflight()
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    // Once an actual bus fault is latched, only the runtime recovery path may
    // clear it. Otherwise one consumer could race ahead while another resets
    // the I2C hardware after the cooldown expires.
    if (s_bus_faulted || in_backoff(xTaskGetTickCount())) return ESP_ERR_TIMEOUT;
    if (!both_lines_high()) {
        note_bus_fault("lines held low");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void note_transaction_result(esp_err_t err, const char* operation)
{
    if (err == ESP_OK) {
        note_success();
        return;
    }
    // A NACK means that particular accessory is absent. Other devices are
    // allowed to keep using the bus: do NOT globally trip the breaker.
    if (err == ESP_ERR_NOT_FOUND) {
        s_consecutive_errors = 0;
        return;
    }
    if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
        if (!both_lines_high() || ++s_consecutive_errors >= 3) {
            note_bus_fault(operation);
            s_consecutive_errors = 0;
        }
    }
}

// Called only with the shared transaction mutex held. Bus resets must never
// overlap SSD1306 transfers or RTC/ALS transactions.
esp_err_t try_recover_locked(const char* requester, bool respect_backoff)
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    const TickType_t now = xTaskGetTickCount();
    if (respect_backoff && in_backoff(now)) return ESP_ERR_TIMEOUT;
    if (s_recovery_attempted &&
        static_cast<TickType_t>(now - s_last_recovery_tick) < pdMS_TO_TICKS(kRecoverySpacingMs)) {
        return ESP_ERR_TIMEOUT;
    }
    s_recovery_attempted = true;
    s_last_recovery_tick = now;
    // SCL held low prevents an I2C clear sequence. Don't storm the driver.
    if (!gpio_get_level(board::kRemoteSclGpio)) {
        note_bus_fault("cannot recover: SCL held low");
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = i2c_master_bus_reset(s_bus);
    if (err != ESP_OK || !both_lines_high()) {
        note_bus_fault(requester ? requester : "I2C reset failed");
        return err == ESP_OK ? ESP_ERR_TIMEOUT : err;
    }
    note_success();
    [[maybe_unused]] const uint32_t generation = s_recovery_generation.fetch_add(1, std::memory_order_relaxed) + 1;
    ESP_LOGI(TAG, "I2C recovered: generation=%lu SDA=1 SCL=1; reinitializing peripherals",
             static_cast<unsigned long>(generation));
    return ESP_OK;
}

// available() is polled by the accessory task even while devices are offline.
// This is deliberately independent of device transactions; previously all
// clients stopped issuing I2C when unavailable, so recovery never ran.
esp_err_t poll_bus_locked()
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    if (s_bus_faulted) return try_recover_locked("runtime bus recovery", true);
    return preflight();
}
} // namespace

namespace shared_i2c {

esp_err_t init()
{
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
    return s_mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t ensure_started()
{
    esp_err_t err = init();
    if (err != ESP_OK) return err;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    if (s_bus) return ESP_OK;

    i2c_master_bus_config_t cfg{};
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.i2c_port = kBusPort;
    cfg.sda_io_num = board::kRemoteSdaGpio;
    cfg.scl_io_num = board::kRemoteSclGpio;
    cfg.glitch_ignore_cnt = 7;
    // External pull-ups are fitted to the expansion system.
    cfg.flags.enable_internal_pullup = false;

    err = i2c_new_master_bus(&cfg, &s_bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Shared I2C PORT%d ready: SDA GPIO%d SCL GPIO%d",
                 static_cast<int>(kBusPort),
                 static_cast<int>(board::kRemoteSdaGpio),
                 static_cast<int>(board::kRemoteSclGpio));
        // app_core starts the bus, waits 250 ms, then calls available().
        // Don't latch startup backoff for a transient low during power settling.
        // All actual transactions still pass through preflight().
        ESP_LOGI(TAG, "Shared I2C controller initialized; awaiting electrical idle check");
    }
    return err;
}

bool started() { return s_bus != nullptr; }
i2c_master_bus_handle_t bus() { return s_bus; }

bool available()
{
    if (ensure_started() != ESP_OK) return false;
    Lock lock;
    return lock.locked() && poll_bus_locked() == ESP_OK;
}

uint32_t recovery_generation()
{
    return s_recovery_generation.load(std::memory_order_relaxed);
}

esp_err_t probe(uint8_t address, int timeout_ms)
{
    const esp_err_t init_err = ensure_started();
    if (init_err != ESP_OK) {
        if (address == 0x68) ESP_LOGW(TAG, "RTC_DIAG bus init rejected: %s", esp_err_to_name(init_err));
        return init_err;
    }
    Lock lock;
    if (!lock.locked()) {
        if (address == 0x68) ESP_LOGW(TAG, "RTC_DIAG bus mutex timeout before probe");
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t ready = preflight();
    if (ready != ESP_OK) {
        if (address == 0x68) ESP_LOGW(TAG, "RTC_DIAG probe BLOCKED before hardware: %s, SDA=%d SCL=%d",
                                     esp_err_to_name(ready),
                                     gpio_get_level(board::kRemoteSdaGpio),
                                     gpio_get_level(board::kRemoteSclGpio));
        return ready;
    }
    const esp_err_t err = i2c_master_probe(s_bus, address, timeout_ms);
    if (address == 0x68) ESP_LOGW(TAG, "RTC_DIAG i2c_master_probe RETURNED: %s, SDA=%d SCL=%d",
                                 esp_err_to_name(err),
                                 gpio_get_level(board::kRemoteSdaGpio),
                                 gpio_get_level(board::kRemoteSclGpio));
    note_transaction_result(err, "probe timeout");
    return err;
}

esp_err_t add_device(uint8_t address, uint32_t scl_hz, i2c_master_dev_handle_t* out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    *out = nullptr;
    const esp_err_t init_err = ensure_started();
    if (init_err != ESP_OK) return init_err;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    const esp_err_t ready = preflight();
    if (ready != ESP_OK) return ready;

    i2c_device_config_t cfg{};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = address;
    cfg.scl_speed_hz = scl_hz;
    return i2c_master_bus_add_device(s_bus, &cfg, out);
}

esp_err_t remove_device(i2c_master_dev_handle_t device)
{
    if (!device) return ESP_OK;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    return i2c_master_bus_rm_device(device);
}

esp_err_t transmit(i2c_master_dev_handle_t dev, const uint8_t* data,
                   size_t length, int timeout_ms)
{
    if (!dev || !data || !length) return ESP_ERR_INVALID_ARG;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    const esp_err_t ready = preflight();
    if (ready != ESP_OK) return ready;
    const esp_err_t err = i2c_master_transmit(dev, data, length, timeout_ms);
    note_transaction_result(err, "write timeout");
    return err;
}

esp_err_t transmit_receive(i2c_master_dev_handle_t dev,
                           const uint8_t* write_data, size_t write_length,
                           uint8_t* read_data, size_t read_length, int timeout_ms)
{
    if (!dev || !write_data || !write_length || !read_data || !read_length) return ESP_ERR_INVALID_ARG;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    const esp_err_t ready = preflight();
    if (ready != ESP_OK) return ready;
    const esp_err_t err = i2c_master_transmit_receive(dev, write_data, write_length,
                                                       read_data, read_length, timeout_ms);
    note_transaction_result(err, "read timeout");
    return err;
}

esp_err_t execute_locked(LockedOperation operation, void* context,
                         const char* operation_name)
{
    if (!operation) return ESP_ERR_INVALID_ARG;
    const esp_err_t init_err = ensure_started();
    if (init_err != ESP_OK) return init_err;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    const esp_err_t ready = preflight();
    if (ready != ESP_OK) return ready;
    const esp_err_t err = operation(context);
    note_transaction_result(err, operation_name ? operation_name : "external driver");
    return err;
}

esp_err_t recover_after_timeout(const char* requester)
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    Lock lock;
    if (!lock.locked()) return ESP_ERR_TIMEOUT;
    // Explicit errors may request an immediate recovery, but still honor
    // the 15-second minimum interval between physical bus-clear attempts.
    return try_recover_locked(requester, false);
}

} // namespace shared_i2c
