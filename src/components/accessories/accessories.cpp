#include "accessories.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "board_pins.hpp"
#include "config_store.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_driver.hpp"
#include "shared_i2c.hpp"

namespace {
constexpr const char* TAG = "accessories";
constexpr uint8_t kRtcAddress = 0x68;
constexpr uint8_t kAlsAddress = 0x10;
constexpr uint8_t kOledAddressA = 0x3C;
constexpr uint8_t kOledAddressB = 0x3D;
constexpr uint32_t kI2cHz = 100000;
constexpr int kTimeoutMs = 100;
constexpr int kPollMs = 500;
constexpr int kProbeRetryMs = 3000;
constexpr int kMaxProbeRetryMs = 60000;
constexpr uint8_t kFailureLimit = 3;

TaskHandle_t s_task = nullptr;
SemaphoreHandle_t s_mutex = nullptr;
SemaphoreHandle_t s_io_mutex = nullptr;
accessories::Snapshot s_status{};
i2c_master_dev_handle_t s_rtc = nullptr;
i2c_master_dev_handle_t s_als = nullptr;
uint8_t s_rtc_failures = 0;
uint8_t s_rtc_probe_timeouts = 0;
uint8_t s_als_failures = 0;
rgb::AlsSensitivity s_als_applied_sensitivity = rgb::AlsSensitivity::Normal;
float s_als_lux_per_count = 0.5376f;
// The VEML7700 can lose configuration on power dips. Reapply it after any
// successful shared-bus recovery, without discarding a valid device handle.
uint32_t s_als_config_generation = 0;
int64_t s_last_rtc_probe_ms = 0;
int64_t s_last_als_probe_ms = 0;
int64_t s_last_remote_probe_ms = 0;
int64_t s_rtc_retry_ms = kProbeRetryMs;
int64_t s_als_retry_ms = kProbeRetryMs;
bool s_led_fault_phase = false;
bool s_last_fault = false;
bool s_have_led_state = false;
int64_t s_last_led_update_ms = 0;

class Lock {
public:
    Lock() { if (s_mutex) locked_ = xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE; }
    ~Lock() { if (locked_) xSemaphoreGive(s_mutex); }
private:
    bool locked_{false};
};

class IoLock {
public:
    IoLock() { if (s_io_mutex) locked_ = xSemaphoreTake(s_io_mutex, portMAX_DELAY) == pdTRUE; }
    ~IoLock() { if (locked_) xSemaphoreGive(s_io_mutex); }
    bool locked() const { return locked_; }
private:
    bool locked_{false};
};

int64_t now_ms() { return esp_timer_get_time() / 1000; }

void increase_retry(int64_t& interval)
{
    interval = std::min<int64_t>(interval * 2, kMaxProbeRetryMs);
}


uint8_t bcd_to_bin(uint8_t v) { return static_cast<uint8_t>(((v >> 4) * 10) + (v & 0x0F)); }
uint8_t bin_to_bcd(uint8_t v) { return static_cast<uint8_t>(((v / 10) << 4) | (v % 10)); }
bool valid_bcd(uint8_t v) { return (v & 0x0F) <= 9 && (v >> 4) <= 9; }

bool leap_year(uint16_t year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

bool valid_date_time(const accessories::RtcDateTime& value)
{
    if (value.year < 2000 || value.year > 2199 || value.month < 1 || value.month > 12 ||
        value.day < 1 || value.hour > 23 || value.minute > 59 || value.second > 59) return false;
    static constexpr uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t max_day = days[value.month - 1];
    if (value.month == 2 && leap_year(value.year)) max_day = 29;
    return value.day <= max_day;
}

uint8_t day_of_week(uint16_t year, uint8_t month, uint8_t day)
{
    // Gregorian calendar, Sunday = 1 ... Saturday = 7. DS3231 only stores
    // the value; it does not use it to advance the calendar.
    static constexpr int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    int y = year;
    if (month < 3) --y;
    const int dow = (y + y/4 - y/100 + y/400 + t[month - 1] + day) % 7;
    return static_cast<uint8_t>(dow + 1);
}

esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, std::size_t len)
{
    return shared_i2c::transmit_receive(dev, &reg, 1, data, len, kTimeoutMs);
}

esp_err_t write_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t value)
{
    const uint8_t data[3] = {reg, static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)};
    return shared_i2c::transmit(dev, data, sizeof(data), kTimeoutMs);
}

esp_err_t read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t& value)
{
    uint8_t data[2]{};
    const esp_err_t err = read_regs(dev, reg, data, sizeof(data));
    if (err == ESP_OK) value = static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
    return err;
}

struct AlsProfile {
    uint16_t config;
    float lux_per_count;
    uint16_t settle_ms;
};

AlsProfile als_profile(rgb::AlsSensitivity sensitivity)
{
    switch (sensitivity) {
        case rgb::AlsSensitivity::Low:
            // Gain 1/8, 50 ms: broad range for a sensor with direct exposure.
            return {0x1200, 1.0752f, 70};
        case rgb::AlsSensitivity::High:
            // Gain 1/4, 200 ms: better dusk/interior resolution.
            return {0x1840, 0.1344f, 220};
        case rgb::AlsSensitivity::VeryHigh:
            // Gain 1, 200 ms: intended for deeply shaded/tinted mounting.
            return {0x0040, 0.0336f, 220};
        case rgb::AlsSensitivity::Normal:
        default:
            // Gain 1/8, 100 ms: Vishay-recommended low-gain starting point.
            return {0x1000, 0.5376f, 120};
    }
}

esp_err_t configure_als(rgb::AlsSensitivity sensitivity)
{
    if (!s_als) return ESP_ERR_INVALID_STATE;
    const AlsProfile profile = als_profile(sensitivity);
    const esp_err_t err = write_reg16(s_als, 0x00, profile.config);
    if (err != ESP_OK) return err;
    s_als_applied_sensitivity = sensitivity;
    s_als_lux_per_count = profile.lux_per_count;
    s_als_config_generation = shared_i2c::recovery_generation();
    vTaskDelay(pdMS_TO_TICKS(profile.settle_ms));
    return ESP_OK;
}

void remove_rtc()
{
    if (s_rtc) {
        (void)shared_i2c::remove_device(s_rtc);
        s_rtc = nullptr;
    }
    s_rtc_failures = 0;
}

void remove_als()
{
    if (s_als) {
        (void)shared_i2c::remove_device(s_als);
        s_als = nullptr;
    }
    s_als_failures = 0;
    s_als_applied_sensitivity = rgb::AlsSensitivity::Normal;
    s_als_lux_per_count = 0.5376f;
    s_als_config_generation = shared_i2c::recovery_generation();
}

esp_err_t start_rtc()
{
    if (s_rtc) return ESP_OK;
    // Register-first detection, as on the last tested diagnostic build:
    // no address-only probe, no write or attempt to repair the RTC here.
    esp_err_t err = shared_i2c::add_device(kRtcAddress, kI2cHz, &s_rtc);
    if (err != ESP_OK) return err;
    uint8_t seconds = 0;
    err = read_regs(s_rtc, 0x00, &seconds, 1);
    if (err != ESP_OK) {
        remove_rtc();
        return err;
    }
    ESP_LOGI(TAG, "RTC at 0x%02X responded to register read", kRtcAddress);
    return ESP_OK;
}

esp_err_t start_als(rgb::AlsSensitivity sensitivity)
{
    if (s_als) return ESP_OK;
    esp_err_t err = shared_i2c::probe(kAlsAddress, 50);
    if (err != ESP_OK) return err;
    err = shared_i2c::add_device(kAlsAddress, kI2cHz, &s_als);
    if (err != ESP_OK) return err;

    err = configure_als(sensitivity);
    if (err != ESP_OK) {
        remove_als();
        return err;
    }
    ESP_LOGI(TAG, "VEML7700 detected at 0x%02X (sensitivity %u)",
             kAlsAddress, static_cast<unsigned>(sensitivity));
    return ESP_OK;
}

esp_err_t read_rtc(accessories::Snapshot& st)
{
    uint8_t data[7]{};
    esp_err_t err = read_regs(s_rtc, 0x00, data, sizeof(data));
    if (err != ESP_OK) return err;

    uint8_t status = 0x80;
    err = read_regs(s_rtc, 0x0F, &status, 1);
    if (err != ESP_OK) return err;

    // EOS and Auto-RGB use the same BCD time block (registers 0..6).
    // A responding address isn't proof that its clock registers are sane.
    const uint8_t second_bcd = data[0] & 0x7F;
    const uint8_t minute_bcd = data[1] & 0x7F;
    const uint8_t hour_bcd = data[2] & ((data[2] & 0x40) ? 0x1F : 0x3F);
    const uint8_t day_bcd = data[4] & 0x3F;
    const uint8_t month_bcd = data[5] & 0x1F;
    const uint8_t year_bcd = data[6];
    const bool bcd_ok = valid_bcd(second_bcd) && valid_bcd(minute_bcd) &&
                        valid_bcd(hour_bcd) && valid_bcd(day_bcd) &&
                        valid_bcd(month_bcd) && valid_bcd(year_bcd) &&
                        (!(data[2] & 0x40) ||
                         (bcd_to_bin(hour_bcd) >= 1 && bcd_to_bin(hour_bcd) <= 12));
    if (!bcd_ok) {
        st.rtc_time_valid = false;
        return ESP_OK; // Device is present, clock values invalid; never write blindly.
    }

    st.second = bcd_to_bin(second_bcd);
    st.minute = bcd_to_bin(minute_bcd);
    if (data[2] & 0x40) {
        uint8_t hour = bcd_to_bin(hour_bcd);
        const bool pm = (data[2] & 0x20) != 0;
        if (hour == 12) hour = 0;
        st.hour = static_cast<uint8_t>(hour + (pm ? 12 : 0));
    } else {
        st.hour = bcd_to_bin(hour_bcd);
    }
    st.day = bcd_to_bin(day_bcd);
    st.month = bcd_to_bin(month_bcd);
    st.year = static_cast<uint16_t>(2000 + bcd_to_bin(year_bcd) + ((data[5] & 0x80) ? 100 : 0));
    const accessories::RtcDateTime time{st.year, st.month, st.day, st.hour, st.minute, st.second};
    st.rtc_time_valid = (status & 0x80) == 0 && valid_date_time(time);
    return ESP_OK;
}

esp_err_t write_rtc_time(const accessories::RtcDateTime& value)
{
    if (!s_rtc) return ESP_ERR_INVALID_STATE;
    if (!valid_date_time(value)) return ESP_ERR_INVALID_ARG;

    const bool century = value.year >= 2100;
    const uint8_t year2 = static_cast<uint8_t>(value.year % 100);
    const std::array<uint8_t, 8> payload{{
        0x00,
        bin_to_bcd(value.second),
        bin_to_bcd(value.minute),
        bin_to_bcd(value.hour),
        bin_to_bcd(day_of_week(value.year, value.month, value.day)),
        bin_to_bcd(value.day),
        static_cast<uint8_t>(bin_to_bcd(value.month) | (century ? 0x80 : 0x00)),
        bin_to_bcd(year2),
    }};
    esp_err_t err = shared_i2c::transmit(s_rtc, payload.data(), payload.size(), kTimeoutMs);
    if (err != ESP_OK) return err;

    uint8_t status = 0;
    err = read_regs(s_rtc, 0x0F, &status, 1);
    if (err != ESP_OK) return err;
    status = static_cast<uint8_t>(status & ~0x80u); // clear oscillator-stop flag
    const uint8_t status_write[2] = {0x0F, status};
    return shared_i2c::transmit(s_rtc, status_write, sizeof(status_write), kTimeoutMs);
}

esp_err_t read_als(accessories::Snapshot& st)
{
    uint16_t raw = 0;
    const esp_err_t err = read_reg16(s_als, 0x04, raw);
    if (err != ESP_OK) return err;
    st.als_raw = raw;
    st.lux = static_cast<float>(raw) * s_als_lux_per_count;
    return ESP_OK;
}

void publish_status(const accessories::Snapshot& st)
{
    Lock lock;
    s_status = st;
}

void update_status_led(bool fault, int64_t now)
{
    // Optional-accessory faults are amber. Core startup/fatal handling remains
    // free to use red elsewhere. Healthy/disabled accessories settle to dim green.
    if (fault) {
        if (now - s_last_led_update_ms < 500) return;
        s_last_led_update_ms = now;
        s_led_fault_phase = !s_led_fault_phase;
        const esp_err_t err = led::set_status(s_led_fault_phase ? rgb::Color{48, 14, 0} : rgb::Color{0, 0, 0});
        if (err == ESP_OK) s_have_led_state = true;
        s_last_fault = true;
        return;
    }

    if (s_have_led_state && !s_last_fault) return;
    if (now - s_last_led_update_ms < 1000) return;
    s_last_led_update_ms = now;
    s_led_fault_phase = false;
    const esp_err_t err = led::set_status({0, 40, 0});
    if (err == ESP_OK) {
        s_have_led_state = true;
        s_last_fault = false;
    }
}

void task(void*)
{
    while (true) {
        const auto settings = config::accessory_settings_snapshot();
        const int64_t now = now_ms();
        accessories::Snapshot st{};
        st.remote_enabled = settings.remote_enabled;
        st.rtc_enabled = settings.rtc_enabled;
        st.als_enabled = settings.als_enabled;
        st.als_sensitivity = settings.als_sensitivity;

        const bool any_enabled = settings.remote_enabled || settings.rtc_enabled || settings.als_enabled;
        if (any_enabled) {
            st.bus_ready = shared_i2c::available();
        }

        {
            IoLock io;
            if (io.locked()) {
                        if (!settings.rtc_enabled) {
                            remove_rtc();
                            s_rtc_retry_ms = kProbeRetryMs;
                        } else if (st.bus_ready) {
                            if (!s_rtc && (s_last_rtc_probe_ms == 0 || now - s_last_rtc_probe_ms >= s_rtc_retry_ms)) {
                                s_last_rtc_probe_ms = now;
                                const esp_err_t probe_err = start_rtc();
                                if (probe_err != ESP_OK) {
                                    if (probe_err == ESP_ERR_TIMEOUT) {
                                        if (++s_rtc_probe_timeouts >= kFailureLimit) {
                                            (void)shared_i2c::recover_after_timeout("RTC probe 0x68");
                                            s_rtc_probe_timeouts = 0;
                                        }
                                    } else {
                                        s_rtc_probe_timeouts = 0;
                                    }
                                    ESP_LOGW(TAG, "RTC 0x68 probe: %s (next retry in %lld ms)",
                                             esp_err_to_name(probe_err),
                                             static_cast<long long>(std::min<int64_t>(s_rtc_retry_ms * 2, kMaxProbeRetryMs)));
                                    increase_retry(s_rtc_retry_ms);
                                    remove_rtc();
                                } else {
                                    s_rtc_probe_timeouts = 0;
                                    s_rtc_retry_ms = kProbeRetryMs;
                                }
                            }
                            if (s_rtc) {
                                st.rtc_present = true;
                                const esp_err_t read_err = read_rtc(st);
                                if (read_err == ESP_OK) {
                                    s_rtc_failures = 0;
                                } else if (++s_rtc_failures >= kFailureLimit) {
                                    ESP_LOGW(TAG, "RTC register read: %s", esp_err_to_name(read_err));
                                    if (read_err == ESP_ERR_TIMEOUT)
                                        (void)shared_i2c::recover_after_timeout("RTC read 0x68");
                                    increase_retry(s_rtc_retry_ms);
                                    remove_rtc();
                                    st.rtc_present = false;
                                    st.rtc_time_valid = false;
                                }
                            }
                        }

                        if (!settings.als_enabled) {
                            remove_als();
                            s_als_retry_ms = kProbeRetryMs;
                        } else if (st.bus_ready) {
                            if (!s_als && (s_last_als_probe_ms == 0 || now - s_last_als_probe_ms >= s_als_retry_ms)) {
                                s_last_als_probe_ms = now;
                                const esp_err_t probe_err = start_als(settings.als_sensitivity);
                                if (probe_err != ESP_OK) {
                                    ESP_LOGW(TAG, "ALS 0x10 probe/configure: %s", esp_err_to_name(probe_err));
                                    if (probe_err == ESP_ERR_TIMEOUT)
                                        (void)shared_i2c::recover_after_timeout("ALS probe 0x10");
                                    increase_retry(s_als_retry_ms);
                                    remove_als();
                                } else {
                                    s_als_retry_ms = kProbeRetryMs;
                                }
                            }
                            if (s_als) {
                                st.als_present = true;
                                if (s_als_applied_sensitivity != settings.als_sensitivity ||
                                    s_als_config_generation != shared_i2c::recovery_generation()) {
                                    ESP_LOGI(TAG, "ALS: restoring configuration after I2C recovery");
                                    if (configure_als(settings.als_sensitivity) != ESP_OK) {
                                        remove_als();
                                        st.als_present = false;
                                    }
                                }
                                const esp_err_t read_err = s_als ? read_als(st) : ESP_ERR_INVALID_STATE;
                                if (s_als && read_err == ESP_OK) {
                                    s_als_failures = 0;
                                } else if (s_als && ++s_als_failures >= kFailureLimit) {
                                    ESP_LOGW(TAG, "ALS register read: %s", esp_err_to_name(read_err));
                                    if (read_err == ESP_ERR_TIMEOUT)
                                        (void)shared_i2c::recover_after_timeout("ALS read 0x10");
                                    increase_retry(s_als_retry_ms);
                                    remove_als();
                                    st.als_present = false;
                                } else if (s_als) {
                                    // Preserve the most recent valid lux on an isolated
                                    // read failure so the LED dimmer never sees false dark.
                                    const auto previous = accessories::snapshot();
                                    st.als_raw = previous.als_raw;
                                    st.lux = previous.lux;
                                }
                            }
                        }

            }
        }

        // Remote Control owns the SSD1306 driver, but the health manager only
        // probes its expected address. When Remote is disabled, no OLED probing occurs.
        if (settings.remote_enabled && st.bus_ready &&
            (s_last_remote_probe_ms == 0 || now - s_last_remote_probe_ms >= 2000)) {
            s_last_remote_probe_ms = now;
            if (shared_i2c::probe(kOledAddressA, 25) == ESP_OK) {
                st.remote_present = true;
                st.oled_address = kOledAddressA;
            } else if (shared_i2c::probe(kOledAddressB, 25) == ESP_OK) {
                st.remote_present = true;
                st.oled_address = kOledAddressB;
            }
        } else if (settings.remote_enabled) {
            const auto prev = accessories::snapshot();
            st.remote_present = prev.remote_present;
            st.oled_address = prev.oled_address;
        }

        st.fault = (settings.remote_enabled && !st.remote_present) ||
                   (settings.rtc_enabled && !st.rtc_present) ||
                   (settings.als_enabled && !st.als_present) ||
                   (any_enabled && !st.bus_ready);

        publish_status(st);
        update_status_led(st.fault, now);
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}

} // namespace

namespace accessories {

esp_err_t start()
{
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) return ESP_ERR_NO_MEM;
    }
    if (!s_io_mutex) {
        s_io_mutex = xSemaphoreCreateMutex();
        if (!s_io_mutex) return ESP_ERR_NO_MEM;
    }
    if (s_task) return ESP_OK;
    return xTaskCreate(task, "accessories", 4096, nullptr, 3, &s_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

Snapshot snapshot()
{
    Lock lock;
    return s_status;
}

esp_err_t set_rtc_time(const RtcDateTime& value)
{
    if (!valid_date_time(value)) return ESP_ERR_INVALID_ARG;
    const Snapshot st = snapshot();
    if (!st.rtc_enabled) return ESP_ERR_INVALID_STATE;
    if (!st.rtc_present) return ESP_ERR_NOT_FOUND;

    IoLock io;
    if (!io.locked()) return ESP_ERR_TIMEOUT;
    const esp_err_t err = write_rtc_time(value);
    if (err != ESP_OK) ESP_LOGW(TAG, "RTC time write failed: %s", esp_err_to_name(err));
    if (err == ESP_OK) ESP_LOGI(TAG, "RTC set to %04u-%02u-%02u %02u:%02u:%02u",
        static_cast<unsigned>(value.year), static_cast<unsigned>(value.month),
        static_cast<unsigned>(value.day), static_cast<unsigned>(value.hour),
        static_cast<unsigned>(value.minute), static_cast<unsigned>(value.second));
    return err;
}

} // namespace accessories
