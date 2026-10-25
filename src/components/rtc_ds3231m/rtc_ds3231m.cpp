#include "rtc_ds3231m.hpp"

#include <array>

#include "esp_log.h"
#include "shared_i2c.hpp"

namespace rtc_ds3231m {
namespace {
constexpr char TAG[] = "rtc_ds3231m";
constexpr uint8_t kAddress = 0x68; // ESP-IDF 7-bit address; on-wire W=0xD0, R=0xD1
constexpr uint8_t kSeconds = 0x00;
constexpr uint8_t kStatus = 0x0F;
constexpr uint8_t kOscillatorStop = 0x80;
constexpr int kTimeoutMs = 250;

uint8_t bcd_to_bin(uint8_t bcd) {
    return static_cast<uint8_t>((bcd >> 4) * 10 + (bcd & 0x0F));
}
uint8_t bin_to_bcd(uint8_t bin) {
    return static_cast<uint8_t>(((bin / 10) << 4) | (bin % 10));
}
bool valid_bcd(uint8_t bcd) {
    return (bcd & 0x0F) <= 9 && (bcd >> 4) <= 9;
}
bool leap_year(uint16_t year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}
bool valid_date_time(const DateTime& v) {
    if (v.year < 2000 || v.year > 2199 || v.month < 1 || v.month > 12 ||
        v.day < 1 || v.hour > 23 || v.minute > 59 || v.second > 59) return false;
    static constexpr uint8_t kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const uint8_t max_day = v.month == 2 && leap_year(v.year) ? 29 : kDays[v.month - 1];
    return v.day <= max_day;
}
uint8_t day_of_week(uint16_t year, uint8_t month, uint8_t day) {
    // DS3231 DOW register stores an ordinary integer from 1..7; not BCD.
    // Sunday=1 .. Saturday=7; DS3231 does not calculate the DOW itself.
    static constexpr int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    int y = year;
    if (month < 3) --y;
    return static_cast<uint8_t>((y + y/4 - y/100 + y/400 + t[month-1] + day) % 7 + 1);
}
} // namespace

esp_err_t Device::begin(uint32_t speed_hz) {
    if (handle_) return ESP_OK;
    return shared_i2c::add_device(kAddress, speed_hz, &handle_);
}

esp_err_t Device::end() {
    if (!handle_) return ESP_OK;
    const esp_err_t err = shared_i2c::remove_device(handle_);
    // Never discard the handle on a failed remove: it may still be attached
    // to the shared bus, and dropping it could leak a live device registration.
    if (err == ESP_OK) handle_ = nullptr;
    return err;
}

esp_err_t Device::read_register(uint8_t reg, uint8_t& value) const {
    if (!handle_) return ESP_ERR_INVALID_STATE;
    uint8_t received = 0xA5;
    const esp_err_t err = shared_i2c::transmit_receive(handle_, &reg, 1, &received, 1, kTimeoutMs);
    if (err == ESP_OK) {
        value = received;
    } else {
        ESP_LOGD(TAG, "read register 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

esp_err_t Device::write_register(uint8_t reg, uint8_t value) const {
    if (!handle_) return ESP_ERR_INVALID_STATE;
    const uint8_t frame[2] = {reg, value};
    const esp_err_t err = shared_i2c::transmit(handle_, frame, sizeof(frame), kTimeoutMs);
    if (err != ESP_OK) ESP_LOGW(TAG, "write register 0x%02X failed: %s", reg, esp_err_to_name(err));
    return err;
}

esp_err_t Device::read_time(DateTime& result) const {
    result = {};
    if (!handle_) return ESP_ERR_INVALID_STATE;
    std::array<uint8_t, 7> b{};
    for (uint8_t r = 0; r < b.size(); ++r) {
        const esp_err_t err = read_register(r, b[r]);
        if (err != ESP_OK) return err;
    }
    uint8_t status = 0;
    const esp_err_t status_err = read_register(kStatus, status);
    if (status_err != ESP_OK) return status_err;

    const uint8_t second = b[0] & 0x7F;
    const uint8_t minute = b[1] & 0x7F;
    const bool mode_12h = (b[2] & 0x40) != 0;
    const uint8_t hour = b[2] & (mode_12h ? 0x1F : 0x3F);
    const uint8_t day = b[4] & 0x3F;
    const uint8_t month = b[5] & 0x1F;
    if (!valid_bcd(second) || !valid_bcd(minute) || !valid_bcd(hour) ||
        !valid_bcd(day) || !valid_bcd(month) || !valid_bcd(b[6])) {
        ESP_LOGW(TAG, "RTC acknowledged but returned invalid BCD");
        return ESP_OK;
    }
    result.second = bcd_to_bin(second);
    result.minute = bcd_to_bin(minute);
    if (mode_12h) {
        uint8_t h = bcd_to_bin(hour);
        if (h < 1 || h > 12) return ESP_OK;
        if (h == 12) h = 0;
        result.hour = static_cast<uint8_t>(h + ((b[2] & 0x20) ? 12 : 0));
    } else {
        result.hour = bcd_to_bin(hour);
    }
    result.day = bcd_to_bin(day);
    result.month = bcd_to_bin(month);
    result.year = static_cast<uint16_t>(2000 + bcd_to_bin(b[6]) + ((b[5] & 0x80) ? 100 : 0));
    result.valid = valid_date_time(result) && ((status & kOscillatorStop) == 0);
    return ESP_OK;
}

esp_err_t Device::set_time(const DateTime& v) const {
    if (!handle_) return ESP_ERR_INVALID_STATE;
    if (!valid_date_time(v)) return ESP_ERR_INVALID_ARG;
    const bool century = v.year >= 2100;
    const std::array<uint8_t, 7> values{{
        bin_to_bcd(v.second),
        bin_to_bcd(v.minute),
        bin_to_bcd(v.hour), // 24-hour mode, bit6 clear
        day_of_week(v.year, v.month, v.day), // raw 1..7, NOT BCD
        bin_to_bcd(v.day),
        static_cast<uint8_t>(bin_to_bcd(v.month) | (century ? 0x80 : 0)),
        bin_to_bcd(static_cast<uint8_t>(v.year % 100))
    }};
    for (uint8_t r = 0; r < values.size(); ++r) {
        const esp_err_t err = write_register(r, values[r]);
        if (err != ESP_OK) return err;
    }
    // Clear OSF only after all time registers succeeded, preserving other
    // status bits. Reads/writes are never attempted during detection.
    uint8_t status = 0;
    esp_err_t err = read_register(kStatus, status);
    if (err != ESP_OK) return err;
    status = static_cast<uint8_t>(status & ~kOscillatorStop);
    return write_register(kStatus, status);
}

} // namespace rtc_ds3231m
