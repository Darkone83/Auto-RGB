#pragma once

#include <cstdint>
#include "driver/i2c_master.h"
#include "esp_err.h"

// Minimal Auto-RGB DS3231M RTC driver.
// Owns only an I2C device handle; shared_i2c owns the ESP32 I2C bus and locks.
// Ported from the EOS/PrometheOS byte-register model, using ESP-IDF 7-bit
// address 0x68 instead of Xbox SMBus HAL's shifted address 0xD0.
namespace rtc_ds3231m {

struct DateTime {
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    // false if uninitialized/invalid BCD or DS3231M oscillator-stop flag set.
    bool valid = false;
};

class Device {
public:
    Device() = default;
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    // Registers a device at 7-bit address 0x68 on the existing shared bus.
    // Registration does NOT indicate that the RTC has acknowledged.
    esp_err_t begin(uint32_t speed_hz = 100000);
    esp_err_t end();
    bool registered() const { return handle_ != nullptr; }

    // SMBus byte-data style: START, address W, reg, repeated START,
    // address R, one data byte, master NACK, STOP.
    esp_err_t read_register(uint8_t reg, uint8_t& value) const;
    // START, address W, reg, value, STOP. No writes during device detection.
    esp_err_t write_register(uint8_t reg, uint8_t value) const;

    // Read each time register individually, including status/OSF at 0x0F.
    // ESP_OK with valid=false means device answered but clock is unset/invalid.
    esp_err_t read_time(DateTime& value) const;
    // User-initiated only. Validate date, write individual time registers,
    // then clear OSF only after all time registers were written successfully.
    esp_err_t set_time(const DateTime& value) const;

private:
    i2c_master_dev_handle_t handle_ = nullptr;
};

} // namespace rtc_ds3231m
