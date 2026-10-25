#pragma once

#include <cstdint>
#include <cstddef>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace shared_i2c {

esp_err_t init();
esp_err_t ensure_started();
bool started();
// Checks both physical I2C lines and enforces fault backoff. An initialized
// hardware controller alone does not prove that the wires are usable.
bool available();
// Changes only after a successful runtime bus reset; accessories can use it
// to restore sensor configuration or redraw the OLED without power cycling.
uint32_t recovery_generation();
i2c_master_bus_handle_t bus();
esp_err_t probe(uint8_t address, int timeout_ms = 50);
esp_err_t add_device(uint8_t address, uint32_t scl_hz, i2c_master_dev_handle_t* out);
esp_err_t remove_device(i2c_master_dev_handle_t device);
// Keep ALL device I/O behind the bus mutex so a reset cannot interrupt
// an OLED / ALS / RTC transaction.
esp_err_t transmit(i2c_master_dev_handle_t device, const uint8_t* data,
                   size_t length, int timeout_ms = 100);
esp_err_t transmit_receive(i2c_master_dev_handle_t device,
                           const uint8_t* write_data, size_t write_length,
                           uint8_t* read_data, size_t read_length, int timeout_ms = 100);
// Run one vendor-library operation under the same mutex and bus-fault
// policy as all OLED / ALS / RTC transfers. The callback MUST NOT call
// shared_i2c::transmit()/probe()/execute_locked() recursively.
using LockedOperation = esp_err_t (*)(void* context);
esp_err_t execute_locked(LockedOperation operation, void* context,
                         const char* operation_name = nullptr);
// Diagnostic/recovery helper for persistent TIMEOUT, never for an ordinary NACK.
// A device physically pulling SCL/SDA low cannot be repaired by bus reset.
esp_err_t recover_after_timeout(const char* requester);

} // namespace shared_i2c
