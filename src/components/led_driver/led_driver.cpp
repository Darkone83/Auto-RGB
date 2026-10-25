#include "led_driver.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

#include "board_pins.hpp"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr const char* TAG = "led_driver";
constexpr uint32_t kResolutionHz = 10'000'000;
constexpr std::size_t kLaneCount = 4;
constexpr std::size_t kRmtMemBlockSymbols = 48; // non-DMA fallback
// ESP32-S3 has DMA access on TX channel 3 only, so the stable option is one
// DMA channel switched across 8 outputs, not four simultaneous DMA lanes.
constexpr std::size_t kDmaBufferSymbols = 1024; // 4 KiB DMA ring buffer

struct WsEncoder {
    rmt_encoder_t base{};
    rmt_encoder_handle_t bytes{};
    rmt_encoder_handle_t copy{};
    uint8_t phase{0};
    rmt_symbol_word_t reset{};
};

RMT_ENCODER_FUNC_ATTR
size_t encode_ws(rmt_encoder_t* encoder, rmt_channel_handle_t channel,
                 const void* data, size_t data_size, rmt_encode_state_t* out_state)
{
    auto* self = reinterpret_cast<WsEncoder*>(encoder);
    size_t written = 0;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    rmt_encode_state_t sub = RMT_ENCODING_RESET;

    if (self->phase == 0) {
        written += self->bytes->encode(self->bytes, channel, data, data_size, &sub);
        if (sub & RMT_ENCODING_COMPLETE) self->phase = 1;
        if (sub & RMT_ENCODING_MEM_FULL) {
            *out_state = RMT_ENCODING_MEM_FULL;
            return written;
        }
    }

    sub = RMT_ENCODING_RESET;
    if (self->phase == 1) {
        written += self->copy->encode(self->copy, channel, &self->reset, sizeof(self->reset), &sub);
        if (sub & RMT_ENCODING_COMPLETE) {
            self->phase = 0;
            state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_COMPLETE);
        }
        if (sub & RMT_ENCODING_MEM_FULL) state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_MEM_FULL);
    }

    *out_state = state;
    return written;
}

esp_err_t reset_ws(rmt_encoder_t* encoder)
{
    auto* self = reinterpret_cast<WsEncoder*>(encoder);
    self->phase = 0;
    esp_err_t err = rmt_encoder_reset(self->bytes);
    if (err != ESP_OK) return err;
    return rmt_encoder_reset(self->copy);
}

esp_err_t delete_ws(rmt_encoder_t* encoder)
{
    auto* self = reinterpret_cast<WsEncoder*>(encoder);
    if (self->bytes) rmt_del_encoder(self->bytes);
    if (self->copy) rmt_del_encoder(self->copy);
    std::free(self);
    return ESP_OK;
}

esp_err_t new_ws_encoder(rmt_encoder_handle_t* output)
{
    if (!output) return ESP_ERR_INVALID_ARG;
    auto* self = static_cast<WsEncoder*>(std::calloc(1, sizeof(WsEncoder)));
    if (!self) return ESP_ERR_NO_MEM;

    self->base.encode = encode_ws;
    self->base.reset = reset_ws;
    self->base.del = delete_ws;

    rmt_bytes_encoder_config_t bytes{};
    bytes.bit0.level0 = 1;
    bytes.bit0.duration0 = 3;  // 0.3 us
    bytes.bit0.level1 = 0;
    bytes.bit0.duration1 = 9;  // 0.9 us
    bytes.bit1.level0 = 1;
    bytes.bit1.duration0 = 9;  // 0.9 us
    bytes.bit1.level1 = 0;
    bytes.bit1.duration1 = 3;  // 0.3 us
    bytes.flags.msb_first = 1;

    esp_err_t err = rmt_new_bytes_encoder(&bytes, &self->bytes);
    if (err != ESP_OK) { std::free(self); return err; }

    rmt_copy_encoder_config_t copy{};
    err = rmt_new_copy_encoder(&copy, &self->copy);
    if (err != ESP_OK) {
        rmt_del_encoder(self->bytes);
        std::free(self);
        return err;
    }

    // 80 us reset/latch, split across both halves of one low symbol.
    self->reset.level0 = 0;
    self->reset.duration0 = 400;
    self->reset.level1 = 0;
    self->reset.duration1 = 400;
    *output = &self->base;
    return ESP_OK;
}

std::array<rmt_channel_handle_t, kLaneCount> s_channels{};
std::array<rmt_encoder_handle_t, kLaneCount> s_encoders{};
// RMT reuses four lanes for eight outputs. Keep any detached GPIO driven
// low: a floating 5 V buffer input can look like extra WS281x data edges.
std::array<gpio_num_t, kLaneCount> s_lane_gpio{
    board::kChannelGpios[0], board::kChannelGpios[1],
    board::kChannelGpios[2], board::kChannelGpios[3],
};
std::array<std::array<uint8_t, rgb::kMaxLedsPerChannel * 3>, kLaneCount> s_wire{};
// Physical strips latch their last colors if data transmission simply stops.
// These counts are per physical output (not per shared RMT lane) and change
// only after the transmission has completed successfully.
std::array<std::size_t, rgb::kChannelCount> s_last_visible_count{};
std::array<std::size_t, rgb::kChannelCount> s_last_blackout_count{};
bool s_initialized = false;
std::size_t s_active_lane_count = kLaneCount;
SemaphoreHandle_t s_rmt_mutex = nullptr;

class RmtLock {
public:
    explicit RmtLock(SemaphoreHandle_t mutex) : mutex_(mutex), locked_(mutex && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {}
    ~RmtLock() { if (locked_) xSemaphoreGive(mutex_); }
    bool locked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

// Call with the RMT lane disabled, and only after its preceding transmission
// has completed. The old GPIO is deliberately reconfigured as push-pull LOW
// *after* the RMT matrix has moved to the new pin.
esp_err_t switch_lane_gpio(std::size_t lane, gpio_num_t next)
{
    if (s_lane_gpio[lane] == next) return ESP_OK;
    esp_err_t err = rmt_tx_switch_gpio(s_channels[lane], next, false);
    if (err != ESP_OK) return err;

    const gpio_num_t previous = s_lane_gpio[lane];
    s_lane_gpio[lane] = next;
    err = gpio_set_level(previous, 0);
    if (err != ESP_OK) return err;
    return gpio_set_direction(previous, GPIO_MODE_OUTPUT);
}

void encode_color(const rgb::Color& c, rgb::PixelOrder order, uint8_t* out)
{
    switch (order) {
        case rgb::PixelOrder::RGB: out[0] = c.r; out[1] = c.g; out[2] = c.b; break;
        case rgb::PixelOrder::BRG: out[0] = c.b; out[1] = c.r; out[2] = c.g; break;
        case rgb::PixelOrder::RBG: out[0] = c.r; out[1] = c.b; out[2] = c.g; break;
        case rgb::PixelOrder::GBR: out[0] = c.g; out[1] = c.b; out[2] = c.r; break;
        case rgb::PixelOrder::BGR: out[0] = c.b; out[1] = c.g; out[2] = c.r; break;
        case rgb::PixelOrder::GRB:
        default: out[0] = c.g; out[1] = c.r; out[2] = c.b; break;
    }
}

} // namespace

namespace led {

esp_err_t init()
{
    if (s_initialized) return ESP_OK;

    if (!s_rmt_mutex) {
        s_rmt_mutex = xSemaphoreCreateMutex();
        if (!s_rmt_mutex) return ESP_ERR_NO_MEM;
    }

    // Keep all inactive external buffer DIN inputs at a defined LOW voltage.
    // RMT will claim the currently active pins during channel creation.
    for (std::size_t ch = 0; ch < rgb::kChannelCount; ++ch) {
        const gpio_num_t pin = board::channel_gpio(ch);
        esp_err_t err = gpio_set_level(pin, 0);
        if (err != ESP_OK) return err;
        err = gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        if (err != ESP_OK) return err;
    }

    // Favor one DMA-backed TX channel, dynamically routed over all eight
    // outputs. This avoids interrupt-refill starvation under BLE/Wi-Fi load.
    // If it cannot be allocated, revert to the proven four-lane layout.
    auto create_lanes = [](std::size_t count, bool with_dma) -> esp_err_t {
        for (std::size_t lane = 0; lane < count; ++lane) {
            rmt_tx_channel_config_t tx{};
            tx.clk_src = RMT_CLK_SRC_DEFAULT;
            tx.gpio_num = board::channel_gpio(lane);
            tx.mem_block_symbols = with_dma ? kDmaBufferSymbols : kRmtMemBlockSymbols;
            tx.resolution_hz = kResolutionHz;
            tx.trans_queue_depth = 1;
            tx.intr_priority = with_dma ? 0 : 3;
            tx.flags.with_dma = with_dma;

            esp_err_t err = rmt_new_tx_channel(&tx, &s_channels[lane]);
            if (err != ESP_OK) return err;
            err = new_ws_encoder(&s_encoders[lane]);
            if (err != ESP_OK) return err;
            err = rmt_enable(s_channels[lane]);
            if (err != ESP_OK) return err;
        }
        return ESP_OK;
    };

    esp_err_t err = create_lanes(1, true);
    bool dma_active = err == ESP_OK;
    if (!dma_active) {
        ESP_LOGW(TAG, "RMT DMA unavailable (%s); falling back to four interrupt-refilled lanes",
                 esp_err_to_name(err));
        if (s_channels[0]) {
            (void)rmt_disable(s_channels[0]);
            (void)rmt_del_channel(s_channels[0]);
            s_channels[0] = nullptr;
        }
        if (s_encoders[0]) {
            (void)rmt_del_encoder(s_encoders[0]);
            s_encoders[0] = nullptr;
        }
        err = create_lanes(kLaneCount, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT fallback initialization failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    s_active_lane_count = dma_active ? 1 : kLaneCount;
    s_initialized = true;
    ESP_LOGI(TAG, "Initialized 8-channel banked RMT output using %s (%u lane%s)",
             dma_active ? "DMA" : "interrupt refill",
             static_cast<unsigned>(s_active_lane_count), dma_active ? "" : "s");
    return ESP_OK;
}

esp_err_t transmit(const Frame& frame, const rgb::DeviceConfig& config)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    RmtLock lock(s_rmt_mutex);
    if (!lock.locked()) return ESP_ERR_TIMEOUT;

    rmt_transmit_config_t tx_cfg{};
    tx_cfg.loop_count = 0;

    // DMA mode uses one lane and eight sequential banks. With 200-250 LEDs
    // total, on-wire time remains well below one 33-ms frame. Very large
    // installations may run slower than 30 FPS; never truncate LED data.
    const std::size_t banks = rgb::kChannelCount / s_active_lane_count;
    for (std::size_t bank = 0; bank < banks; ++bank) {
        std::array<bool, kLaneCount> active{};
        std::array<std::size_t, kLaneCount> next_visible_count{};
        std::array<std::size_t, kLaneCount> next_blackout_count{};

        for (std::size_t lane = 0; lane < s_active_lane_count; ++lane) {
            const std::size_t ch_index = bank * s_active_lane_count + lane;
            const auto& ch = config.channels[ch_index];

            esp_err_t err = rmt_disable(s_channels[lane]);
            if (err != ESP_OK) return err;
            err = switch_lane_gpio(lane, board::channel_gpio(ch_index));
            if (err != ESP_OK) return err;
            err = rmt_enable(s_channels[lane]);
            if (err != ESP_OK) return err;

            const std::size_t configured_count = std::min<std::size_t>(ch.led_count, rgb::kMaxLedsPerChannel);
            const std::size_t visible_count = ch.enabled ? configured_count : 0;
            // If a channel is disabled (or shortened), previously lit LEDs
            // still need explicit zero data. Also clear disabled configured
            // outputs once after startup in case a strip retained old data.
            std::size_t send_count = std::max(visible_count, s_last_visible_count[ch_index]);
            if (visible_count == 0) {
                send_count = std::max(send_count, configured_count);
                if (send_count <= s_last_blackout_count[ch_index]) continue;
            }
            if (!send_count) continue;

            for (std::size_t i = 0; i < visible_count; ++i) {
                encode_color(frame[ch_index][i], ch.order, &s_wire[lane][i * 3]);
            }
            // Zero is identical in every pixel order; clear the entire tail.
            if (send_count > visible_count) {
                std::memset(s_wire[lane].data() + visible_count * 3, 0, (send_count - visible_count) * 3);
            }

            err = rmt_transmit(s_channels[lane], s_encoders[lane], s_wire[lane].data(), send_count * 3, &tx_cfg);
            if (err != ESP_OK) return err;
            active[lane] = true;
            next_visible_count[lane] = visible_count;
            next_blackout_count[lane] = visible_count ? 0 : send_count;
        }

        for (std::size_t lane = 0; lane < s_active_lane_count; ++lane) {
            if (!active[lane]) continue;
            const esp_err_t err = rmt_tx_wait_all_done(s_channels[lane], -1);
            if (err != ESP_OK) return err;
            const std::size_t ch_index = bank * s_active_lane_count + lane;
            s_last_visible_count[ch_index] = next_visible_count[lane];
            s_last_blackout_count[ch_index] = next_blackout_count[lane];
        }
    }

    return ESP_OK;
}

esp_err_t set_status(rgb::Color color)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    RmtLock lock(s_rmt_mutex);
    if (!lock.locked()) return ESP_ERR_TIMEOUT;

    // The Waveshare ESP32-S3-Zero onboard WS2812 is on GPIO21 and uses RGB byte order.
    constexpr std::size_t lane = 0;
    uint8_t pixel[3] = { color.r, color.g, color.b };
    rmt_transmit_config_t tx_cfg{};
    tx_cfg.loop_count = 0;

    esp_err_t err = rmt_disable(s_channels[lane]);
    if (err != ESP_OK) return err;
    err = switch_lane_gpio(lane, board::kStatusLedGpio);
    if (err != ESP_OK) return err;
    err = rmt_enable(s_channels[lane]);
    if (err != ESP_OK) return err;
    err = rmt_transmit(s_channels[lane], s_encoders[lane], pixel, sizeof(pixel), &tx_cfg);
    if (err != ESP_OK) return err;
    return rmt_tx_wait_all_done(s_channels[lane], -1);
}

} // namespace led
