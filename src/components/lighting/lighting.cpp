#include "lighting.hpp"

#include <algorithm>
#include <array>
#include <cstring>

#include "config_store.hpp"
#include "accessories.hpp"
#include "effects.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_driver.hpp"
#include "rgb_model.hpp"

namespace {
constexpr const char* TAG = "lighting";
constexpr uint32_t kFrameMs = 33; // ~30 FPS initial target

led::Frame s_frame{};
std::array<rgb::Color, rgb::kChannelCount * rgb::kMaxLedsPerChannel> s_scratch{};
TaskHandle_t s_task = nullptr;
rgb::DeviceConfig s_render_config{};
volatile int s_identify_channel = -1;
volatile int64_t s_identify_until_us = 0;
volatile int s_preview_channel = -1;
volatile std::size_t s_preview_count = 0;

bool s_power_initialized = false;
bool s_last_power_enabled = false;
uint64_t s_power_fade_start_ms = 0;
uint16_t s_power_fade_duration_ms = 0;
uint8_t s_power_envelope = 0;
// Auto-dim state lives only in the lighting task and never touches NVS.
float s_ambient_filtered_lux = 0.0f;
float s_ambient_factor = 1.0f;
bool s_ambient_initialized = false;

uint8_t scale8(uint8_t value, uint8_t scale)
{
    return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale + 127) / 255);
}

rgb::Color scaled(rgb::Color c, uint8_t amount)
{
    return {scale8(c.r, amount), scale8(c.g, amount), scale8(c.b, amount)};
}

std::size_t segment_length(const rgb::DeviceConfig& cfg, const rgb::SegmentConfig& seg)
{
    if (seg.channel >= rgb::kChannelCount) return 0;
    const auto& ch = cfg.channels[seg.channel];
    if (!ch.enabled || seg.start >= ch.led_count) return 0;
    const std::size_t available = ch.led_count - seg.start;
    return seg.length == 0 ? available : std::min<std::size_t>(seg.length, available);
}

std::size_t zone_length(const rgb::DeviceConfig& cfg, const rgb::ZoneConfig& zone)
{
    std::size_t total = 0;
    for (std::size_t i = 0; i < zone.segment_count; ++i) total += segment_length(cfg, zone.segments[i]);
    return std::min<std::size_t>(total, s_scratch.size());
}

void map_zone(const rgb::DeviceConfig& cfg, const rgb::ZoneConfig& zone,
              const rgb::Color* pixels, std::size_t pixel_count)
{
    std::size_t source = 0;
    for (std::size_t s = 0; s < zone.segment_count && source < pixel_count; ++s) {
        const auto& seg = zone.segments[s];
        if (seg.channel >= rgb::kChannelCount) continue;
        const auto& ch = cfg.channels[seg.channel];
        const std::size_t len = segment_length(cfg, seg);

        for (std::size_t i = 0; i < len && source < pixel_count; ++i, ++source) {
            std::size_t logical = seg.reversed ? (len - 1 - i) : i;
            std::size_t physical = seg.start + logical;
            if (ch.reversed) physical = ch.led_count - 1 - physical;
            if (physical < rgb::kMaxLedsPerChannel) s_frame[seg.channel][physical] = pixels[source];
        }
    }
}

uint8_t update_power_envelope(const rgb::DeviceConfig& cfg, uint64_t now_ms)
{
    const bool enabled = cfg.power.master_enabled;

    if (!s_power_initialized) {
        s_power_initialized = true;
        s_last_power_enabled = enabled;
        s_power_fade_start_ms = now_ms;
        s_power_fade_duration_ms = cfg.power.power_on_fade_ms;
        s_power_envelope = enabled && s_power_fade_duration_ms == 0 ? 255 : 0;
    } else if (enabled != s_last_power_enabled) {
        s_last_power_enabled = enabled;
        if (!enabled) {
            // Master Off is deliberately immediate. The controller and WebUI stay alive.
            s_power_envelope = 0;
        } else {
            s_power_fade_start_ms = now_ms;
            s_power_fade_duration_ms = cfg.power.power_on_fade_ms;
            s_power_envelope = s_power_fade_duration_ms == 0 ? 255 : 0;
        }
    }

    if (!enabled) return 0;
    if (s_power_envelope == 255 || s_power_fade_duration_ms == 0) return 255;

    const uint64_t elapsed = now_ms - s_power_fade_start_ms;
    if (elapsed >= s_power_fade_duration_ms) {
        s_power_envelope = 255;
        return 255;
    }

    // Smoothstep makes the cabin come up gently instead of looking like a linear dimmer ramp.
    const float t = static_cast<float>(elapsed) / static_cast<float>(s_power_fade_duration_ms);
    const float smooth = t * t * (3.0f - 2.0f * t);
    s_power_envelope = static_cast<uint8_t>(smooth * 255.0f + 0.5f);
    return s_power_envelope;
}

// Bright ambient conditions DIM vehicle LEDs; darkness leaves manual
// brightness untouched. Only the lighting renderer consumes this value.
// Sensor and output smoothing below prevent headlights from causing flashes.
float ambient_target(float lux)
{
    if (lux <= 2.0f) return 1.0f;
    if (lux < 20.0f) return 1.0f - (lux - 2.0f) * (0.22f / 18.0f);
    if (lux < 100.0f) return 0.78f - (lux - 20.0f) * (0.28f / 80.0f);
    if (lux < 500.0f) return 0.50f - (lux - 100.0f) * (0.50f / 400.0f);
    // Above 500 lux the ALS may reduce the physical LED output fully to OFF;
    // the saved master brightness is untouched and is restored in darkness.
    return 0.0f;
}

uint8_t auto_brightness_scale(const accessories::Snapshot& als)
{
    const bool sensor_ok = als.als_enabled && als.als_present &&
                           als.lux >= 0.0f && als.lux < 200000.0f;
    float requested = 1.0f;
    if (sensor_ok) {
        if (!s_ambient_initialized) {
            s_ambient_filtered_lux = als.lux;
            s_ambient_initialized = true;
        }
        // Respond faster when light increases; brighten more gently when it
        // goes away. This avoids a long delay before a bright source dims LEDs
        // without making the cabin flash at every momentary passing shadow.
        const float lux_alpha = als.lux > s_ambient_filtered_lux ? 0.07f : 0.025f;
        s_ambient_filtered_lux += lux_alpha * (als.lux - s_ambient_filtered_lux);
        requested = ambient_target(s_ambient_filtered_lux);
    } else {
        s_ambient_initialized = false;
    }
    const float brightness_alpha = requested < s_ambient_factor ? 0.08f : 0.025f;
    s_ambient_factor += brightness_alpha * (requested - s_ambient_factor);
    s_ambient_factor = std::clamp(s_ambient_factor, 0.0f, 1.0f);
    return static_cast<uint8_t>(s_ambient_factor * 255.0f + 0.5f);
}

void apply_output_brightness(const rgb::DeviceConfig& cfg, uint8_t power_envelope,
                             uint8_t ambient_scale)
{
    for (std::size_t ch_index = 0; ch_index < rgb::kChannelCount; ++ch_index) {
        const auto& ch = cfg.channels[ch_index];
        const uint8_t channel_master = scale8(cfg.system.master_brightness, ch.brightness);
        const uint8_t combined = scale8(scale8(channel_master, power_envelope), ambient_scale);
        const std::size_t count = std::min<std::size_t>(ch.led_count, rgb::kMaxLedsPerChannel);
        for (std::size_t i = 0; i < count; ++i) s_frame[ch_index][i] = scaled(s_frame[ch_index][i], combined);
    }
}


void render_length_preview(int channel, std::size_t preview_count)
{
    if (channel < 0 || channel >= static_cast<int>(rgb::kChannelCount)) return;
    const std::size_t count = std::min<std::size_t>(preview_count, rgb::kMaxLedsPerChannel);
    if (!count) return;
    for (std::size_t i = 0; i < count; ++i) {
        s_frame[channel][i] = (i + 1 == count) ? rgb::Color{0, 255, 80} : rgb::Color{24, 24, 24};
    }
}

void render_identify(const rgb::DeviceConfig& cfg, int channel, uint64_t now_ms)
{
    if (channel < 0 || channel >= static_cast<int>(rgb::kChannelCount)) return;
    const auto& ch = cfg.channels[channel];
    const std::size_t count = std::min<std::size_t>(ch.led_count, rgb::kMaxLedsPerChannel);
    const bool on = ((now_ms / 200) % 2) == 0;
    const rgb::Color c = on ? rgb::Color{255, 255, 255} : rgb::Color{0, 0, 0};
    for (std::size_t i = 0; i < count; ++i) s_frame[channel][i] = c;
}

void render_frame(const rgb::DeviceConfig& cfg, uint64_t now_ms, uint8_t ambient_scale)
{
    for (auto& channel : s_frame) std::fill(channel.begin(), channel.end(), rgb::Color{});

    const int preview = s_preview_channel;
    if (preview >= 0 && s_preview_count > 0) {
        render_length_preview(preview, s_preview_count);
        return;
    }

    const int identify = s_identify_channel;
    if (identify >= 0 && esp_timer_get_time() < s_identify_until_us) {
        render_identify(cfg, identify, now_ms);
        return;
    }
    if (identify >= 0) s_identify_channel = -1;

    const uint8_t power_envelope = update_power_envelope(cfg, now_ms);
    if (power_envelope == 0) return;

    if (!cfg.scene_count || cfg.system.active_scene >= cfg.scene_count) return;
    const auto& scene = cfg.scenes[cfg.system.active_scene];
    if (!scene.enabled) return;

    // Base layer: All Lighting across every enabled physical output. It is
    // not a user-created zone and continues working when zone_count == 0.
    const auto& global = scene.all_lighting;
    if (global.enabled) {
        rgb::ZoneConfig whole{};
        whole.segment_count = static_cast<uint8_t>(rgb::kChannelCount);
        for (std::size_t i = 0; i < rgb::kChannelCount; ++i) {
            whole.segments[i].channel = static_cast<uint8_t>(i);
        }
        const std::size_t length = zone_length(cfg, whole);
        if (length) {
            effects::render(global.effect.id, s_scratch.data(), length, global.effect, now_ms);
            for (std::size_t i = 0; i < length; ++i) s_scratch[i] = scaled(s_scratch[i], global.brightness);
            map_zone(cfg, whole, s_scratch.data(), length);
        }
    }
    // User zones override the global pixels they cover; later zones win.
    for (std::size_t z = 0; z < cfg.zone_count; ++z) {
        const auto& zone = cfg.zones[z];
        const auto& style = scene.zones[z];
        if (!zone.enabled || !style.enabled) continue;

        const std::size_t len = zone_length(cfg, zone);
        if (!len) continue;
        effects::render(style.effect.id, s_scratch.data(), len, style.effect, now_ms);
        for (std::size_t i = 0; i < len; ++i) s_scratch[i] = scaled(s_scratch[i], style.brightness);
        map_zone(cfg, zone, s_scratch.data(), len);
    }

    apply_output_brightness(cfg, power_envelope, ambient_scale);
}

void task(void*)
{
    TickType_t last = xTaskGetTickCount();
    while (true) {
        config::snapshot_into(s_render_config);
        auto& cfg = s_render_config;
        const int preview = s_preview_channel;
        const std::size_t preview_count = s_preview_count;
        if (preview >= 0 && preview < static_cast<int>(rgb::kChannelCount) && preview_count > 0) {
            cfg.channels[preview].enabled = true;
            cfg.channels[preview].led_count = static_cast<uint16_t>(std::min<std::size_t>(preview_count, rgb::kMaxLedsPerChannel));
            cfg.channels[preview].brightness = 255;
        }
        const uint64_t now_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);
        // A read-only sensor snapshot has no BLE traffic and doesn't alter
        // saved master/channel/zone brightness settings.
        const uint8_t ambient_scale = auto_brightness_scale(accessories::snapshot());
        render_frame(cfg, now_ms, ambient_scale);
        const esp_err_t err = led::transmit(s_frame, cfg);
        if (err != ESP_OK) ESP_LOGW(TAG, "LED transmit failed: %s", esp_err_to_name(err));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(kFrameMs));
    }
}

} // namespace

namespace lighting {

esp_err_t init()
{
    return led::init();
}

esp_err_t start()
{
    if (s_task) return ESP_OK;
    if (xTaskCreatePinnedToCore(task, "lighting", 6144, nullptr, 6, &s_task, 1) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Lighting renderer started at ~30 FPS");
    return ESP_OK;
}

void identify(std::size_t channel, uint32_t duration_ms)
{
    if (channel >= rgb::kChannelCount) return;
    s_identify_channel = static_cast<int>(channel);
    s_identify_until_us = esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
}

void stop_identify()
{
    s_identify_channel = -1;
    s_identify_until_us = 0;
}

void preview_length(std::size_t channel, std::size_t count)
{
    if (channel >= rgb::kChannelCount || count == 0) {
        stop_length_preview();
        return;
    }
    s_identify_channel = -1;
    s_preview_count = std::min<std::size_t>(count, rgb::kMaxLedsPerChannel);
    s_preview_channel = static_cast<int>(channel);
}

void stop_length_preview()
{
    s_preview_channel = -1;
    s_preview_count = 0;
}

} // namespace lighting
