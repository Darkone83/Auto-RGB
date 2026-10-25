#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "esp_err.h"
#include "rgb_model.hpp"

namespace config {

struct ControlSnapshot {
    uint32_t revision{0};
    bool dirty{false};
    bool remote_enabled{false};
    bool master_power{false};
    uint8_t master_brightness{0};
    uint8_t active_scene{0};
    uint8_t scene_count{0};
    std::array<std::array<char, 24>, rgb::kMaxScenes> scene_names{};
};


struct AccessorySettingsSnapshot {
    uint32_t revision{0};
    bool remote_enabled{false};
    bool rtc_enabled{false};
    bool als_enabled{false};
    rgb::AlsSensitivity als_sensitivity{rgb::AlsSensitivity::Normal};
};

struct RemoteSnapshot {
    uint32_t revision{0};
    bool dirty{false};
    bool remote_enabled{false};
    bool master_power{false};
    uint8_t master_brightness{0};
    uint8_t active_scene{0};
    uint8_t scene_count{0};
    uint8_t zone_count{0};
    uint8_t enabled_channels{0};
    std::array<std::array<char, 24>, rgb::kMaxScenes> scene_names{};
    std::array<std::array<char, 24>, rgb::kMaxZones> zone_names{};
    std::array<rgb::ZoneStyle, rgb::kMaxZones> active_zone_styles{};
    rgb::RemoteControlConfig remote{};
    rgb::StartupConfig startup{};
    uint16_t power_on_fade_ms{0};
};

esp_err_t init();
rgb::DeviceConfig snapshot();
void snapshot_into(rgb::DeviceConfig& out);
uint32_t snapshot_into_with_revision(rgb::DeviceConfig& out);
rgb::SystemConfig system_snapshot();
ControlSnapshot control_snapshot();
RemoteSnapshot remote_snapshot();
AccessorySettingsSnapshot accessory_settings_snapshot();
uint32_t revision();

esp_err_t update_channel(std::size_t index, const rgb::ChannelConfig& channel);
esp_err_t update_zone(std::size_t index, const rgb::ZoneConfig& zone);
esp_err_t update_scene(std::size_t index, const rgb::SceneConfig& scene);
// Atomic, revision-checked compaction; built-in demos cannot be deleted.
esp_err_t remove_zone(std::size_t index, uint32_t expected_revision);
esp_err_t remove_custom_scene(std::size_t index, uint32_t expected_revision);
esp_err_t update_system(const rgb::SystemConfig& system);

esp_err_t set_zone_count(uint8_t count);
esp_err_t set_scene_count(uint8_t count);
esp_err_t set_active_scene(uint8_t scene);
esp_err_t set_master_brightness(uint8_t brightness);
esp_err_t set_master_power(bool enabled);
esp_err_t set_power_on_fade_ms(uint16_t duration_ms);
esp_err_t set_remote_control_enabled(bool enabled);
esp_err_t set_remote_display(uint8_t brightness, uint16_t timeout_s, bool invert, bool auto_dim, uint8_t screensaver_minutes);
esp_err_t set_accessories_enabled(bool rtc_enabled, bool als_enabled);
esp_err_t set_als_sensitivity(rgb::AlsSensitivity sensitivity);
esp_err_t set_startup_power_mode(rgb::StartupPowerMode mode);
esp_err_t set_startup_scene(uint8_t scene_or_last);

// Remote quick-control helpers. Editing a firmware demo scene automatically
// clones it into the next free custom slot before applying the change.
esp_err_t set_active_zone_style(std::size_t zone, const rgb::ZoneStyle& style);
esp_err_t apply_quick_color(int zone_or_all, rgb::Color color);

esp_err_t save();
esp_err_t revert();
esp_err_t factory_reset(bool persist);
bool dirty();

} // namespace config
