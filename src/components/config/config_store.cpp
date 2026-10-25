#include "config_store.hpp"
#include "config_compaction.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <new>

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {
constexpr const char* TAG = "config";
constexpr const char* kNamespace = "rgbctl";
constexpr const char* kBlobKey = "device";
constexpr uint16_t kSchemaV1 = 1;
constexpr uint16_t kSchemaV2 = 2;
constexpr uint16_t kSchemaV3 = 3;
constexpr uint16_t kSchemaV4 = 4;
constexpr uint16_t kSchemaV5 = 5;
constexpr uint16_t kSchemaV6 = 6;
constexpr uint16_t kSchemaV7 = 7;
constexpr uint16_t kSchemaV8 = 8;
constexpr uint16_t kSchemaV9 = 9;
constexpr std::size_t kLegacyMaxScenes = 8;
constexpr uint16_t kMaxPowerOnFadeMs = 10000;
constexpr uint16_t kMaxOledTimeoutS = 3600;

// Schema 1-5 all used the original two-color EffectConfig. Keep an exact
// local copy of that layout so increasing EffectConfig in schema 6 does not
// invalidate older NVS blobs.
struct LegacyEffectConfigV5 {
    rgb::EffectId id{rgb::EffectId::Solid};
    rgb::Color primary{96, 0, 160};
    rgb::Color secondary{0, 0, 0};
    uint16_t speed_ms{1800};
    uint8_t width{12};
    uint8_t intensity{255};
    uint8_t density{96};
    bool reverse{false};
};

struct LegacyZoneStyleV5 {
    bool enabled{true};
    uint8_t brightness{255};
    LegacyEffectConfigV5 effect{};
};

struct LegacySceneConfigV5 {
    char name[24]{};
    bool enabled{true};
    std::array<LegacyZoneStyleV5, rgb::kMaxZones> zones{};
};

struct LegacyDeviceConfigV1 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV1};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV5, kLegacyMaxScenes> scenes{};
};

struct LegacyDeviceConfigV2 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV2};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV5, kLegacyMaxScenes> scenes{};
    rgb::PowerConfig power{};
};

struct LegacyDeviceConfigV3 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV3};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV5, rgb::kMaxScenes> scenes{};
    rgb::PowerConfig power{};
};

struct LegacyRemoteControlConfigV4 {
    bool enabled{false};
    std::array<uint8_t, 7> reserved{};
};

struct LegacyDeviceConfigV4 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV4};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV5, rgb::kMaxScenes> scenes{};
    rgb::PowerConfig power{};
    LegacyRemoteControlConfigV4 remote{};
};

struct LegacyDeviceConfigV5 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV5};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV5, rgb::kMaxScenes> scenes{};
    rgb::PowerConfig power{};
    rgb::RemoteControlConfig remote{};
    rgb::StartupConfig startup{};
};

// Schema 6-9 used the same eight-zone SceneConfig without a separate
// All Lighting style. Keep the exact old binary layout for NVS migration.
struct LegacySceneConfigV9 {
    char name[24]{};
    bool enabled{true};
    std::array<rgb::ZoneStyle, rgb::kMaxZones> zones{};
};

struct LegacyDeviceConfigV9 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV9};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV9, rgb::kMaxScenes> scenes{};
    rgb::PowerConfig power{};
    rgb::RemoteControlConfig remote{};
    rgb::StartupConfig startup{};
    rgb::AccessoryConfig accessories{};
};

// Schema 6 introduced the five-color EffectConfig but did not yet include
// RTC / ambient-light accessory enable flags.
struct LegacyDeviceConfigV6 {
    uint32_t magic{rgb::kConfigMagic};
    uint16_t schema_version{kSchemaV6};
    rgb::SystemConfig system{};
    std::array<rgb::ChannelConfig, rgb::kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<rgb::ZoneConfig, rgb::kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<LegacySceneConfigV9, rgb::kMaxScenes> scenes{};
    rgb::PowerConfig power{};
    rgb::RemoteControlConfig remote{};
    rgb::StartupConfig startup{};
};

SemaphoreHandle_t s_mutex = nullptr;
rgb::DeviceConfig s_active{};
rgb::DeviceConfig s_saved{};
bool s_dirty = false;
LegacyDeviceConfigV1 s_legacy_v1{};
LegacyDeviceConfigV2 s_legacy_v2{};
LegacyDeviceConfigV3 s_legacy_v3{};
LegacyDeviceConfigV4 s_legacy_v4{};
LegacyDeviceConfigV5 s_legacy_v5{};
LegacyDeviceConfigV6 s_legacy_v6{};
LegacyDeviceConfigV9 s_legacy_v9{};
uint32_t s_revision = 1;

void copy_text(char* dst, std::size_t size, const char* src)
{
    if (!dst || size == 0) return;
    std::snprintf(dst, size, "%s", src ? src : "");
}

constexpr std::array<const char*, rgb::kBuiltInSceneCount> kBuiltInNames{{
    "Ambient Purple", "Night Drive", "Purple Breathe",
    "Rainbow Cruise", "Meteor Sweep", "Wraith"
}};

int built_in_index_by_name(const char* name)
{
    if (!name) return -1;
    for (std::size_t i = 0; i < kBuiltInNames.size(); ++i) {
        if (std::strcmp(name, kBuiltInNames[i]) == 0) return static_cast<int>(i);
    }
    return -1;
}

void seed_builtin_scenes(rgb::DeviceConfig& cfg)
{
    auto setup_scene = [&](std::size_t index, const char* name, rgb::EffectId effect,
                           rgb::Color primary, rgb::Color secondary, uint8_t brightness,
                           uint16_t speed_ms, uint8_t width = 12, uint8_t density = 96) {
        auto& scene = cfg.scenes[index];
        scene = rgb::SceneConfig{};
        copy_text(scene.name, sizeof(scene.name), name);
        scene.enabled = true;
        for (auto& style : scene.zones) {
            style.enabled = true;
            style.brightness = brightness;
            style.effect.id = effect;
            style.effect.primary = primary;
            style.effect.secondary = secondary;
            style.effect.speed_ms = speed_ms;
            style.effect.width = width;
            style.effect.density = density;
            style.effect.intensity = 255;
            style.effect.reverse = false;
            style.effect.palette_count = 2;
        }
        scene.all_lighting = scene.zones[0];
    };

    setup_scene(0, "Ambient Purple", rgb::EffectId::Solid,
                {96, 0, 160}, {0, 0, 0}, 150, 1800);
    setup_scene(1, "Night Drive", rgb::EffectId::Solid,
                {72, 0, 36}, {0, 0, 0}, 90, 1800);
    setup_scene(2, "Purple Breathe", rgb::EffectId::Breathe,
                {128, 0, 190}, {0, 0, 0}, 150, 3200);
    setup_scene(3, "Rainbow Cruise", rgb::EffectId::Rainbow,
                {0, 0, 0}, {0, 0, 0}, 170, 5000);
    setup_scene(4, "Meteor Sweep", rgb::EffectId::Meteor,
                {255, 0, 96}, {12, 0, 28}, 180, 1800, 14);
    setup_scene(5, "Wraith", rgb::EffectId::Wraith,
                {150, 24, 255}, {8, 0, 18}, 135, 5200, 12, 108);
    for (auto& style : cfg.scenes[5].zones) {
        style.effect.palette_count = 4;
        style.effect.extra_colors[0] = {190, 24, 118};
        style.effect.extra_colors[1] = {24, 40, 130};
    }
    cfg.scenes[5].all_lighting = cfg.scenes[5].zones[0];

    if (cfg.scene_count < rgb::kBuiltInSceneCount) {
        cfg.scene_count = static_cast<uint8_t>(rgb::kBuiltInSceneCount);
    }
}

void set_defaults(rgb::DeviceConfig& cfg)
{
    // Construct directly into the destination. DeviceConfig is several KiB, so
    // returning it by value from this startup path wastes most of the ESP-IDF
    // main task stack. Placement-new keeps the object out of the stack.
    new (&cfg) rgb::DeviceConfig{};
    cfg.magic = rgb::kConfigMagic;
    cfg.schema_version = rgb::kConfigSchemaVersion;
    copy_text(cfg.system.device_name, sizeof(cfg.system.device_name), "RGB Controller");
    copy_text(cfg.system.ap_ssid, sizeof(cfg.system.ap_ssid), "RGB-Control");
    cfg.system.ap_password[0] = '\0'; // Auto-RGB SoftAP is intentionally open.
    cfg.system.master_brightness = 160;
    cfg.system.active_scene = 0;
    cfg.power.master_enabled = true;
    cfg.power.power_on_fade_ms = 1500;
    cfg.remote.enabled = false;
    cfg.remote.oled_contrast = 127;
    cfg.remote.oled_timeout_s = 45;
    cfg.remote.oled_invert = false;
    cfg.remote.oled_auto_dim = false;
    cfg.remote.oled_screensaver_minutes = 0;
    cfg.startup.power_mode = rgb::StartupPowerMode::RestoreLast;
    cfg.startup.scene = rgb::kStartupSceneLast;
    cfg.accessories.rtc_enabled = false;
    cfg.accessories.als_enabled = false;
    cfg.accessories.als_sensitivity = rgb::AlsSensitivity::Normal;

    for (std::size_t i = 0; i < rgb::kChannelCount; ++i) {
        auto& ch = cfg.channels[i];
        std::snprintf(ch.name, sizeof(ch.name), "Channel %u", static_cast<unsigned>(i + 1));
        ch.enabled = false;
        ch.led_count = 0;
        ch.brightness = 255;
        ch.order = rgb::PixelOrder::GRB;
        ch.reversed = false;
    }

    // All Lighting is a global effect layer, not a ZoneConfig. Zero
    // user zones is valid: enabled outputs still render the active scene.
    cfg.zone_count = 0;

    // Six firmware-owned demos are always present and read-only. They do not
    // consume the four owner-customizable scene slots.
    cfg.scene_count = static_cast<uint8_t>(rgb::kBuiltInSceneCount);
    seed_builtin_scenes(cfg);
}

bool valid_common(const rgb::DeviceConfig& cfg)
{
    const auto mode = static_cast<uint8_t>(cfg.startup.power_mode);
    const bool startup_scene_ok = cfg.startup.scene == rgb::kStartupSceneLast || cfg.startup.scene < cfg.scene_count;
    return cfg.magic == rgb::kConfigMagic &&
           cfg.zone_count <= rgb::kMaxZones &&
           cfg.scene_count >= rgb::kBuiltInSceneCount &&
           cfg.scene_count <= rgb::kMaxScenes &&
           mode <= static_cast<uint8_t>(rgb::StartupPowerMode::AlwaysOff) &&
           startup_scene_ok;
}

struct Lock {
    Lock() { if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY); }
    ~Lock() { if (s_mutex) xSemaphoreGive(s_mutex); }
};

void touch_locked()
{
    ++s_revision;
    if (s_revision == 0) s_revision = 1;
}

esp_err_t write_blob(const rgb::DeviceConfig& cfg)
{
    nvs_handle_t handle{};
    esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, kBlobKey, &cfg, sizeof(cfg));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

rgb::EffectConfig migrate_effect(const LegacyEffectConfigV5& old)
{
    rgb::EffectConfig out{};
    out.id = old.id;
    out.primary = old.primary;
    out.secondary = old.secondary;
    out.speed_ms = old.speed_ms;
    out.width = old.width;
    out.intensity = old.intensity;
    out.density = old.density;
    out.reverse = old.reverse;
    out.palette_count = 2;
    return out;
}

rgb::SceneConfig migrate_scene(const LegacySceneConfigV5& old)
{
    rgb::SceneConfig out{};
    std::memcpy(out.name, old.name, sizeof(out.name));
    out.name[sizeof(out.name) - 1] = '\0';
    out.enabled = old.enabled;
    for (std::size_t z = 0; z < rgb::kMaxZones; ++z) {
        out.zones[z].enabled = old.zones[z].enabled;
        out.zones[z].brightness = old.zones[z].brightness;
        out.zones[z].effect = migrate_effect(old.zones[z].effect);
    }
    out.all_lighting = out.zones[0];
    return out;
}

// Old factory configuration placed a generated "All Lighting" mapping into
// zones[0]. Move its style into the global effect and reclaim the user slot.
// Installations that already removed/renamed it keep all their user zones.
void extract_legacy_all_lighting(rgb::DeviceConfig& out)
{
    if (!out.zone_count || std::strcmp(out.zones[0].name, "All Lighting") != 0) return;
    // Only remove the known factory-generated map. If an owner edited this
    // area's segments, preserve it as a user zone instead of silently losing
    // its physical mapping during migration.
    const auto& legacy = out.zones[0];
    if (legacy.segment_count != rgb::kChannelCount) return;
    for (std::size_t i = 0; i < rgb::kChannelCount; ++i) {
        const auto& part = legacy.segments[i];
        if (part.channel != i || part.start || part.length || part.reversed) return;
    }
    for (std::size_t s = 0; s < out.scene_count; ++s) {
        out.scenes[s].all_lighting = out.scenes[s].zones[0];
    }
    const std::size_t old_count = out.zone_count;
    for (std::size_t z = 0; z + 1 < old_count; ++z) out.zones[z] = out.zones[z + 1];
    out.zones[old_count - 1] = rgb::ZoneConfig{};
    for (std::size_t s = 0; s < out.scene_count; ++s) {
        for (std::size_t z = 0; z + 1 < old_count; ++z) {
            out.scenes[s].zones[z] = out.scenes[s].zones[z + 1];
        }
        out.scenes[s].zones[old_count - 1] = rgb::ZoneStyle{};
    }
    out.zone_count = static_cast<uint8_t>(old_count - 1);
}

template <typename LegacyConfig>
void migrate_legacy(const LegacyConfig& legacy, rgb::DeviceConfig& out, const rgb::PowerConfig* legacy_power)
{
    set_defaults(out);

    out.system = legacy.system;
    out.system.ap_password[0] = '\0';
    out.channels = legacy.channels;
    out.zone_count = std::min<uint8_t>(legacy.zone_count, static_cast<uint8_t>(rgb::kMaxZones));
    out.zones = legacy.zones;
    if (legacy_power) out.power = *legacy_power;

    std::array<int, kLegacyMaxScenes> old_to_new{};
    old_to_new.fill(-1);
    std::size_t custom_count = 0;
    const std::size_t old_count = std::min<std::size_t>(legacy.scene_count, kLegacyMaxScenes);

    for (std::size_t i = 0; i < old_count; ++i) {
        const int demo = built_in_index_by_name(legacy.scenes[i].name);
        if (demo >= 0) {
            old_to_new[i] = demo;
            continue;
        }
        if (custom_count >= rgb::kCustomSceneCount) continue;
        const std::size_t dst = rgb::kBuiltInSceneCount + custom_count++;
        out.scenes[dst] = migrate_scene(legacy.scenes[i]);
        old_to_new[i] = static_cast<int>(dst);
    }

    out.scene_count = static_cast<uint8_t>(rgb::kBuiltInSceneCount + custom_count);
    out.system.active_scene = 0;
    if (legacy.system.active_scene < old_count) {
        const int mapped = old_to_new[legacy.system.active_scene];
        if (mapped >= 0) out.system.active_scene = static_cast<uint8_t>(mapped);
    }

    extract_legacy_all_lighting(out);
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

template <typename LegacyConfig>
void migrate_scene_array(const LegacyConfig& legacy, rgb::DeviceConfig& out)
{
    out.system = legacy.system;
    out.system.ap_password[0] = '\0';
    out.channels = legacy.channels;
    out.zone_count = std::min<uint8_t>(legacy.zone_count, static_cast<uint8_t>(rgb::kMaxZones));
    out.zones = legacy.zones;
    out.scene_count = std::clamp<uint8_t>(legacy.scene_count,
                                          static_cast<uint8_t>(rgb::kBuiltInSceneCount),
                                          static_cast<uint8_t>(rgb::kMaxScenes));
    for (std::size_t i = 0; i < out.scene_count; ++i) out.scenes[i] = migrate_scene(legacy.scenes[i]);
    out.power = legacy.power;
}

void migrate_v3(const LegacyDeviceConfigV3& legacy, rgb::DeviceConfig& out)
{
    set_defaults(out);
    migrate_scene_array(legacy, out);
    out.remote.enabled = false;
    if (out.system.active_scene >= out.scene_count) out.system.active_scene = 0;
    extract_legacy_all_lighting(out);
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

void migrate_v4(const LegacyDeviceConfigV4& legacy, rgb::DeviceConfig& out)
{
    set_defaults(out);
    migrate_scene_array(legacy, out);
    out.remote.enabled = legacy.remote.enabled;
    if (out.system.active_scene >= out.scene_count) out.system.active_scene = 0;
    extract_legacy_all_lighting(out);
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

void migrate_v5(const LegacyDeviceConfigV5& legacy, rgb::DeviceConfig& out)
{
    set_defaults(out);
    migrate_scene_array(legacy, out);
    out.remote = legacy.remote;
    out.remote.oled_auto_dim = false;
    out.remote.oled_screensaver_minutes = 0;
    out.startup = legacy.startup;
    if (out.system.active_scene >= out.scene_count) out.system.active_scene = 0;
    extract_legacy_all_lighting(out);
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

void migrate_v6(const LegacyDeviceConfigV6& legacy, rgb::DeviceConfig& out)
{
    set_defaults(out);
    out.system = legacy.system;
    out.system.ap_password[0] = '\0';
    out.channels = legacy.channels;
    out.zone_count = std::min<uint8_t>(legacy.zone_count, static_cast<uint8_t>(rgb::kMaxZones));
    out.zones = legacy.zones;
    out.scene_count = std::clamp<uint8_t>(legacy.scene_count,
                                          static_cast<uint8_t>(rgb::kBuiltInSceneCount),
                                          static_cast<uint8_t>(rgb::kMaxScenes));
    for (std::size_t i = 0; i < out.scene_count; ++i) {
        std::memcpy(out.scenes[i].name, legacy.scenes[i].name, sizeof(out.scenes[i].name));
        out.scenes[i].enabled = legacy.scenes[i].enabled;
        out.scenes[i].zones = legacy.scenes[i].zones;
        out.scenes[i].all_lighting = legacy.scenes[i].zones[0];
    }
    out.power = legacy.power;
    out.remote = legacy.remote;
    out.remote.oled_auto_dim = false;
    out.remote.oled_screensaver_minutes = 0;
    out.startup = legacy.startup;
    out.accessories = rgb::AccessoryConfig{};
    if (out.system.active_scene >= out.scene_count) out.system.active_scene = 0;
    extract_legacy_all_lighting(out);
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

void migrate_v9(const LegacyDeviceConfigV9& legacy, rgb::DeviceConfig& out)
{
    set_defaults(out);
    out.system = legacy.system;
    out.system.ap_password[0] = '\0';
    out.channels = legacy.channels;
    out.zone_count = std::min<uint8_t>(legacy.zone_count, static_cast<uint8_t>(rgb::kMaxZones));
    out.zones = legacy.zones;
    out.scene_count = std::clamp<uint8_t>(legacy.scene_count,
                                          static_cast<uint8_t>(rgb::kBuiltInSceneCount),
                                          static_cast<uint8_t>(rgb::kMaxScenes));
    for (std::size_t s = 0; s < out.scene_count; ++s) {
        auto& target = out.scenes[s];
        const auto& original = legacy.scenes[s];
        std::memcpy(target.name, original.name, sizeof(target.name));
        target.name[sizeof(target.name)-1] = '\0';
        target.enabled = original.enabled;
        target.zones = original.zones;
        target.all_lighting = original.zones[0];
    }
    out.power = legacy.power;
    out.remote = legacy.remote;
    out.startup = legacy.startup;
    out.accessories = legacy.accessories;
    if (legacy.schema_version == kSchemaV7) {
        out.accessories.als_sensitivity = rgb::AlsSensitivity::Normal;
        out.remote.oled_auto_dim = false;
        out.remote.oled_screensaver_minutes = 0;
    } else if (legacy.schema_version == kSchemaV8) {
        out.remote.oled_auto_dim = false;
        out.remote.oled_screensaver_minutes = 0;
    }
    extract_legacy_all_lighting(out);
    if (out.system.active_scene >= out.scene_count) out.system.active_scene = 0;
    out.schema_version = rgb::kConfigSchemaVersion;
    seed_builtin_scenes(out);
}

void apply_startup_policy(rgb::DeviceConfig& cfg)
{
    switch (cfg.startup.power_mode) {
        case rgb::StartupPowerMode::AlwaysOn: cfg.power.master_enabled = true; break;
        case rgb::StartupPowerMode::AlwaysOff: cfg.power.master_enabled = false; break;
        case rgb::StartupPowerMode::RestoreLast:
        default: break;
    }
    if (cfg.startup.scene != rgb::kStartupSceneLast && cfg.startup.scene < cfg.scene_count) {
        cfg.system.active_scene = cfg.startup.scene;
    }
}

esp_err_t ensure_active_scene_custom_locked()
{
    if (s_active.system.active_scene >= s_active.scene_count) return ESP_ERR_INVALID_STATE;
    if (s_active.system.active_scene >= rgb::kBuiltInSceneCount) return ESP_OK;
    if (s_active.scene_count >= rgb::kMaxScenes) return ESP_ERR_NO_MEM;

    const std::size_t source = s_active.system.active_scene;
    const std::size_t target = s_active.scene_count;

    // Snapshot the source name before copying the scene. GCC 14 correctly treats
    // snprintf() source/destination pointers into the same aggregate as a possible
    // restrict overlap even though source and target are different scene slots.
    char source_name[sizeof(s_active.scenes[target].name)]{};
    std::memcpy(source_name, s_active.scenes[source].name, sizeof(source_name));
    source_name[sizeof(source_name) - 1] = '\0';

    s_active.scenes[target] = s_active.scenes[source];
    std::snprintf(s_active.scenes[target].name, sizeof(s_active.scenes[target].name),
                  "%.16s Custom", source_name);
    s_active.scene_count = static_cast<uint8_t>(target + 1);
    s_active.system.active_scene = static_cast<uint8_t>(target);
    return ESP_OK;
}

esp_err_t read_blob(rgb::DeviceConfig& out, bool& migrated)
{
    migrated = false;

    nvs_handle_t handle{};
    esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    size_t stored_size = 0;
    err = nvs_get_blob(handle, kBlobKey, nullptr, &stored_size);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    if (stored_size == sizeof(rgb::DeviceConfig)) {
        size_t read_size = sizeof(rgb::DeviceConfig);
        err = nvs_get_blob(handle, kBlobKey, &out, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (out.magic != rgb::kConfigMagic || out.schema_version != rgb::kConfigSchemaVersion ||
            out.zone_count > rgb::kMaxZones || out.scene_count > rgb::kMaxScenes) {
            return ESP_ERR_INVALID_VERSION;
        }
        seed_builtin_scenes(out);
    } else if (stored_size == sizeof(LegacyDeviceConfigV9)) {
        size_t read_size = sizeof(LegacyDeviceConfigV9);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v9, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v9.magic != rgb::kConfigMagic ||
            (s_legacy_v9.schema_version != kSchemaV7 &&
             s_legacy_v9.schema_version != kSchemaV8 &&
             s_legacy_v9.schema_version != kSchemaV9)) return ESP_ERR_INVALID_VERSION;
        migrate_v9(s_legacy_v9, out);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV6)) {
        size_t read_size = sizeof(LegacyDeviceConfigV6);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v6, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v6.magic != rgb::kConfigMagic || s_legacy_v6.schema_version != kSchemaV6) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_v6(s_legacy_v6, out);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV5)) {
        size_t read_size = sizeof(LegacyDeviceConfigV5);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v5, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v5.magic != rgb::kConfigMagic || s_legacy_v5.schema_version != kSchemaV5) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_v5(s_legacy_v5, out);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV4)) {
        size_t read_size = sizeof(LegacyDeviceConfigV4);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v4, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v4.magic != rgb::kConfigMagic || s_legacy_v4.schema_version != kSchemaV4) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_v4(s_legacy_v4, out);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV3)) {
        size_t read_size = sizeof(LegacyDeviceConfigV3);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v3, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v3.magic != rgb::kConfigMagic || s_legacy_v3.schema_version != kSchemaV3) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_v3(s_legacy_v3, out);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV2)) {
        size_t read_size = sizeof(LegacyDeviceConfigV2);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v2, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v2.magic != rgb::kConfigMagic || s_legacy_v2.schema_version != kSchemaV2) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_legacy(s_legacy_v2, out, &s_legacy_v2.power);
        migrated = true;
    } else if (stored_size == sizeof(LegacyDeviceConfigV1)) {
        size_t read_size = sizeof(LegacyDeviceConfigV1);
        err = nvs_get_blob(handle, kBlobKey, &s_legacy_v1, &read_size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (s_legacy_v1.magic != rgb::kConfigMagic || s_legacy_v1.schema_version != kSchemaV1) {
            return ESP_ERR_INVALID_VERSION;
        }
        migrate_legacy(s_legacy_v1, out, nullptr);
        migrated = true;
    } else {
        nvs_close(handle);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!valid_common(out)) return ESP_ERR_INVALID_STATE;

    if (out.system.ap_password[0] != '\0') {
        out.system.ap_password[0] = '\0';
        migrated = true;
    }
    if (out.power.power_on_fade_ms > kMaxPowerOnFadeMs) {
        out.power.power_on_fade_ms = kMaxPowerOnFadeMs;
        migrated = true;
    }
    if (out.system.active_scene >= out.scene_count) {
        out.system.active_scene = 0;
        migrated = true;
    }
    if (out.remote.oled_timeout_s > kMaxOledTimeoutS) {
        out.remote.oled_timeout_s = 45;
        migrated = true;
    }
    if (out.remote.oled_screensaver_minutes > 5) {
        out.remote.oled_screensaver_minutes = 0;
        migrated = true;
    }
    if (static_cast<uint8_t>(out.startup.power_mode) > static_cast<uint8_t>(rgb::StartupPowerMode::AlwaysOff)) {
        out.startup.power_mode = rgb::StartupPowerMode::RestoreLast;
        migrated = true;
    }
    if (out.startup.scene != rgb::kStartupSceneLast && out.startup.scene >= out.scene_count) {
        out.startup.scene = rgb::kStartupSceneLast;
        migrated = true;
    }
    if (static_cast<uint8_t>(out.accessories.als_sensitivity) >
        static_cast<uint8_t>(rgb::AlsSensitivity::VeryHigh)) {
        out.accessories.als_sensitivity = rgb::AlsSensitivity::Normal;
        migrated = true;
    }
    for (std::size_t sidx = 0; sidx < out.scene_count; ++sidx) {
        for (auto& style : out.scenes[sidx].zones) {
            if (static_cast<uint8_t>(style.effect.id) > static_cast<uint8_t>(rgb::EffectId::ColorWaves)) {
                style.effect.id = rgb::EffectId::Solid;
                migrated = true;
            }
            if (style.effect.palette_count < 1 || style.effect.palette_count > rgb::kMaxPaletteColors) {
                style.effect.palette_count = 2;
                migrated = true;
            }
        }
    }

    return ESP_OK;
}

} // namespace

namespace config {

esp_err_t init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    bool migrated = false;
    {
        Lock lock;
        // Read directly into static storage. Keeping a full DeviceConfig local here
        // consumed ~2.3 KiB of the main task stack before Wi-Fi even started.
        err = read_blob(s_active, migrated);
        if (err != ESP_OK) set_defaults(s_active);
        apply_startup_policy(s_active);
        s_saved = s_active;
        s_dirty = false;
        s_revision = 1;
    }

    if (err == ESP_OK) {
        if (migrated) {
            ESP_LOGI(TAG, "Migrated configuration to schema %u", rgb::kConfigSchemaVersion);
            const esp_err_t save_err = write_blob(s_active);
            if (save_err != ESP_OK) ESP_LOGW(TAG, "Could not persist migrated configuration: %s", esp_err_to_name(save_err));
        } else {
            ESP_LOGI(TAG, "Loaded configuration schema %u", s_active.schema_version);
        }
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No valid saved configuration; using factory defaults (%s)", esp_err_to_name(err));
    return write_blob(s_active);
}

rgb::DeviceConfig snapshot()
{
    Lock lock;
    return s_active;
}

void snapshot_into(rgb::DeviceConfig& out)
{
    Lock lock;
    out = s_active;
}

// The UI deletes numbered slots using this snapshot's revision. Taking both
// under one lock prevents a stale list being paired with a newer revision.
uint32_t snapshot_into_with_revision(rgb::DeviceConfig& out)
{
    Lock lock;
    out = s_active;
    return s_revision;
}

rgb::SystemConfig system_snapshot()
{
    Lock lock;
    return s_active.system;
}

ControlSnapshot control_snapshot()
{
    Lock lock;
    ControlSnapshot out{};
    out.revision = s_revision;
    out.dirty = s_dirty;
    out.remote_enabled = s_active.remote.enabled;
    out.master_power = s_active.power.master_enabled;
    out.master_brightness = s_active.system.master_brightness;
    out.active_scene = s_active.system.active_scene;
    out.scene_count = s_active.scene_count;
    for (std::size_t i = 0; i < s_active.scene_count && i < rgb::kMaxScenes; ++i) {
        std::snprintf(out.scene_names[i].data(), out.scene_names[i].size(), "%s", s_active.scenes[i].name);
    }
    return out;
}

RemoteSnapshot remote_snapshot()
{
    Lock lock;
    RemoteSnapshot out{};
    out.revision = s_revision;
    out.dirty = s_dirty;
    out.remote_enabled = s_active.remote.enabled;
    out.master_power = s_active.power.master_enabled;
    out.master_brightness = s_active.system.master_brightness;
    out.active_scene = s_active.system.active_scene;
    out.scene_count = s_active.scene_count;
    out.zone_count = s_active.zone_count;
    out.remote = s_active.remote;
    out.startup = s_active.startup;
    out.power_on_fade_ms = s_active.power.power_on_fade_ms;
    for (const auto& ch : s_active.channels) if (ch.enabled) ++out.enabled_channels;
    for (std::size_t i = 0; i < s_active.scene_count && i < rgb::kMaxScenes; ++i) {
        std::snprintf(out.scene_names[i].data(), out.scene_names[i].size(), "%s", s_active.scenes[i].name);
    }
    for (std::size_t z = 0; z < s_active.zone_count && z < rgb::kMaxZones; ++z) {
        std::snprintf(out.zone_names[z].data(), out.zone_names[z].size(), "%s", s_active.zones[z].name);
        if (s_active.system.active_scene < s_active.scene_count) {
            out.active_zone_styles[z] = s_active.scenes[s_active.system.active_scene].zones[z];
        }
    }
    return out;
}

AccessorySettingsSnapshot accessory_settings_snapshot()
{
    Lock lock;
    AccessorySettingsSnapshot out{};
    out.revision = s_revision;
    out.remote_enabled = s_active.remote.enabled;
    out.rtc_enabled = s_active.accessories.rtc_enabled;
    out.als_enabled = s_active.accessories.als_enabled;
    out.als_sensitivity = s_active.accessories.als_sensitivity;
    return out;
}

uint32_t revision()
{
    Lock lock;
    return s_revision;
}

esp_err_t update_channel(std::size_t index, const rgb::ChannelConfig& channel)
{
    if (index >= rgb::kChannelCount || channel.led_count > rgb::kMaxLedsPerChannel) return ESP_ERR_INVALID_ARG;
    Lock lock;
    s_active.channels[index] = channel;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t update_zone(std::size_t index, const rgb::ZoneConfig& zone)
{
    if (index >= rgb::kMaxZones || zone.segment_count > rgb::kMaxSegmentsPerZone) return ESP_ERR_INVALID_ARG;
    for (std::size_t i = 0; i < zone.segment_count; ++i) {
        if (zone.segments[i].channel >= rgb::kChannelCount) return ESP_ERR_INVALID_ARG;
    }
    Lock lock;
    s_active.zones[index] = zone;
    if (index >= s_active.zone_count) s_active.zone_count = static_cast<uint8_t>(index + 1);
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

// Called only from the API after explicit user confirmation. No flash write
// occurs here: Save persists, Revert restores the prior configuration.
esp_err_t remove_zone(std::size_t index, uint32_t expected_revision)
{
    Lock lock;
    if (expected_revision != s_revision) return ESP_ERR_INVALID_STATE;
    if (!config_compaction::remove_zone(s_active, index)) return ESP_ERR_INVALID_ARG;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t remove_custom_scene(std::size_t index, uint32_t expected_revision)
{
    Lock lock;
    if (expected_revision != s_revision) return ESP_ERR_INVALID_STATE;
    if (!config_compaction::remove_custom_scene(s_active, index)) return ESP_ERR_INVALID_ARG;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t update_scene(std::size_t index, const rgb::SceneConfig& scene)
{
    // Firmware demo scenes are read-only. Owner scenes live in the four slots
    // immediately after the built-ins.
    if (index < rgb::kBuiltInSceneCount || index >= rgb::kMaxScenes) return ESP_ERR_INVALID_ARG;
    Lock lock;
    if (index > s_active.scene_count) return ESP_ERR_INVALID_ARG; // no sparse scene holes
    s_active.scenes[index] = scene;
    if (index >= s_active.scene_count) s_active.scene_count = static_cast<uint8_t>(index + 1);
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t update_system(const rgb::SystemConfig& system)
{
    Lock lock;
    s_active.system = system;
    s_active.system.ap_password[0] = '\0';
    if (s_active.scene_count && s_active.system.active_scene >= s_active.scene_count) s_active.system.active_scene = 0;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_zone_count(uint8_t count)
{
    if (count > rgb::kMaxZones) return ESP_ERR_INVALID_ARG;
    Lock lock;
    s_active.zone_count = count;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_scene_count(uint8_t count)
{
    if (count < rgb::kBuiltInSceneCount || count > rgb::kMaxScenes) return ESP_ERR_INVALID_ARG;
    Lock lock;
    s_active.scene_count = count;
    if (s_active.system.active_scene >= count) s_active.system.active_scene = 0;
    if (s_active.startup.scene != rgb::kStartupSceneLast && s_active.startup.scene >= count) {
        s_active.startup.scene = rgb::kStartupSceneLast;
    }
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_active_scene(uint8_t scene)
{
    Lock lock;
    if (scene >= s_active.scene_count) return ESP_ERR_INVALID_ARG;
    s_active.system.active_scene = scene;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_master_brightness(uint8_t brightness)
{
    Lock lock;
    s_active.system.master_brightness = brightness;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_master_power(bool enabled)
{
    Lock lock;
    s_active.power.master_enabled = enabled;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_power_on_fade_ms(uint16_t duration_ms)
{
    if (duration_ms > kMaxPowerOnFadeMs) return ESP_ERR_INVALID_ARG;
    Lock lock;
    s_active.power.power_on_fade_ms = duration_ms;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_remote_control_enabled(bool enabled)
{
    Lock lock;
    s_active.remote.enabled = enabled;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_remote_display(uint8_t brightness, uint16_t timeout_s, bool invert,
                             bool auto_dim, uint8_t screensaver_minutes)
{
    if (timeout_s > kMaxOledTimeoutS || screensaver_minutes > 5) return ESP_ERR_INVALID_ARG;
    Lock lock;
    s_active.remote.oled_contrast = brightness;
    s_active.remote.oled_timeout_s = timeout_s;
    s_active.remote.oled_invert = invert;
    s_active.remote.oled_auto_dim = auto_dim;
    s_active.remote.oled_screensaver_minutes = screensaver_minutes;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_accessories_enabled(bool rtc_enabled, bool als_enabled)
{
    Lock lock;
    s_active.accessories.rtc_enabled = rtc_enabled;
    s_active.accessories.als_enabled = als_enabled;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_als_sensitivity(rgb::AlsSensitivity sensitivity)
{
    if (static_cast<uint8_t>(sensitivity) > static_cast<uint8_t>(rgb::AlsSensitivity::VeryHigh)) {
        return ESP_ERR_INVALID_ARG;
    }
    Lock lock;
    s_active.accessories.als_sensitivity = sensitivity;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_startup_power_mode(rgb::StartupPowerMode mode)
{
    if (static_cast<uint8_t>(mode) > static_cast<uint8_t>(rgb::StartupPowerMode::AlwaysOff)) {
        return ESP_ERR_INVALID_ARG;
    }
    Lock lock;
    s_active.startup.power_mode = mode;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_startup_scene(uint8_t scene_or_last)
{
    Lock lock;
    if (scene_or_last != rgb::kStartupSceneLast && scene_or_last >= s_active.scene_count) {
        return ESP_ERR_INVALID_ARG;
    }
    s_active.startup.scene = scene_or_last;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t set_active_zone_style(std::size_t zone, const rgb::ZoneStyle& style)
{
    Lock lock;
    if (zone >= s_active.zone_count || zone >= rgb::kMaxZones) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = ensure_active_scene_custom_locked();
    if (err != ESP_OK) return err;
    s_active.scenes[s_active.system.active_scene].zones[zone] = style;
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t apply_quick_color(int zone_or_all, rgb::Color color)
{
    Lock lock;
    if (zone_or_all >= static_cast<int>(s_active.zone_count)) return ESP_ERR_INVALID_ARG;
    if (zone_or_all < -1) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = ensure_active_scene_custom_locked();
    if (err != ESP_OK) return err;

    auto apply = [&](std::size_t z) {
        auto& style = s_active.scenes[s_active.system.active_scene].zones[z];
        style.enabled = true;
        style.effect.id = rgb::EffectId::Solid;
        style.effect.primary = color;
        style.effect.secondary = {0, 0, 0};
        style.effect.palette_count = 1;
        style.effect.intensity = 255;
    };

    if (zone_or_all < 0) {
        auto& global = s_active.scenes[s_active.system.active_scene].all_lighting;
        global.enabled = true;
        global.effect.id = rgb::EffectId::Solid;
        global.effect.primary = color;
        global.effect.secondary = {0, 0, 0};
        global.effect.palette_count = 1;
        global.effect.intensity = 255;
        for (std::size_t z = 0; z < s_active.zone_count; ++z) apply(z);
    } else {
        apply(static_cast<std::size_t>(zone_or_all));
    }
    s_dirty = true;
    touch_locked();
    return ESP_OK;
}

esp_err_t save()
{
    rgb::DeviceConfig copy{};
    uint32_t captured_revision = 0;
    {
        Lock lock;
        copy = s_active;
        captured_revision = s_revision;
    }
    const esp_err_t err = write_blob(copy);
    if (err == ESP_OK) {
        Lock lock;
        s_saved = copy;
        // A WebUI request or encoder action may have changed the live state
        // while NVS was being written. Never clear dirty in that case.
        s_dirty = s_revision != captured_revision;
        touch_locked();
    }
    return err;
}

esp_err_t revert()
{
    Lock lock;
    s_active = s_saved;
    s_dirty = false;
    touch_locked();
    return ESP_OK;
}

esp_err_t factory_reset(bool persist)
{
    Lock lock;
    set_defaults(s_active);

    if (!persist) {
        s_dirty = true;
        touch_locked();
        return ESP_OK;
    }

    // Keep the configuration mutex held across the NVS write so another HTTP
    // request cannot modify s_active between generating defaults and persisting
    // them. Only mark the reset clean/saved after NVS has confirmed the write.
    const esp_err_t err = write_blob(s_active);
    if (err == ESP_OK) {
        s_saved = s_active;
        s_dirty = false;
    } else {
        s_dirty = true;
    }
    touch_locked();
    return err;
}

bool dirty()
{
    Lock lock;
    return s_dirty;
}

} // namespace config
