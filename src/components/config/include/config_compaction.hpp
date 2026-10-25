#pragma once

// Compact numbered zone / custom-scene slots without changing the persistent
// DeviceConfig layout or schema. This header is intentionally ESP-IDF-free so
// its data-preservation properties can be tested on a host compiler.
#include <cstddef>
#include "rgb_model.hpp"

namespace config_compaction {

inline bool remove_zone(rgb::DeviceConfig& cfg, std::size_t index)
{
    if (index >= cfg.zone_count || cfg.zone_count > rgb::kMaxZones) return false;
    const std::size_t old_count = cfg.zone_count;

    // ZoneStyle is indexed by zone ID in every scene, including built-in demos.
    // Shift styles and zones together to keep existing effect assignments.
    for (std::size_t z = index; z + 1 < old_count; ++z) {
        cfg.zones[z] = cfg.zones[z + 1];
    }
    cfg.zones[old_count - 1] = rgb::ZoneConfig{};
    for (std::size_t s = 0; s < cfg.scene_count; ++s) {
        for (std::size_t z = index; z + 1 < old_count; ++z) {
            cfg.scenes[s].zones[z] = cfg.scenes[s].zones[z + 1];
        }
        cfg.scenes[s].zones[old_count - 1] = rgb::ZoneStyle{};
    }
    cfg.zone_count = static_cast<uint8_t>(old_count - 1);
    return true;
}

inline bool remove_custom_scene(rgb::DeviceConfig& cfg, std::size_t index)
{
    if (index < rgb::kBuiltInSceneCount || index >= cfg.scene_count ||
        cfg.scene_count > rgb::kMaxScenes) return false;
    const std::size_t old_count = cfg.scene_count;
    for (std::size_t s = index; s + 1 < old_count; ++s) {
        cfg.scenes[s] = cfg.scenes[s + 1];
    }
    cfg.scenes[old_count - 1] = rgb::SceneConfig{};
    cfg.scene_count = static_cast<uint8_t>(old_count - 1);

    // If the selected custom scene disappeared, fall back to first built-in.
    // Subsequent scene numbers must follow the compacted slots.
    if (cfg.system.active_scene == index) cfg.system.active_scene = 0;
    else if (cfg.system.active_scene > index) --cfg.system.active_scene;

    // A deleted startup scene must not point to a different scene by accident.
    if (cfg.startup.scene != rgb::kStartupSceneLast) {
        if (cfg.startup.scene == index) cfg.startup.scene = rgb::kStartupSceneLast;
        else if (cfg.startup.scene > index) --cfg.startup.scene;
    }
    return true;
}

} // namespace config_compaction
