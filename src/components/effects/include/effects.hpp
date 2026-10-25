#pragma once

#include <cstddef>
#include <cstdint>
#include "rgb_model.hpp"

namespace effects {

enum Control : uint8_t {
    Primary   = 1u << 0,
    Secondary = 1u << 1,
    Speed     = 1u << 2,
    Width     = 1u << 3,
    Density   = 1u << 4,
    Reverse   = 1u << 5,
    Palette   = 1u << 6,
};

struct Descriptor {
    rgb::EffectId id;
    const char* key;
    const char* name;
    uint8_t controls;
    uint8_t palette_min{0};
    uint8_t palette_max{0};
};

std::size_t count();
const Descriptor& descriptor(std::size_t index);
const Descriptor* find(rgb::EffectId id);
const Descriptor* find_key(const char* key);
bool uses(const Descriptor& descriptor, Control control);

void render(rgb::EffectId id,
            rgb::Color* pixels,
            std::size_t pixel_count,
            const rgb::EffectConfig& config,
            uint64_t now_ms);

const char* key(rgb::EffectId id);

} // namespace effects
