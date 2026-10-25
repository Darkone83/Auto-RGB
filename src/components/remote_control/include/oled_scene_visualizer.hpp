#pragma once

// Decorative monochrome "scene motion" for the 128x64 SSD1306.
// Integer-only; no heap, RNG, effects engine calls, timers, or I2C.
// Reads the selected scene's representative EffectId/speed/direction without
// pretending to reproduce the real multi-zone LED renderer.
#include <cstdint>
#include "rgb_model.hpp"

namespace oled_scene_visualizer {

constexpr uint8_t kBars = 15;

enum class Motion : uint8_t { Pulse, Wave, Sweep, Spark, Fire, Orbit };

inline Motion classify(rgb::EffectId id)
{
    switch (id) {
        case rgb::EffectId::Solid:
        case rgb::EffectId::Breathe:
        case rgb::EffectId::Heartbeat:
        case rgb::EffectId::PulseTrain:
            return Motion::Pulse;
        case rgb::EffectId::Meteor:
        case rgb::EffectId::ColorWipe:
        case rgb::EffectId::TheaterChase:
        case rgb::EffectId::Comet:
        case rgb::EffectId::PaletteChase:
        case rgb::EffectId::Crossflow:
        case rgb::EffectId::Wraith:
            return Motion::Sweep;
        case rgb::EffectId::Twinkle:
        case rgb::EffectId::Starlight:
        case rgb::EffectId::Nebula:
            return Motion::Spark;
        case rgb::EffectId::Fire:
        case rgb::EffectId::Lava:
            return Motion::Fire;
        case rgb::EffectId::Larson:
        case rgb::EffectId::ClockSpin:
        case rgb::EffectId::Collision:
        case rgb::EffectId::DualOrbit:
            return Motion::Orbit;
        default:
            return Motion::Wave;
    }
}

// A faster effect yields faster animation; clamp to a sensible shared-I2C
// budget. The OLED transport itself is rate-limited separately to ~5.5 fps.
inline uint32_t frame_duration_ms(uint16_t effect_speed_ms)
{
    const uint32_t step = static_cast<uint32_t>(effect_speed_ms) / 11u;
    return step < 80u ? 80u : (step > 320u ? 320u : step);
}

inline uint32_t hash(uint32_t v)
{
    // Fast deterministic noise. No global PRNG state or scene-side effects.
    v ^= v >> 16;
    v *= 0x7FEB352Du;
    v ^= v >> 15;
    v *= 0x846CA68Bu;
    return v ^ (v >> 16);
}

inline uint8_t triangle(uint32_t x, uint8_t span)
{
    const uint32_t p = x % (static_cast<uint32_t>(span) * 2u);
    return static_cast<uint8_t>(p <= span ? p : (2u * span - p));
}

// Height is always in [1, 15] OLED pixels, for 15 small segmented columns.
inline uint8_t height(Motion motion, uint8_t column, uint32_t phase, bool reverse)
{
    const uint8_t x = reverse ? static_cast<uint8_t>((kBars - 1u) - column) : column;
    switch (motion) {
        case Motion::Pulse: {
            const uint8_t breathe = triangle(phase, 11);
            const uint8_t center = static_cast<uint8_t>(7u - (x > 7u ? x - 7u : 7u - x));
            return static_cast<uint8_t>(2u + breathe / 2u + center);
        }
        case Motion::Wave: {
            const uint8_t wave = triangle(static_cast<uint32_t>(x) * 2u + phase, 14);
            return static_cast<uint8_t>(2u + (wave * 12u) / 14u);
        }
        case Motion::Sweep: {
            const uint8_t head = static_cast<uint8_t>(phase % kBars);
            const uint8_t dist = static_cast<uint8_t>((head + kBars - x) % kBars);
            return dist == 0 ? 15 : dist == 1 ? 12 : dist == 2 ? 9 : dist == 3 ? 5 : 2;
        }
        case Motion::Spark: {
            const uint32_t seed = hash(static_cast<uint32_t>(x) * 0x9E3779B9u + phase / 2u);
            return static_cast<uint8_t>((seed & 7u) == 0u ? 15u :
                                        (seed & 7u) == 1u ? 9u : 2u + ((seed >> 8u) % 4u));
        }
        case Motion::Fire: {
            const uint32_t noise = hash(static_cast<uint32_t>(x) * 103u + phase * 29u);
            const uint8_t pulse = triangle(phase + x / 3u, 5);
            return static_cast<uint8_t>(3u + ((noise >> 8u) % 7u) + pulse);
        }
        case Motion::Orbit: {
            const uint8_t left = triangle(phase, 14);
            const uint8_t right = static_cast<uint8_t>(14u - left);
            const int dl = x > left ? static_cast<int>(x - left) : static_cast<int>(left - x);
            const int dr = x > right ? static_cast<int>(x - right) : static_cast<int>(right - x);
            const int distance = dl < dr ? dl : dr;
            return static_cast<uint8_t>(distance == 0 ? 15 : distance == 1 ? 11 :
                                        distance == 2 ? 7 : 2);
        }
    }
    return 2;
}

} // namespace oled_scene_visualizer
