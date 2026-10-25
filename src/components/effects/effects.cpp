#include "effects.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

using effects::Control;

constexpr uint8_t P = effects::Primary;
constexpr uint8_t S = effects::Secondary;
constexpr uint8_t V = effects::Speed;
constexpr uint8_t W = effects::Width;
constexpr uint8_t D = effects::Density;
constexpr uint8_t R = effects::Reverse;
constexpr uint8_t L = effects::Palette;

constexpr effects::Descriptor kEffects[] = {
    {rgb::EffectId::Solid,         "solid",          "Solid",           P},
    {rgb::EffectId::Breathe,       "breathe",        "Breathe",         static_cast<uint8_t>(P | V)},
    {rgb::EffectId::ColorWipe,     "color-wipe",     "Color Wipe",      static_cast<uint8_t>(P | S | V | R)},
    {rgb::EffectId::Larson,        "larson",         "Larson Scanner",  static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::Rainbow,       "rainbow",        "Rainbow",         static_cast<uint8_t>(V | R)},
    {rgb::EffectId::TheaterChase,  "theater-chase",  "Theater Chase",   static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::Twinkle,       "twinkle",        "Twinkle",         static_cast<uint8_t>(P | S | V | D)},
    {rgb::EffectId::Comet,         "comet",          "Comet",           static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::Meteor,        "meteor",         "Meteor",          static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::ClockSpin,     "clock-spin",     "Clock Spin",      static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::Plasma,        "plasma",         "Plasma",          static_cast<uint8_t>(P | S | V | R)},
    {rgb::EffectId::Fire,          "fire",           "Fire / Flicker",  static_cast<uint8_t>(P | S | V | D)},
    {rgb::EffectId::PaletteCycle,  "palette-cycle",  "Palette Cycle",   static_cast<uint8_t>(L | V), 2, 5},
    {rgb::EffectId::PaletteChase,  "palette-chase",  "Palette Chase",   static_cast<uint8_t>(L | V | W | R), 2, 5},
    {rgb::EffectId::Crossflow,     "crossflow",      "Crossflow",       static_cast<uint8_t>(P | S | V | W | R)},
    {rgb::EffectId::Wraith,        "wraith",         "Wraith",          static_cast<uint8_t>(L | V | D | R), 2, 5},
    {rgb::EffectId::Heartbeat,     "heartbeat",      "Heartbeat",       static_cast<uint8_t>(P | S | V)},
    {rgb::EffectId::Aurora,        "aurora",         "Aurora",          static_cast<uint8_t>(L | V | D | R), 3, 5},
    {rgb::EffectId::Nebula,        "nebula",         "Nebula",          static_cast<uint8_t>(L | V | D | R), 3, 5},
    {rgb::EffectId::SpectrumBands, "spectrum-bands", "Spectrum Bands",  static_cast<uint8_t>(L | V | W | R), 2, 5},
    {rgb::EffectId::Collision,     "collision",      "Collision",       static_cast<uint8_t>(L | V | W | R), 3, 5},
    {rgb::EffectId::Starlight,     "starlight",      "Starlight",       static_cast<uint8_t>(L | V | D), 2, 5},
    {rgb::EffectId::Lava,          "lava",           "Lava",            static_cast<uint8_t>(L | V | D | R), 3, 5},
    {rgb::EffectId::PulseTrain,    "pulse-train",    "Pulse Train",     static_cast<uint8_t>(L | V | W | R), 2, 5},
    {rgb::EffectId::DualOrbit,     "dual-orbit",     "Multi Orbit",     static_cast<uint8_t>(L | V | W | R), 2, 5},
    {rgb::EffectId::Synthwave,     "synthwave",      "Synthwave",       static_cast<uint8_t>(L | V | W | R), 3, 5},
    {rgb::EffectId::ColorWaves,    "color-waves",    "Color Waves",     static_cast<uint8_t>(L | V | D | R), 2, 5},
};

constexpr float kTau = 6.28318530718f;

uint8_t scale8(uint8_t value, uint8_t scale)
{
    return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale + 127) / 255);
}

rgb::Color scaled(rgb::Color c, uint8_t amount)
{
    return {scale8(c.r, amount), scale8(c.g, amount), scale8(c.b, amount)};
}

rgb::Color blend(rgb::Color a, rgb::Color b, uint8_t mix)
{
    const uint16_t inv = static_cast<uint16_t>(255 - mix);
    return {
        static_cast<uint8_t>((a.r * inv + b.r * mix + 127) / 255),
        static_cast<uint8_t>((a.g * inv + b.g * mix + 127) / 255),
        static_cast<uint8_t>((a.b * inv + b.b * mix + 127) / 255),
    };
}

uint8_t palette_count(const rgb::EffectConfig& cfg)
{
    return static_cast<uint8_t>(std::clamp<int>(cfg.palette_count, 1, static_cast<int>(rgb::kMaxPaletteColors)));
}

rgb::Color palette_color(const rgb::EffectConfig& cfg, std::size_t index)
{
    const uint8_t n = palette_count(cfg);
    index %= n;
    if (index == 0) return cfg.primary;
    if (index == 1) return cfg.secondary;
    return cfg.extra_colors[index - 2];
}

rgb::Color palette_sample(const rgb::EffectConfig& cfg, float position, bool wrap = true)
{
    const uint8_t n = palette_count(cfg);
    if (n <= 1) return cfg.primary;

    if (wrap) {
        position -= std::floor(position);
        const float scaled_pos = position * static_cast<float>(n);
        const std::size_t a = static_cast<std::size_t>(scaled_pos) % n;
        const std::size_t b = (a + 1) % n;
        const float frac = scaled_pos - std::floor(scaled_pos);
        return blend(palette_color(cfg, a), palette_color(cfg, b),
                     static_cast<uint8_t>(std::clamp(frac, 0.0f, 1.0f) * 255.0f));
    }

    position = std::clamp(position, 0.0f, 1.0f);
    const float scaled_pos = position * static_cast<float>(n - 1);
    const std::size_t a = std::min<std::size_t>(static_cast<std::size_t>(scaled_pos), n - 1);
    const std::size_t b = std::min<std::size_t>(a + 1, n - 1);
    const float frac = scaled_pos - static_cast<float>(a);
    return blend(palette_color(cfg, a), palette_color(cfg, b),
                 static_cast<uint8_t>(std::clamp(frac, 0.0f, 1.0f) * 255.0f));
}

rgb::Color wheel(uint8_t pos)
{
    pos = 255 - pos;
    if (pos < 85) return {static_cast<uint8_t>(255 - pos * 3), 0, static_cast<uint8_t>(pos * 3)};
    if (pos < 170) {
        pos -= 85;
        return {0, static_cast<uint8_t>(pos * 3), static_cast<uint8_t>(255 - pos * 3)};
    }
    pos -= 170;
    return {static_cast<uint8_t>(pos * 3), static_cast<uint8_t>(255 - pos * 3), 0};
}

void fill(rgb::Color* pixels, std::size_t count, rgb::Color c)
{
    std::fill(pixels, pixels + count, c);
}

uint32_t hash32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

std::size_t directed_index(std::size_t index, std::size_t count, bool reverse)
{
    return reverse && count ? count - 1 - index : index;
}

void solid(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t)
{
    fill(pixels, count, scaled(cfg.primary, cfg.intensity));
}

void breathe(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    const float phase = static_cast<float>(now_ms % period) / static_cast<float>(period);
    const float wave = 0.12f + 0.88f * (0.5f - 0.5f * std::cos(phase * kTau));
    fill(pixels, count, scaled(cfg.primary, static_cast<uint8_t>(wave * cfg.intensity)));
}

void color_wipe(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, scaled(cfg.secondary, cfg.intensity));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    const std::size_t lit = std::min<std::size_t>(count, (now_ms % period) * (count + 1) / period);
    for (std::size_t i = 0; i < lit; ++i) pixels[directed_index(i, count, cfg.reverse)] = scaled(cfg.primary, cfg.intensity);
}

void larson(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, scaled(cfg.secondary, cfg.intensity));
    if (count == 1) { pixels[0] = scaled(cfg.primary, cfg.intensity); return; }
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const std::size_t path = (count - 1) * 2;
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    std::size_t phase = static_cast<std::size_t>((now_ms % period) * path / period);
    std::size_t head = phase < count ? phase : path - phase;
    if (cfg.reverse) head = count - 1 - head;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t dist = i > head ? i - head : head - i;
        if (dist >= width) continue;
        const uint8_t fade = static_cast<uint8_t>(((width - dist) * 255) / width);
        pixels[i] = blend(pixels[i], scaled(cfg.primary, cfg.intensity), fade);
    }
}

void rainbow(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 200);
    const uint8_t shift = static_cast<uint8_t>((now_ms * 256ULL / period) & 0xFF);
    for (std::size_t i = 0; i < count; ++i) {
        const int base = static_cast<int>((i * 256) / count);
        const int offset = cfg.reverse ? -static_cast<int>(shift) : static_cast<int>(shift);
        pixels[i] = scaled(wheel(static_cast<uint8_t>((base + offset) & 0xFF)), cfg.intensity);
    }
}

void theater_chase(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, 16));
    const std::size_t period_px = width * 3;
    const uint32_t step_ms = std::max<uint32_t>(35, cfg.speed_ms / 12);
    const std::size_t phase = static_cast<std::size_t>(now_ms / step_ms) % period_px;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t p = directed_index(i, count, cfg.reverse);
        const bool on = ((p + phase) % period_px) < width;
        pixels[i] = scaled(on ? cfg.primary : cfg.secondary, cfg.intensity);
    }
}

void twinkle(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const uint32_t step_ms = std::max<uint32_t>(45, cfg.speed_ms / 10);
    const uint32_t frame = static_cast<uint32_t>(now_ms / step_ms);
    const uint8_t density = cfg.density;
    for (std::size_t i = 0; i < count; ++i) {
        const uint32_t h = hash32(static_cast<uint32_t>(i) * 0x9e3779b9U ^ frame * 0x85ebca6bU);
        const bool active = static_cast<uint8_t>(h & 0xFF) < density;
        const uint8_t sparkle = active ? static_cast<uint8_t>(160 + ((h >> 8) & 0x5F)) : 0;
        pixels[i] = active ? blend(scaled(cfg.secondary, cfg.intensity), scaled(cfg.primary, cfg.intensity), sparkle)
                           : scaled(cfg.secondary, cfg.intensity);
    }
}

void comet(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, scaled(cfg.secondary, cfg.intensity));
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    std::size_t head = static_cast<std::size_t>((now_ms % period) * count / period);
    if (cfg.reverse) head = count - 1 - head;
    for (std::size_t tail = 0; tail < width; ++tail) {
        const long pos = cfg.reverse ? static_cast<long>(head + tail) : static_cast<long>(head) - static_cast<long>(tail);
        if (pos < 0 || pos >= static_cast<long>(count)) continue;
        const uint8_t fade = static_cast<uint8_t>(((width - tail) * 255) / width);
        pixels[pos] = blend(scaled(cfg.secondary, cfg.intensity), scaled(cfg.primary, cfg.intensity), fade);
    }
}

void meteor(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, scaled(cfg.secondary, cfg.intensity));
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const uint32_t step_ms = std::max<uint32_t>(20, cfg.speed_ms / std::max<std::size_t>(count, 1));
    std::size_t head = static_cast<std::size_t>(now_ms / step_ms) % (count + width);
    if (cfg.reverse) head = (count + width - 1) - head;
    for (std::size_t tail = 0; tail < width; ++tail) {
        const long pos = cfg.reverse ? static_cast<long>(head + tail) : static_cast<long>(head) - static_cast<long>(tail);
        if (pos < 0 || pos >= static_cast<long>(count)) continue;
        const uint8_t fade = static_cast<uint8_t>(((width - tail) * 255) / width);
        pixels[pos] = blend(scaled(cfg.secondary, cfg.intensity), scaled(cfg.primary, cfg.intensity), fade);
    }
}

void clock_spin(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, scaled(cfg.secondary, cfg.intensity));
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    std::size_t head = static_cast<std::size_t>((now_ms % period) * count / period);
    if (cfg.reverse) head = count - 1 - head;
    for (std::size_t n = 0; n < width; ++n) {
        const std::size_t pos = cfg.reverse ? (head + n) % count : (head + count - (n % count)) % count;
        pixels[pos] = scaled(cfg.primary, cfg.intensity);
    }
}

void plasma(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const float time = static_cast<float>(now_ms) / static_cast<float>(std::max<uint32_t>(cfg.speed_ms, 200));
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(directed_index(i, count, cfg.reverse)) / static_cast<float>(std::max<std::size_t>(count, 1));
        const float wave = std::sin((x * 2.7f + time) * kTau) +
                           std::sin((x * 5.1f - time * 0.73f) * kTau) +
                           std::sin((x * 1.3f + time * 1.37f) * kTau);
        const uint8_t mix = static_cast<uint8_t>(std::clamp((wave + 3.0f) / 6.0f, 0.0f, 1.0f) * 255.0f);
        pixels[i] = scaled(blend(cfg.secondary, cfg.primary, mix), cfg.intensity);
    }
}

void fire(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const uint32_t step_ms = std::max<uint32_t>(35, cfg.speed_ms / 16);
    const uint32_t frame = static_cast<uint32_t>(now_ms / step_ms);
    for (std::size_t i = 0; i < count; ++i) {
        const uint32_t h = hash32(static_cast<uint32_t>(i) * 0x27d4eb2dU ^ frame * 0x165667b1U);
        const uint8_t noise = static_cast<uint8_t>((h >> 8) & 0xFF);
        const uint8_t floor = static_cast<uint8_t>(255 - cfg.density);
        const uint8_t mix = std::max<uint8_t>(floor, noise);
        pixels[i] = scaled(blend(cfg.secondary, cfg.primary, mix), cfg.intensity);
    }
}

void palette_cycle(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    const float phase = static_cast<float>(now_ms % period) / static_cast<float>(period);
    fill(pixels, count, scaled(palette_sample(cfg, phase), cfg.intensity));
}

void palette_chase(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const std::size_t width = std::max<std::size_t>(2, std::min<std::size_t>(cfg.width, 64));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 250);
    const float shift = static_cast<float>(now_ms % period) / static_cast<float>(period);
    const float cycle_px = static_cast<float>(width * palette_count(cfg));
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t p = directed_index(i, count, cfg.reverse);
        const float phase = static_cast<float>(p) / cycle_px + shift;
        pixels[i] = scaled(palette_sample(cfg, phase), cfg.intensity);
    }
}

void crossflow(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, rgb::Color{});
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 300);
    const std::size_t travel = count + width;
    std::size_t step = static_cast<std::size_t>((now_ms % period) * travel / period);
    if (cfg.reverse) step = travel - 1 - step;
    for (std::size_t i = 0; i < count; ++i) {
        uint8_t a = 0, b = 0;
        const long da = static_cast<long>(i) - static_cast<long>(step);
        const long db = static_cast<long>(count - 1 - i) - static_cast<long>(step);
        if (da <= 0 && da > -static_cast<long>(width)) {
            const long remaining = static_cast<long>(width) + da;
            a = static_cast<uint8_t>((remaining * 255) / static_cast<long>(width));
        }
        if (db <= 0 && db > -static_cast<long>(width)) {
            const long remaining = static_cast<long>(width) + db;
            b = static_cast<uint8_t>((remaining * 255) / static_cast<long>(width));
        }
        rgb::Color c{};
        if (a) c = blend(c, cfg.primary, a);
        if (b) c = blend(c, cfg.secondary, b);
        pixels[i] = scaled(c, cfg.intensity);
    }
}

void wraith(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const uint8_t n = palette_count(cfg);
    const rgb::Color background = n > 1 ? palette_color(cfg, 1) : rgb::Color{};
    fill(pixels, count, scaled(background, cfg.intensity));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 500);
    const float t = static_cast<float>(now_ms % (period * 8ULL)) / static_cast<float>(period);
    const std::size_t ghosts = 1 + (cfg.density * 4u / 255u);
    for (std::size_t g = 0; g < ghosts; ++g) {
        float phase = std::fmod(t * (0.35f + 0.11f * static_cast<float>(g)) + 0.217f * static_cast<float>(g), 1.0f);
        if (cfg.reverse) phase = 1.0f - phase;
        const float center = phase * static_cast<float>(count - 1);
        const float radius = std::max<float>(2.0f, static_cast<float>(count) * (0.08f + 0.02f * static_cast<float>(g)));
        const std::size_t color_index = n > 2 ? (g == 0 ? 0 : 2 + ((g - 1) % (n - 2))) : 0;
        const rgb::Color ghost_color = palette_color(cfg, color_index);
        for (std::size_t i = 0; i < count; ++i) {
            const float dist = std::fabs(static_cast<float>(i) - center);
            if (dist >= radius) continue;
            const float q = 1.0f - dist / radius;
            const uint8_t mix = static_cast<uint8_t>(q * q * static_cast<float>(96 + cfg.density / 2));
            pixels[i] = blend(pixels[i], scaled(ghost_color, cfg.intensity), mix);
        }
    }
}

void heartbeat(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 500);
    const float p = static_cast<float>(now_ms % period) / static_cast<float>(period);
    auto pulse = [](float x, float center, float width) {
        const float d = std::fabs(x - center) / width;
        return d >= 1.0f ? 0.0f : (1.0f - d) * (1.0f - d);
    };
    const float level = std::clamp(pulse(p, 0.16f, 0.09f) + 0.82f * pulse(p, 0.34f, 0.10f), 0.0f, 1.0f);
    const uint8_t mix = static_cast<uint8_t>(level * 255.0f);
    fill(pixels, count, scaled(blend(cfg.secondary, cfg.primary, mix), cfg.intensity));
}

void aurora(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const float time = static_cast<float>(now_ms) / static_cast<float>(std::max<uint32_t>(cfg.speed_ms, 400));
    const float detail = 1.4f + 3.2f * (static_cast<float>(cfg.density) / 255.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(directed_index(i, count, cfg.reverse)) / static_cast<float>(std::max<std::size_t>(count - 1, 1));
        const float warp = 0.10f * std::sin((x * detail - time * 0.21f) * kTau) +
                           0.05f * std::sin((x * (detail * 2.1f) + time * 0.13f) * kTau);
        const float phase = x * 0.72f + time * 0.08f + warp;
        const float glow = 0.34f + 0.66f * (0.5f + 0.5f * std::sin((x * 1.8f + time * 0.17f) * kTau));
        const uint8_t level = static_cast<uint8_t>(std::clamp(glow, 0.0f, 1.0f) * cfg.intensity);
        pixels[i] = scaled(palette_sample(cfg, phase), level);
    }
}

void nebula(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const float time = static_cast<float>(now_ms) / static_cast<float>(std::max<uint32_t>(cfg.speed_ms, 500));
    const float density = 1.0f + 4.0f * (static_cast<float>(cfg.density) / 255.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(directed_index(i, count, cfg.reverse)) / static_cast<float>(std::max<std::size_t>(count - 1, 1));
        const float cloud = std::sin((x * density + time * 0.11f) * kTau) +
                            0.65f * std::sin((x * density * 2.37f - time * 0.07f) * kTau) +
                            0.38f * std::sin((x * density * 4.11f + time * 0.043f) * kTau);
        const float norm = std::clamp((cloud + 2.03f) / 4.06f, 0.0f, 1.0f);
        const float dim = 0.22f + 0.78f * norm;
        pixels[i] = scaled(palette_sample(cfg, norm + time * 0.025f, false),
                           static_cast<uint8_t>(dim * cfg.intensity));
    }
}

void spectrum_bands(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const std::size_t band = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, 64));
    const uint32_t step_ms = std::max<uint32_t>(20, cfg.speed_ms / std::max<std::size_t>(count, 1));
    const std::size_t shift = static_cast<std::size_t>(now_ms / step_ms);
    const uint8_t n = palette_count(cfg);
    for (std::size_t i = 0; i < count; ++i) {
        std::size_t p = directed_index(i, count, cfg.reverse);
        p = cfg.reverse ? (p + count - (shift % count)) % count : (p + shift) % count;
        const std::size_t color_index = (p / band) % n;
        pixels[i] = scaled(palette_color(cfg, color_index), cfg.intensity);
    }
}

void collision(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, rgb::Color{});
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, std::max<std::size_t>(count / 2, 1)));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 500);
    const float phase = static_cast<float>(now_ms % period) / static_cast<float>(period);
    const float travel = phase < 0.58f ? phase / 0.58f : 1.0f;
    const float left_head = travel * static_cast<float>(count - 1) * 0.5f;
    const float right_head = static_cast<float>(count - 1) - left_head;
    const rgb::Color left = palette_color(cfg, cfg.reverse ? 1 : 0);
    const rgb::Color right = palette_color(cfg, cfg.reverse ? 0 : 1);

    for (std::size_t i = 0; i < count; ++i) {
        const float dl = std::fabs(static_cast<float>(i) - left_head);
        const float dr = std::fabs(static_cast<float>(i) - right_head);
        rgb::Color c{};
        if (dl < static_cast<float>(width)) {
            const uint8_t q = static_cast<uint8_t>((1.0f - dl / static_cast<float>(width)) * 255.0f);
            c = blend(c, left, q);
        }
        if (dr < static_cast<float>(width)) {
            const uint8_t q = static_cast<uint8_t>((1.0f - dr / static_cast<float>(width)) * 255.0f);
            c = blend(c, right, q);
        }
        pixels[i] = scaled(c, cfg.intensity);
    }

    if (phase >= 0.50f && phase <= 0.78f) {
        const float center = static_cast<float>(count - 1) * 0.5f;
        const float q = 1.0f - std::fabs(phase - 0.64f) / 0.14f;
        const float radius = static_cast<float>(width) * (1.0f + 2.0f * q);
        const rgb::Color flash = palette_color(cfg, std::min<std::size_t>(2, palette_count(cfg) - 1));
        for (std::size_t i = 0; i < count; ++i) {
            const float d = std::fabs(static_cast<float>(i) - center);
            if (d >= radius) continue;
            const uint8_t mix = static_cast<uint8_t>(std::clamp(q * (1.0f - d / radius), 0.0f, 1.0f) * 255.0f);
            pixels[i] = blend(pixels[i], scaled(flash, cfg.intensity), mix);
        }
    }
}

void starlight(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const uint8_t n = palette_count(cfg);
    const rgb::Color base = scaled(palette_color(cfg, 0), static_cast<uint8_t>(cfg.intensity / 10));
    const uint32_t density_gate = std::max<uint32_t>(8, cfg.density);
    for (std::size_t i = 0; i < count; ++i) {
        const uint32_t seed = hash32(static_cast<uint32_t>(i) * 0x9e3779b9U + 0x51ed2705U);
        if ((seed & 0xFFu) >= density_gate) {
            pixels[i] = base;
            continue;
        }
        const uint32_t period = std::max<uint32_t>(450, cfg.speed_ms / 2 + 450 + ((seed >> 8) & 0x3FFu));
        const uint32_t offset = (seed >> 18) % period;
        const float p = static_cast<float>((now_ms + offset) % period) / static_cast<float>(period);
        const float pulse = std::max(0.0f, 1.0f - std::fabs(p - 0.5f) * 2.0f);
        const float shaped = pulse * pulse;
        const std::size_t ci = n > 1 ? 1 + ((seed >> 5) % (n - 1)) : 0;
        const uint8_t level = static_cast<uint8_t>(shaped * cfg.intensity);
        pixels[i] = blend(base, scaled(palette_color(cfg, ci), level), static_cast<uint8_t>(shaped * 255.0f));
    }
}

void lava(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const float time = static_cast<float>(now_ms) / static_cast<float>(std::max<uint32_t>(cfg.speed_ms, 450));
    const float density = 0.8f + 3.5f * (static_cast<float>(cfg.density) / 255.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(directed_index(i, count, cfg.reverse)) / static_cast<float>(std::max<std::size_t>(count - 1, 1));
        const float a = std::sin((x * density - time * 0.13f) * kTau);
        const float b = std::sin((x * density * 1.73f + time * 0.09f) * kTau);
        const float c = std::sin((x * density * 0.47f - time * 0.04f) * kTau);
        float heat = (a + 0.65f * b + 0.35f * c + 2.0f) / 4.0f;
        heat = std::clamp(heat, 0.0f, 1.0f);
        heat *= heat;
        pixels[i] = scaled(palette_sample(cfg, heat, false),
                           static_cast<uint8_t>((0.30f + 0.70f * heat) * cfg.intensity));
    }
}

void pulse_train(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, rgb::Color{});
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, 32));
    const std::size_t spacing = width * 3;
    const uint32_t step_ms = std::max<uint32_t>(20, cfg.speed_ms / std::max<std::size_t>(count, 1));
    const std::size_t shift = static_cast<std::size_t>(now_ms / step_ms);
    const uint8_t n = palette_count(cfg);
    for (std::size_t i = 0; i < count; ++i) {
        std::size_t p = directed_index(i, count, cfg.reverse) + shift;
        const std::size_t local = p % spacing;
        if (local >= width) continue;
        const uint8_t fade = static_cast<uint8_t>(((width - local) * 255) / width);
        const std::size_t pulse_index = (p / spacing) % n;
        pixels[i] = scaled(palette_color(cfg, pulse_index), scale8(cfg.intensity, fade));
    }
}

void dual_orbit(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    fill(pixels, count, rgb::Color{});
    const std::size_t width = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, count));
    const uint32_t period = std::max<uint32_t>(cfg.speed_ms, 300);
    const uint8_t n = palette_count(cfg);
    std::size_t shift = static_cast<std::size_t>((now_ms % period) * count / period);
    if (cfg.reverse) shift = count - 1 - shift;
    for (uint8_t orb = 0; orb < n; ++orb) {
        const std::size_t head = (shift + (static_cast<std::size_t>(orb) * count) / n) % count;
        for (std::size_t tail = 0; tail < width; ++tail) {
            const std::size_t pos = cfg.reverse ? (head + tail) % count : (head + count - (tail % count)) % count;
            const uint8_t fade = static_cast<uint8_t>(((width - tail) * 255) / width);
            pixels[pos] = blend(pixels[pos], scaled(palette_color(cfg, orb), cfg.intensity), fade);
        }
    }
}

void synthwave(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const std::size_t band = std::max<std::size_t>(1, std::min<std::size_t>(cfg.width, 32));
    const uint32_t step_ms = std::max<uint32_t>(25, cfg.speed_ms / std::max<std::size_t>(count, 1));
    const std::size_t shift = static_cast<std::size_t>(now_ms / step_ms);
    const uint8_t n = palette_count(cfg);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t p0 = directed_index(i, count, cfg.reverse);
        const std::size_t p = p0 + shift;
        const std::size_t slot = p / band;
        if ((slot & 1u) != 0) {
            pixels[i] = rgb::Color{};
            continue;
        }
        const std::size_t ci = (slot / 2) % n;
        const std::size_t local = p % band;
        const float edge = band > 1 ? static_cast<float>(local) / static_cast<float>(band - 1) : 0.5f;
        const float glow = 0.55f + 0.45f * std::sin(edge * 3.14159265359f);
        pixels[i] = scaled(palette_color(cfg, ci), static_cast<uint8_t>(glow * cfg.intensity));
    }
}

void color_waves(rgb::Color* pixels, std::size_t count, const rgb::EffectConfig& cfg, uint64_t now_ms)
{
    if (!count) return;
    const float time = static_cast<float>(now_ms) / static_cast<float>(std::max<uint32_t>(cfg.speed_ms, 300));
    const float density = 1.0f + 3.0f * (static_cast<float>(cfg.density) / 255.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = static_cast<float>(directed_index(i, count, cfg.reverse)) / static_cast<float>(std::max<std::size_t>(count - 1, 1));
        const float wave = 0.5f + 0.25f * std::sin((x * density + time * 0.18f) * kTau) +
                           0.25f * std::sin((x * density * 0.53f - time * 0.11f) * kTau);
        pixels[i] = scaled(palette_sample(cfg, wave + time * 0.035f), cfg.intensity);
    }
}

} // namespace

namespace effects {

std::size_t count() { return sizeof(kEffects) / sizeof(kEffects[0]); }

const Descriptor& descriptor(std::size_t index)
{
    return kEffects[index < count() ? index : 0];
}

const Descriptor* find(rgb::EffectId id)
{
    for (const auto& effect : kEffects) if (effect.id == id) return &effect;
    return nullptr;
}

const Descriptor* find_key(const char* value)
{
    if (!value) return nullptr;
    for (const auto& effect : kEffects) if (std::strcmp(effect.key, value) == 0) return &effect;
    return nullptr;
}

bool uses(const Descriptor& descriptor, Control control)
{
    return (descriptor.controls & static_cast<uint8_t>(control)) != 0;
}

const char* key(rgb::EffectId id)
{
    if (const auto* d = find(id)) return d->key;
    return "solid";
}

void render(rgb::EffectId id, rgb::Color* pixels, std::size_t pixel_count,
            const rgb::EffectConfig& config, uint64_t now_ms)
{
    switch (id) {
        case rgb::EffectId::Breathe:       breathe(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::ColorWipe:     color_wipe(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Larson:        larson(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Rainbow:       rainbow(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::TheaterChase:  theater_chase(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Twinkle:       twinkle(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Comet:         comet(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Meteor:        meteor(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::ClockSpin:     clock_spin(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Plasma:        plasma(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Fire:          fire(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::PaletteCycle:  palette_cycle(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::PaletteChase:  palette_chase(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Crossflow:     crossflow(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Wraith:        wraith(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Heartbeat:     heartbeat(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Aurora:        aurora(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Nebula:        nebula(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::SpectrumBands: spectrum_bands(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Collision:     collision(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Starlight:     starlight(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Lava:          lava(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::PulseTrain:    pulse_train(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::DualOrbit:     dual_orbit(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Synthwave:     synthwave(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::ColorWaves:    color_waves(pixels, pixel_count, config, now_ms); break;
        case rgb::EffectId::Solid:
        default: solid(pixels, pixel_count, config, now_ms); break;
    }
}

} // namespace effects
