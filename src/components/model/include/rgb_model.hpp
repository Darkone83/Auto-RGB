#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rgb {

constexpr std::size_t kChannelCount = 8;
constexpr std::size_t kMaxLedsPerChannel = 300;
constexpr std::size_t kMaxZones = 8;
constexpr std::size_t kMaxSegmentsPerZone = 8;
constexpr std::size_t kBuiltInSceneCount = 6;
constexpr std::size_t kCustomSceneCount = 4;
constexpr std::size_t kMaxScenes = kBuiltInSceneCount + kCustomSceneCount;
constexpr std::size_t kMaxPaletteColors = 5;
constexpr uint32_t kConfigMagic = 0x52474231; // "RGB1"
constexpr uint16_t kConfigSchemaVersion = 10;

struct Color {
    uint8_t r{0};
    uint8_t g{0};
    uint8_t b{0};
};

enum class PixelOrder : uint8_t {
    GRB = 0,
    RGB,
    BRG,
    RBG,
    GBR,
    BGR,
};

enum class EffectId : uint8_t {
    // Keep the first four numeric values stable for Rev-A NVS compatibility.
    Solid = 0,
    Breathe = 1,
    Rainbow = 2,
    Meteor = 3,
    ColorWipe,
    Larson,
    TheaterChase,
    Twinkle,
    Comet,
    ClockSpin,
    Plasma,
    Fire,
    PaletteCycle,
    PaletteChase,
    Crossflow,
    Wraith,
    Heartbeat,
    Aurora,
    Nebula,
    SpectrumBands,
    Collision,
    Starlight,
    Lava,
    PulseTrain,
    DualOrbit,
    Synthwave,
    ColorWaves,
};

struct EffectConfig {
    EffectId id{EffectId::Solid};
    Color primary{96, 0, 160};
    Color secondary{0, 0, 0};
    uint16_t speed_ms{1800};
    uint8_t width{12};
    uint8_t intensity{255};
    uint8_t density{96};
    bool reverse{false};
    uint8_t palette_count{2}; // primary + secondary, up to 5 colors total
    std::array<Color, kMaxPaletteColors - 2> extra_colors{{
        Color{255, 0, 96}, Color{0, 96, 255}, Color{255, 96, 0}
    }};
};

struct ChannelConfig {
    char name[24]{};
    uint16_t led_count{0};
    uint8_t brightness{255};
    PixelOrder order{PixelOrder::GRB};
    bool enabled{false};
    bool reversed{false};
};

struct SegmentConfig {
    uint8_t channel{0};
    uint16_t start{0};
    uint16_t length{0}; // 0 = remainder of channel from start
    bool reversed{false};
};

struct ZoneConfig {
    char name[24]{};
    bool enabled{true};
    uint8_t segment_count{0};
    std::array<SegmentConfig, kMaxSegmentsPerZone> segments{};
};

struct ZoneStyle {
    bool enabled{true};
    uint8_t brightness{255};
    EffectConfig effect{};
};

struct SceneConfig {
    char name[24]{};
    bool enabled{true};
    std::array<ZoneStyle, kMaxZones> zones{}; // user-created zones only
    ZoneStyle all_lighting{};               // independent global effect for every output
};

struct SystemConfig {
    char device_name[32]{};
    char ap_ssid[33]{};
    char ap_password[65]{}; // retained for schema compatibility; SoftAP is always open
    uint8_t master_brightness{160};
    uint8_t active_scene{0};
};

struct PowerConfig {
    bool master_enabled{true};
    uint16_t power_on_fade_ms{1500};
};

enum class StartupPowerMode : uint8_t {
    RestoreLast = 0,
    AlwaysOn,
    AlwaysOff,
};

constexpr uint8_t kStartupSceneLast = 0xFF;

struct RemoteControlConfig {
    bool enabled{false};
    uint8_t oled_contrast{127};        // user-facing OLED brightness, SSD1306 contrast register
    uint16_t oled_timeout_s{45};       // 0 = never sleep
    bool oled_invert{false};
    bool oled_auto_dim{false};         // uses ALS when available; falls back to oled_contrast
    uint8_t oled_screensaver_minutes{0}; // 0 = off, otherwise 1..5; requires healthy RTC
    uint8_t reserved{0};
};

static_assert(sizeof(RemoteControlConfig) == 8, "RemoteControlConfig size must remain schema-compatible");


enum class AlsSensitivity : uint8_t {
    Low = 0,
    Normal,
    High,
    VeryHigh,
};

struct AccessoryConfig {
    bool rtc_enabled{false};
    bool als_enabled{false};
    AlsSensitivity als_sensitivity{AlsSensitivity::Normal};
    std::array<uint8_t, 5> reserved{};
};

struct StartupConfig {
    StartupPowerMode power_mode{StartupPowerMode::RestoreLast};
    uint8_t scene{kStartupSceneLast}; // 0xFF = restore last active scene
    std::array<uint8_t, 2> reserved{};
};

struct DeviceConfig {
    uint32_t magic{kConfigMagic};
    uint16_t schema_version{kConfigSchemaVersion};
    SystemConfig system{};
    std::array<ChannelConfig, kChannelCount> channels{};
    uint8_t zone_count{0};
    std::array<ZoneConfig, kMaxZones> zones{};
    uint8_t scene_count{0};
    std::array<SceneConfig, kMaxScenes> scenes{};
    PowerConfig power{}; // appended in schema v2 so schema v1 remains prefix-compatible
    RemoteControlConfig remote{}; // appended in schema v4
    StartupConfig startup{}; // appended in schema v5
    AccessoryConfig accessories{}; // appended in schema v7
};

static_assert(std::is_trivially_copyable_v<DeviceConfig>, "DeviceConfig must remain NVS-blob safe");

} // namespace rgb
