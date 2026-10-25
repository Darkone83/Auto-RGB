#include "remote_control.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "accessories.hpp"
#include "auto_rgb_oled_logo.hpp"
#include "oled_clock_face.hpp"
#include "oled_scene_visualizer.hpp"
#include "board_pins.hpp"
#include "config_store.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "shared_i2c.hpp"
#include "effects.hpp"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "remote";
constexpr uint32_t kI2cHz = 100000;
constexpr uint8_t kOledAddrPrimary = 0x3C;
constexpr uint8_t kOledAddrSecondary = 0x3D;
constexpr int kI2cTimeoutMs = 100;
constexpr int kPollMs = 10;
constexpr int64_t kEncoderSampleUs = 2000; // sample independently of slow OLED I2C writes
constexpr int kDisplayRefreshMs = 150;
constexpr int kRetryDisplayMs = 5000;
// A sleeping OLED does not generate normal framebuffer traffic. Check that it
// still ACKs, so removal/reconnection is detected even after display sleep.
constexpr int kOledPresenceCheckMs = 5000;
constexpr uint8_t kOledPresenceFailureLimit = 2;
constexpr int kIdleHomeMs = 45000; // leave enough time for installation/menu adjustments
constexpr int kAmbientHomeMs = 12000;
constexpr int kSplashIntroFrames = 10;
constexpr int kSplashRevealFrames = 11;
// Three distinct panels: legible clock, branding, then a motion-driven scene card.
constexpr int kSaverClockMs = 6000;
constexpr int kSaverLogoMs = 4400;
constexpr int kSaverSceneMs = 7200;
constexpr int kSaverSceneFrameMs = 180; // ~5.5 fps, gentle on 100-kHz shared I2C
constexpr int kScreensaverFadeMs = 220;
constexpr int kSwitchDebounceMs = 30;
constexpr int kMenuExitHoldMs = 1000;
constexpr int kPowerHoldMs = 3000; // only on Home; never triggers while exiting menus
constexpr int kEncoderTransitionsPerDetent = 2; // EC11E: 30 detents / 15 pulses

constexpr int kWidth = 128;
constexpr int kHeight = 64;
constexpr int kPages = kHeight / 8;
constexpr std::size_t kFramebufferSize = kWidth * kPages;
constexpr uint8_t kVisibleRows = 4;

TaskHandle_t s_task = nullptr;
i2c_master_dev_handle_t s_oled = nullptr;
uint8_t s_oled_addr = 0;
std::array<uint8_t, kFramebufferSize> s_fb{};
std::array<uint8_t, kFramebufferSize> s_last_sent_fb{};
bool s_fb_cache_valid = false;

bool s_display_ready = false;
uint32_t s_bus_generation_seen = 0;
bool s_oled_awake = true;
bool s_last_enabled = false;
int64_t s_last_display_attempt_ms = 0;
int64_t s_last_presence_check_ms = 0;
uint8_t s_presence_failures = 0;
int64_t s_last_render_ms = 0;
int64_t s_last_input_ms = 0;
uint8_t s_applied_contrast = 0xFF;
bool s_applied_invert = false;
bool s_splash_pending = false;
bool s_screensaver_active = false;
enum class SaverPanel : uint8_t { Clock, Logo, Scene };
SaverPanel s_screensaver_panel = SaverPanel::Clock;
int64_t s_screensaver_panel_ms = 0;
int64_t s_screensaver_started_ms = 0;

// Encoder sampling runs on esp_timer's callback task. OLED I2C transfers
// block the UI task, but must not make rotary steps disappear.
portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t s_encoder_timer = nullptr;
bool s_encoder_timer_started = false;
uint8_t s_encoder_prev = 0;
int s_encoder_accum = 0;
int s_encoder_steps = 0;
bool s_switch_raw = true;
bool s_switch_stable = true;
bool s_switch_activity = false;
int64_t s_switch_change_ms = 0;
int64_t s_press_start_ms = 0;
bool s_hold_action_sent = false;
bool s_ignore_wake_press = false; // Do not activate a menu item on wake release.
bool s_press_started_on_home = true;

char s_notice[24]{};
int64_t s_notice_until_ms = 0;

enum class Screen : uint8_t {
    Home,
    Main,
    Scenes,
    Zones,
    ZoneMenu,
    ZoneBrightness,
    ZoneEffect,
    ZoneColor,
    ZoneSpeed,
    ZoneDirection,
    QuickColor,
    QuickColorColor,
    MasterBrightness,
    Power,
    QuickSettings,
    StartupPower,
    StartupScene,
    SoftStart,
    Display,
    DisplayBrightness,
    DisplayAutoDim,
    DisplaySleep,
    DisplayInvert,
    DisplayScreensaver,
    Info,
};

enum class ButtonEvent : uint8_t { None, Short, Long, VeryLong };

Screen s_screen = Screen::Home;
uint8_t s_main_index = 0;
uint8_t s_scene_index = 0;
uint8_t s_zone_index = 0;
uint8_t s_zone_menu_index = 0;
uint8_t s_quick_color_index = 0;
int s_quick_target = -1; // -1 = all zones
uint8_t s_quick_menu_index = 0;
uint8_t s_settings_index = 0;
uint8_t s_display_index = 0;
uint8_t s_info_page = 0;
uint8_t s_startup_power_candidate = 0;
uint8_t s_startup_scene_candidate = 0; // 0 = last, 1..N = scene
uint16_t s_soft_start_candidate = 1500;
uint8_t s_master_brightness_candidate = 160;
bool s_power_candidate = true;
uint8_t s_display_brightness_candidate = 127;
uint8_t s_display_timeout_index = 3;
bool s_display_auto_dim_candidate = false;
bool s_display_invert_candidate = false;
uint8_t s_display_screensaver_candidate = 0;

struct Glyph {
    char c;
    std::array<uint8_t, 5> col;
};

constexpr Glyph kFont[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}}, {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'.', {0x00,0x60,0x60,0x00,0x00}}, {':', {0x00,0x36,0x36,0x00,0x00}},
    {'%', {0x63,0x13,0x08,0x64,0x63}}, {'>', {0x00,0x22,0x14,0x08,0x00}},
    {'*', {0x14,0x08,0x3E,0x08,0x14}}, {'/', {0x20,0x10,0x08,0x04,0x02}},
    {'=', {0x14,0x14,0x14,0x14,0x14}}, {'<', {0x08,0x14,0x22,0x41,0x00}},
    {'+', {0x08,0x08,0x3E,0x08,0x08}}, {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}}, {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}}, {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}}, {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}}, {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}}, {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}}, {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}}, {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}}, {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}}, {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}}, {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}}, {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}}, {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x03,0x04,0x78,0x04,0x03}}, {'Z', {0x61,0x51,0x49,0x45,0x43}},
};

struct NamedColor {
    const char* name;
    rgb::Color color;
};

constexpr std::array<NamedColor, 10> kColors{{
    {"PURPLE",   {128, 0, 190}},
    {"BURGUNDY", {128, 0, 48}},
    {"RED",      {255, 0, 0}},
    {"MAGENTA",  {255, 0, 120}},
    {"BLUE",     {0, 70, 255}},
    {"CYAN",     {0, 220, 255}},
    {"GREEN",    {0, 255, 80}},
    {"AMBER",    {255, 90, 0}},
    {"WHITE",    {255, 255, 255}},
    {"WARM",     {255, 150, 90}},
}};

constexpr std::array<uint16_t, 8> kTimeouts{{0, 15, 30, 45, 60, 90, 120, 300}};
constexpr std::array<uint8_t, 6> kScreensaverMinutes{{0, 1, 2, 3, 4, 5}};

int64_t now_ms()
{
    return esp_timer_get_time() / 1000;
}

// Centralise every way the clock saver ends (input, sleep, RTC loss,
// display disable and hot-plug recovery). Do not alter LED/RTC ownership.
void reset_screensaver()
{
    s_screensaver_active = false;
    s_screensaver_panel = SaverPanel::Clock;
    s_screensaver_panel_ms = 0;
    s_screensaver_started_ms = 0;
}

const std::array<uint8_t, 5>& glyph_for(char c)
{
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    for (const auto& g : kFont) if (g.c == c) return g.col;
    return kFont[0].col;
}

void clear_fb() { s_fb.fill(0); }

void draw_auto_rgb_logo()
{
    static_assert(auto_rgb_oled_logo::kBitmap.size() == kFramebufferSize,
                  "Auto-RGB OLED logo must be 128x64 / 1024 bytes");
    s_fb = auto_rgb_oled_logo::kBitmap;
}

void set_pixel(int x, int y, bool on = true)
{
    if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
    const std::size_t idx = static_cast<std::size_t>(x + (y / 8) * kWidth);
    const uint8_t mask = static_cast<uint8_t>(1u << (y & 7));
    if (on) s_fb[idx] |= mask;
    else s_fb[idx] &= static_cast<uint8_t>(~mask);
}

void draw_char(int x, int y, char c, int scale = 1)
{
    const auto& g = glyph_for(c);
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            if ((g[col] & (1u << row)) == 0) continue;
            for (int dx = 0; dx < scale; ++dx)
                for (int dy = 0; dy < scale; ++dy)
                    set_pixel(x + col * scale + dx, y + row * scale + dy);
        }
    }
}

void draw_text(int x, int y, const char* text, int scale = 1, int max_chars = 21)
{
    if (!text) return;
    int count = 0;
    while (*text && count < max_chars) {
        draw_char(x, y, *text++, scale);
        x += 6 * scale;
        ++count;
    }
}

void draw_centered(int y, const char* text, int scale = 1)
{
    if (!text) return;
    const int max_chars = kWidth / (6 * scale);
    const int len = std::min<int>(static_cast<int>(std::strlen(text)), max_chars);
    const int width = len * 6 * scale - scale;
    draw_text(std::max(0, (kWidth - width) / 2), y, text, scale, len);
}

// Simple vector primitives keep every UI animation native to the 1-bit OLED.
void draw_line(int x0, int y0, int x1, int y1)
{
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        set_pixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        const int twice = 2 * err;
        if (twice > -dy) { err -= dy; x0 += sx; }
        if (twice < dx) { err += dx; y0 += sy; }
    }
}

void draw_outline(int x, int y, int width, int height)
{
    if (width < 2 || height < 2) return;
    draw_line(x, y, x + width - 1, y);
    draw_line(x, y + height - 1, x + width - 1, y + height - 1);
    draw_line(x, y, x, y + height - 1);
    draw_line(x + width - 1, y, x + width - 1, y + height - 1);
}

void fill_rect(int x, int y, int width, int height)
{
    for (int yy = y; yy < y + height; ++yy)
        for (int xx = x; xx < x + width; ++xx) set_pixel(xx, yy);
}

void invert_rect(int x, int y, int width, int height)
{
    for (int yy = y; yy < y + height; ++yy)
        for (int xx = x; xx < x + width; ++xx)
            if (xx >= 0 && xx < kWidth && yy >= 0 && yy < kHeight) {
                const std::size_t i = static_cast<std::size_t>(xx + (yy / 8) * kWidth);
                s_fb[i] ^= static_cast<uint8_t>(1u << (yy & 7));
            }
}

void draw_meter(int x, int y, int width, int height, uint8_t value)
{
    draw_outline(x, y, width, height);
    const int fill = ((width - 4) * static_cast<int>(value) + 127) / 255;
    if (fill > 0) fill_rect(x + 2, y + 2, fill, height - 4);
}

void draw_row(int y, bool selected, const char* left, const char* right = nullptr)
{
    if (selected) draw_text(1, y, ">");
    const int right_len = right ? std::min<int>(static_cast<int>(std::strlen(right)), 7) : 0;
    const int right_x = 126 - right_len * 6;
    const int left_limit = right ? std::max(1, (right_x - 13) / 6) : 19;
    draw_text(9, y, left, 1, left_limit);
    if (right) draw_text(right_x, y, right, 1, right_len);
    if (selected) invert_rect(0, y - 1, kWidth, 9); // high contrast selection bar
}

void draw_header(const char* title, bool dirty = false)
{
    draw_text(2, 1, title, 1, dirty ? 17 : 20);
    if (dirty) draw_text(119, 1, "*");
    draw_line(0, 10, kWidth - 1, 10);
}

void draw_menu_footer()
{
    draw_line(0, 54, kWidth - 1, 54);
    draw_centered(57, "HOLD 1S: HOME");
}

template <typename LabelFn>
void draw_list(const char* title, uint8_t count, uint8_t selected, bool dirty, LabelFn label)
{
    clear_fb();
    draw_header(title, dirty);
    if (!count) draw_centered(27, "NONE AVAILABLE");
    else {
        selected = std::min<uint8_t>(selected, static_cast<uint8_t>(count - 1));
        const uint8_t top = selected >= kVisibleRows
            ? static_cast<uint8_t>(selected - kVisibleRows + 1) : 0;
        for (uint8_t row = 0; row < kVisibleRows && top + row < count; ++row) {
            const uint8_t idx = static_cast<uint8_t>(top + row);
            draw_row(15 + row * 10, idx == selected, label(idx));
        }
    }
    draw_menu_footer();
}

esp_err_t oled_tx(const uint8_t* data, std::size_t len)
{
    if (!s_oled) return ESP_ERR_INVALID_STATE;
    return shared_i2c::transmit(s_oled, data, len, kI2cTimeoutMs);
}

esp_err_t oled_commands(const uint8_t* cmds, std::size_t len)
{
    if (len > 31) return ESP_ERR_INVALID_SIZE;
    uint8_t packet[32]{};
    packet[0] = 0x00;
    std::memcpy(packet + 1, cmds, len);
    return oled_tx(packet, len + 1);
}

// Only transfer changed page/column ranges. This avoids filling the 100-kHz
// shared bus on static menus and keeps the 2-ms encoder sampler responsive.
esp_err_t oled_flush()
{
    uint8_t packet[33]{};
    packet[0] = 0x40;
    for (int page = 0; page < kPages; ++page) {
        const std::size_t base = static_cast<std::size_t>(page) * kWidth;
        int first = kWidth;
        int last = -1;
        for (int x = 0; x < kWidth; ++x) {
            if (!s_fb_cache_valid || s_fb[base + x] != s_last_sent_fb[base + x]) {
                first = std::min(first, x);
                last = x;
            }
        }
        if (last < 0) continue;
        const uint8_t window[] = {0x21, static_cast<uint8_t>(first), static_cast<uint8_t>(last),
                                  0x22, static_cast<uint8_t>(page), static_cast<uint8_t>(page)};
        esp_err_t err = oled_commands(window, sizeof(window));
        if (err != ESP_OK) { s_fb_cache_valid = false; return err; }
        for (int col = first; col <= last; col += 32) {
            const int n = std::min(32, last - col + 1);
            std::memcpy(packet + 1, s_fb.data() + base + col, static_cast<std::size_t>(n));
            err = oled_tx(packet, static_cast<std::size_t>(n) + 1);
            if (err != ESP_OK) { s_fb_cache_valid = false; return err; }
        }
        std::memcpy(s_last_sent_fb.data() + base, s_fb.data() + base, kWidth);
    }
    s_fb_cache_valid = true;
    return ESP_OK;
}

void oled_set_awake(bool awake)
{
    if (!s_oled || s_oled_awake == awake) return;
    const uint8_t cmd = awake ? 0xAF : 0xAE;
    if (oled_commands(&cmd, 1) == ESP_OK) s_oled_awake = awake;
}

esp_err_t oled_set_brightness(uint8_t brightness)
{
    if (!s_oled) return ESP_ERR_INVALID_STATE;
    if (brightness == s_applied_contrast) return ESP_OK;
    const uint8_t cmds[] = {0x81, brightness};
    const esp_err_t err = oled_commands(cmds, sizeof(cmds));
    if (err == ESP_OK) s_applied_contrast = brightness;
    return err;
}

uint8_t effective_display_brightness(const rgb::RemoteControlConfig& cfg,
                                     const accessories::Snapshot& acc)
{
    // ALS is an LED-output sensor only. The OLED always uses its own manual
    // brightness setting; retain the old config field for schema stability.
    (void)acc;
    return cfg.oled_contrast;
}

void apply_display_config(const rgb::RemoteControlConfig& cfg,
                          const accessories::Snapshot& acc)
{
    if (!s_oled) return;
    const uint8_t target = effective_display_brightness(cfg, acc);
    (void)oled_set_brightness(target);

    if (cfg.oled_invert != s_applied_invert) {
        const uint8_t cmd = cfg.oled_invert ? 0xA7 : 0xA6;
        if (oled_commands(&cmd, 1) == ESP_OK) s_applied_invert = cfg.oled_invert;
    }
}

void fade_to(uint8_t target, int duration_ms)
{
    if (!s_oled || duration_ms <= 0) {
        (void)oled_set_brightness(target);
        return;
    }

    const int start = s_applied_contrast == 0xFF ? 0 : s_applied_contrast;
    constexpr int kSteps = 16;
    for (int step = 1; step <= kSteps; ++step) {
        const int value = start + ((static_cast<int>(target) - start) * step) / kSteps;
        if (oled_set_brightness(static_cast<uint8_t>(std::clamp(value, 0, 255))) != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(std::max(1, duration_ms / kSteps)));
    }
}

void display_shutdown()
{
    if (s_oled) {
        const uint8_t off = 0xAE;
        (void)oled_commands(&off, 1);
        (void)shared_i2c::remove_device(s_oled);
        s_oled = nullptr;
    }
    s_display_ready = false;
    s_fb_cache_valid = false;
    s_oled_awake = true;
    s_oled_addr = 0;
    s_applied_contrast = 0xFF;
    s_applied_invert = false;
    s_splash_pending = false;
    reset_screensaver();
    s_ignore_wake_press = false;
    s_last_presence_check_ms = 0;
    s_presence_failures = 0;
}

// A transient bus outage is not a reason to remove/re-add the I2C device
// handle. Keep it registered, wait for bus recovery, then restore in place.
void display_defer_retry()
{
    s_display_ready = false;
    s_fb_cache_valid = false;
    s_splash_pending = false;
    reset_screensaver();
    s_last_display_attempt_ms = now_ms();
    s_presence_failures = 0;
}

// Reset/reconfigure the SSD1306 over an existing device handle. ESP-IDF's
// bus reset does not invalidate device handles, so there is no need to remove
// and re-add the OLED every time the bus recovers.
esp_err_t oled_restore_controller(const rgb::RemoteControlConfig& display_cfg)
{
    if (!s_oled) return ESP_ERR_INVALID_STATE;
    static constexpr uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x00, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF
    };
    const esp_err_t err = oled_commands(init_seq, sizeof(init_seq));
    if (err != ESP_OK) return err;
    s_display_ready = true;
    s_fb_cache_valid = false;
    s_oled_awake = true;
    s_applied_contrast = 0;
    s_applied_invert = false;
    if (display_cfg.oled_invert) {
        const uint8_t cmd = 0xA7;
        const esp_err_t invert_err = oled_commands(&cmd, 1);
        if (invert_err != ESP_OK) return invert_err;
        s_applied_invert = true;
    }
    return ESP_OK;
}

// Declared here because the regular screen renderer is defined below the
// hardware initialization code. Reuse it to transfer a real visible frame,
// rather than logging success after flushing an all-zero framebuffer.
void render(const config::RemoteSnapshot& st);

esp_err_t display_start(const rgb::RemoteControlConfig& display_cfg)
{
    // Retain the ESP-IDF device handle across an OLED power interruption,
    // but confirm that the device actually ACKs before any 'restored' log.
    if (s_oled) {
        if (!shared_i2c::available()) return ESP_ERR_TIMEOUT;
        const esp_err_t presence = shared_i2c::probe(s_oled_addr, kI2cTimeoutMs);
        if (presence != ESP_OK) {
            display_defer_retry();
            return presence;
        }
        esp_err_t restore_err = oled_restore_controller(display_cfg);
        if (restore_err != ESP_OK) {
            display_defer_retry();
            return restore_err;
        }

        // Reconnected after a physical OLED power interruption: the previous
        // inactivity timer may have expired while unplugged. Without clearing
        // it, the freshly initialized display is turned off immediately.
        const int64_t recovered_ms = now_ms();
        s_last_input_ms = recovered_ms;
        reset_screensaver();
        s_splash_pending = false; // no splash replay during recovery
        restore_err = oled_set_brightness(display_cfg.oled_contrast);
        if (restore_err != ESP_OK) {
            display_defer_retry();
            return restore_err;
        }
        // This performs a full framebuffer transfer because the restore
        // invalidated its cache. render() marks the display unavailable on
        // any write error, so success is not reported prematurely.
        render(config::remote_snapshot());
        if (!s_display_ready) return ESP_ERR_TIMEOUT;

        s_last_render_ms = now_ms();
        s_last_presence_check_ms = s_last_render_ms;
        s_presence_failures = 0;
        ESP_LOGI(TAG, "SSD1306 reconnected: ACK, init, contrast and frame sent");
        return ESP_OK;
    }

    display_shutdown();

    esp_err_t err = shared_i2c::ensure_started();
    if (err != ESP_OK) return err;

    for (uint8_t addr : {kOledAddrPrimary, kOledAddrSecondary}) {
        if (shared_i2c::probe(addr, kI2cTimeoutMs) == ESP_OK) {
            s_oled_addr = addr;
            break;
        }
    }
    if (!s_oled_addr) {
        display_shutdown();
        return ESP_ERR_NOT_FOUND;
    }

    err = shared_i2c::add_device(s_oled_addr, kI2cHz, &s_oled);
    if (err != ESP_OK) {
        display_shutdown();
        return err;
    }

    err = oled_restore_controller(display_cfg);
    if (err != ESP_OK) {
        display_defer_retry();
        return err;
    }
    clear_fb();
    err = oled_flush();
    if (err != ESP_OK) {
        display_defer_retry();
        return err;
    }

    s_splash_pending = true;
    s_last_presence_check_ms = now_ms();
    s_presence_failures = 0;
    ESP_LOGI(TAG, "SSD1306 detected at 0x%02X", s_oled_addr);
    return ESP_OK;
}

// Validate quadrature transitions. Invalid (two-bit) jumps are ignored.
void encoder_sample_locked(uint8_t current)
{
    static constexpr int8_t table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0,
    };
    if (current == s_encoder_prev) return;
    const uint8_t idx = static_cast<uint8_t>((s_encoder_prev << 2) | current);
    s_encoder_prev = current;
    s_encoder_accum += table[idx & 0x0F];
    if (s_encoder_accum >= kEncoderTransitionsPerDetent) {
        s_encoder_accum -= kEncoderTransitionsPerDetent;
        if (s_encoder_steps < 1024) ++s_encoder_steps;
    } else if (s_encoder_accum <= -kEncoderTransitionsPerDetent) {
        s_encoder_accum += kEncoderTransitionsPerDetent;
        if (s_encoder_steps > -1024) --s_encoder_steps;
    }
}

void encoder_poll_timer(void*)
{
    const uint8_t current = static_cast<uint8_t>(
        (gpio_get_level(board::kRemoteEncoderAGpio) << 1) |
         gpio_get_level(board::kRemoteEncoderBGpio));
    portENTER_CRITICAL(&s_encoder_lock);
    encoder_sample_locked(current);
    portEXIT_CRITICAL(&s_encoder_lock);
}

void encoder_init()
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask = (1ULL << board::kRemoteEncoderAGpio) |
                       (1ULL << board::kRemoteEncoderBGpio) |
                       (1ULL << board::kRemoteEncoderSwitchGpio);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    (void)gpio_config(&cfg);

    portENTER_CRITICAL(&s_encoder_lock);
    s_encoder_prev = static_cast<uint8_t>((gpio_get_level(board::kRemoteEncoderAGpio) << 1) |
                                           gpio_get_level(board::kRemoteEncoderBGpio));
    s_encoder_accum = 0;
    s_encoder_steps = 0;
    portEXIT_CRITICAL(&s_encoder_lock);

    // A 2-ms timer captures knob transitions even when the remote task is
    // waiting for 1024 OLED data bytes to cross the 100-kHz shared bus.
    esp_timer_create_args_t args{};
    args.callback = encoder_poll_timer;
    args.name = "rgb_encoder";
    esp_err_t err = esp_timer_create(&args, &s_encoder_timer);
    if (err == ESP_OK) err = esp_timer_start_periodic(s_encoder_timer, kEncoderSampleUs);
    s_encoder_timer_started = err == ESP_OK;
    if (!s_encoder_timer_started) {
        ESP_LOGW(TAG, "Encoder timer unavailable (%s); using slow task polling", esp_err_to_name(err));
        if (s_encoder_timer) { (void)esp_timer_delete(s_encoder_timer); s_encoder_timer = nullptr; }
    }

    s_switch_raw = gpio_get_level(board::kRemoteEncoderSwitchGpio) != 0;
    s_switch_stable = s_switch_raw;
    s_switch_change_ms = now_ms();
}

int encoder_step()
{
    if (!s_encoder_timer_started) encoder_poll_timer(nullptr);
    portENTER_CRITICAL(&s_encoder_lock);
    const int steps = s_encoder_steps;
    s_encoder_steps = 0;
    portEXIT_CRITICAL(&s_encoder_lock);
    return steps;
}

void encoder_reset_pending()
{
    portENTER_CRITICAL(&s_encoder_lock);
    s_encoder_accum = 0;
    s_encoder_steps = 0;
    portEXIT_CRITICAL(&s_encoder_lock);
}

ButtonEvent button_event()
{
    s_switch_activity = false;
    const bool raw = gpio_get_level(board::kRemoteEncoderSwitchGpio) != 0;
    const int64_t now = now_ms();
    if (raw != s_switch_raw) {
        s_switch_raw = raw;
        s_switch_change_ms = now;
        s_switch_activity = true;
    }
    // Debounce both edges, then identify a press. A hold fires while the
    // button is held, exactly once; releasing it cannot also select a menu item.
    if (raw != s_switch_stable && now - s_switch_change_ms >= kSwitchDebounceMs) {
        s_switch_stable = raw;
        s_switch_activity = true;
        if (!s_switch_stable) {
            s_press_start_ms = now;
            s_press_started_on_home = (s_screen == Screen::Home);
            s_hold_action_sent = s_ignore_wake_press;
        } else if (s_ignore_wake_press) {
            // A press used to wake or dismiss the saver is entirely consumed.
            s_ignore_wake_press = false;
            s_hold_action_sent = true;
        } else if (!s_hold_action_sent) {
            return ButtonEvent::Short;
        }
    }
    if (raw && s_switch_stable) s_ignore_wake_press = false;
    if (!s_switch_stable && !s_hold_action_sent) {
        const int64_t threshold = s_press_started_on_home ? kPowerHoldMs : kMenuExitHoldMs;
        if (now - s_press_start_ms >= threshold) {
            s_hold_action_sent = true;
            return s_press_started_on_home ? ButtonEvent::VeryLong : ButtonEvent::Long;
        }
    }
    return ButtonEvent::None;
}

void set_notice(const char* text)
{
    std::snprintf(s_notice, sizeof(s_notice), "%s", text ? text : "");
    s_notice_until_ms = now_ms() + 900;
}

void format_brightness(uint8_t value, char (&out)[16])
{
    const unsigned pct = static_cast<unsigned>((static_cast<unsigned>(value) * 100u + 127u) / 255u);
    std::snprintf(out, sizeof(out), "%u%%", pct);
}

void format_seconds(uint16_t ms, char (&out)[16])
{
    if (ms == 0) std::snprintf(out, sizeof(out), "OFF");
    else std::snprintf(out, sizeof(out), "%u.%us", static_cast<unsigned>(ms / 1000), static_cast<unsigned>((ms % 1000) / 100));
}

const char* startup_power_name(rgb::StartupPowerMode mode)
{
    switch (mode) {
        case rgb::StartupPowerMode::AlwaysOn: return "ALWAYS ON";
        case rgb::StartupPowerMode::AlwaysOff: return "ALWAYS OFF";
        case rgb::StartupPowerMode::RestoreLast:
        default: return "RESTORE";
    }
}

uint8_t nearest_timeout_index(uint16_t seconds)
{
    uint8_t best = 0;
    uint16_t best_diff = 0xFFFF;
    for (uint8_t i = 0; i < kTimeouts.size(); ++i) {
        const uint16_t a = kTimeouts[i] > seconds ? kTimeouts[i] - seconds : seconds - kTimeouts[i];
        if (a < best_diff) { best = i; best_diff = a; }
    }
    return best;
}

uint8_t color_index(rgb::Color color)
{
    uint8_t best = 0;
    uint32_t best_dist = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < kColors.size(); ++i) {
        const int dr = static_cast<int>(color.r) - kColors[i].color.r;
        const int dg = static_cast<int>(color.g) - kColors[i].color.g;
        const int db = static_cast<int>(color.b) - kColors[i].color.b;
        const uint32_t dist = static_cast<uint32_t>(dr * dr + dg * dg + db * db);
        if (dist < best_dist) { best = i; best_dist = dist; }
    }
    return best;
}

uint8_t effect_index(rgb::EffectId id)
{
    for (std::size_t i = 0; i < effects::count(); ++i) if (effects::descriptor(i).id == id) return static_cast<uint8_t>(i);
    return 0;
}

void render_notice()
{
    clear_fb();
    draw_centered(20, s_notice, 1);
}

// Abstract signal tracer only: visual accent, never claims to mirror a
// particular LED effect or physical output. No RTC is needed for idle mode.
void draw_signal_tracer(int top, int64_t clock_ms, bool powered)
{
    draw_line(3, top + 5, 124, top + 5);
    if (!powered) return;
    const int phase = static_cast<int>((clock_ms / 180) % 21);
    for (int lane = 0; lane < 8; ++lane) {
        const int x = 7 + lane * 15;
        const int wave = (lane * 3 + phase) % 21;
        const int height = 2 + (wave < 10 ? wave : 20 - wave) / 2;
        draw_line(x, top + 5 - height, x, top + 5 + height);
        if ((lane + phase) % 4 == 0) set_pixel(x + 3, top + 1);
    }
}

void render_home(const config::RemoteSnapshot& st)
{
    clear_fb();
    const uint8_t scene = st.scene_count ? std::min<uint8_t>(st.active_scene, st.scene_count - 1) : 0;
    const char* scene_name = st.scene_count ? st.scene_names[scene].data() : "NO SCENE";
    char bright[16]{};
    format_brightness(st.master_brightness, bright);
    const bool ambient = now_ms() - s_last_input_ms >= kAmbientHomeMs;

    if (!ambient) {
        draw_text(2, 1, "AUTO-RGB");
        if (st.dirty) draw_text(88, 1, "*");
        draw_text(st.master_power ? 107 : 102, 1, st.master_power ? "ON" : "OFF");
        draw_line(0, 10, 127, 10);
        draw_centered(14, scene_name);
        draw_signal_tracer(29, now_ms(), st.master_power);
        draw_text(2, 44, "BRI");
        draw_meter(26, 43, 64, 10, st.master_power ? st.master_brightness : 0);
        draw_text(97, 44, bright);
        draw_line(0, 54, 127, 54);
        draw_centered(57, "TURN:BRI PRESS:MENU");
    } else {
        // Quiet, screen-independent idle display: a slow-moving highlight,
        // prominent branding and the real current scene/brightness state.
        draw_line(9, 4, 23, 4);
        draw_line(104, 4, 118, 4);
        draw_centered(4, "AUTO-RGB", 2);
        draw_signal_tracer(30, now_ms() / 2, st.master_power);
        draw_centered(46, scene_name);
        draw_text(2, 57, st.master_power ? "ON" : "OFF");
        if (st.dirty) draw_text(31, 57, "*");
        draw_text(78, 57, "BRI");
        draw_text(100, 57, bright);
    }
}

bool rtc_screensaver_ready(const rgb::RemoteControlConfig& cfg,
                           const accessories::Snapshot& acc)
{
    return cfg.oled_screensaver_minutes >= 1 &&
           cfg.oled_screensaver_minutes <= 5 &&
           acc.rtc_enabled && acc.rtc_present && acc.rtc_time_valid;
}

// The animation is a tiny scene-aware *visual motif*, not a second LED
// renderer. It reads the current scene and a representative zone effect only;
// nothing is written to the effect engine, LEDs, config, or NVS.
void render_scene_saver(const config::RemoteSnapshot& st, int64_t ms)
{
    clear_fb();
    const uint8_t scene = st.scene_count
        ? std::min<uint8_t>(st.active_scene, static_cast<uint8_t>(st.scene_count - 1)) : 0;
    const char* raw_name = st.scene_count ? st.scene_names[scene].data() : "NO SCENE";
    char scene_name[25]{};
    std::size_t length = 0;
    while (length < 24 && raw_name[length] != '\0') {
        scene_name[length] = raw_name[length];
        ++length;
    }
    if (!length) { std::memcpy(scene_name, "UNTITLED", 9); length = 8; }

    uint8_t shown_zone = 0;
    bool have_zone = false;
    uint8_t active_zones = 0;
    const uint8_t zone_count = std::min<uint8_t>(st.zone_count, rgb::kMaxZones);
    for (uint8_t z = 0; z < zone_count; ++z) {
        const auto& style = st.active_zone_styles[z];
        if (!style.enabled || style.brightness == 0) continue;
        if (!have_zone) { shown_zone = z; have_zone = true; }
        ++active_zones;
    }
    const rgb::EffectConfig& effect = st.active_zone_styles[shown_zone].effect;
    const auto motion = oled_scene_visualizer::classify(effect.id);
    const auto* info = have_zone ? effects::find(effect.id) : nullptr;
    const uint32_t elapsed = static_cast<uint32_t>(std::max<int64_t>(0, ms - s_screensaver_panel_ms));
    const uint32_t phase = elapsed / oled_scene_visualizer::frame_duration_ms(effect.speed_ms);

    // A restrained instrument-panel header with a tiny moving status pip.
    draw_text(3, 2, "NOW PLAYING");
    const char* status = st.master_power ? "ON" : "OFF";
    draw_text(st.master_power ? 111 : 105, 2, status);
    draw_line(0, 11, 127, 11);
    if (st.master_power) {
        const int pip = 79 + static_cast<int>((elapsed / 175u) % 17u);
        fill_rect(pip, 4, 3, 3);
    }

    // Center short names large; scroll names beyond the 19-glyph OLED window.
    draw_line(2, 15, 2, 31);
    draw_line(125, 15, 125, 31);
    if (length <= 10) {
        draw_centered(17, scene_name, 2);
    } else if (length <= 19) {
        draw_centered(20, scene_name);
    } else {
        char window[20]{};
        const std::size_t wrap = length + 3u;
        const std::size_t offset = (elapsed / 640u) % wrap;
        for (std::size_t i = 0; i < 19u; ++i) {
            const std::size_t p = (offset + i) % wrap;
            window[i] = p < length ? scene_name[p] : ' ';
        }
        draw_centered(20, window);
    }
    draw_line(6, 34, 121, 34);

    // 15 segmented pseudo-LED columns; intentionally monochrome, subtle, and
    // effect-aware (wave, chase, starfield, flame, pulse, dual orbit). The
    // whole 17-pixel-high band can update without redrawing the static header.
    const bool powered = st.master_power && have_zone && st.master_brightness > 0;
    for (uint8_t column = 0; column < oled_scene_visualizer::kBars; ++column) {
        const int x = 7 + static_cast<int>(column) * 8;
        set_pixel(x + 1, 53); // quiet rail visible even when master is off
        if (!powered) continue;
        const uint8_t height = oled_scene_visualizer::height(
            motion, column, phase, effect.reverse);
        for (int dy = 0; dy < height; dy += 3) {
            const int y = 52 - dy;
            fill_rect(x, y - 1, 5, 2);
        }
        if (height >= 13 && ((column + phase) & 3u) == 0u)
            set_pixel(x + 2, 36);
    }

    draw_line(0, 55, 127, 55);
    if (!st.master_power) draw_text(4, 57, "LIGHTING OFF");
    else if (!have_zone) draw_text(4, 57, "NO ACTIVE ZONES");
    else draw_text(4, 57, info ? info->name : "SCENE FX", 1, 16);
    char zones[8]{};
    std::snprintf(zones, sizeof(zones), "%uZ", static_cast<unsigned>(active_zones));
    draw_text(114, 57, zones, 1, 2);
}

int saver_panel_duration_ms()
{
    switch (s_screensaver_panel) {
        case SaverPanel::Clock: return kSaverClockMs;
        case SaverPanel::Logo: return kSaverLogoMs;
        case SaverPanel::Scene: return kSaverSceneMs;
    }
    return kSaverClockMs;
}

void render_screensaver(const config::RemoteSnapshot& st,
                        const accessories::Snapshot& acc)
{
    if (s_screensaver_panel == SaverPanel::Logo) {
        // Preserve the existing detailed monochrome logo without scaling it.
        draw_auto_rgb_logo();
        return;
    }
    if (s_screensaver_panel == SaverPanel::Scene) {
        render_scene_saver(st, now_ms());
        return;
    }

    const oled_clock_face::Time time{
        static_cast<unsigned>(acc.year),
        static_cast<unsigned>(acc.month),
        static_cast<unsigned>(acc.day),
        static_cast<unsigned>(acc.hour),
        static_cast<unsigned>(acc.minute),
        static_cast<unsigned>(acc.second),
    };
    const oled_clock_face::Face face = oled_clock_face::format(time);

    clear_fb();
    draw_text(4, 2, "AUTO-RGB");
    draw_line(58, 6, 96, 6);
    draw_text(104, 2, "RTC");
    draw_line(3, 12, 124, 12);

    draw_line(3, 19, 3, 32);
    draw_line(124, 19, 124, 32);
    draw_text(7, 18, face.clock, 2, 8);
    draw_text(111, 23, face.period, 1, 2);

    draw_line(3, 38, 124, 38);
    draw_centered(44, face.date);
    draw_line(3, 54, 124, 54);
    draw_text(4, 56, "RTC LIVE");

    const int64_t elapsed_ms = std::max<int64_t>(0, now_ms() - s_screensaver_panel_ms);
    const int progress = std::min<int>(6, static_cast<int>(elapsed_ms / 1000) + 1);
    for (int i = 0; i < 6; ++i) {
        const int x = 88 + i * 6;
        draw_outline(x, 57, 4, 4);
        if (i < progress) fill_rect(x + 1, 58, 2, 2);
    }
}
void render_splash_intro(int frame)
{
    clear_fb();
    // Circuit rails approach the title from both sides while the ignition
    // tracer grows. Fixed monochrome geometry, no color/alpha assets needed.
    const int reach = std::min(43, frame * 5);
    draw_line(0, 11, reach, 11);
    draw_line(127 - reach, 11, 127, 11);
    draw_line(0, 51, reach, 51);
    draw_line(127 - reach, 51, 127, 51);
    draw_centered(15, "WELCOME TO");
    draw_centered(26, "AUTO-RGB", 2);
    for (int i = 0; i < 13; ++i) {
        const int x = 12 + i * 8;
        draw_outline(x, 43, 5, 4);
        if (i <= frame * 13 / (kSplashIntroFrames - 1)) fill_rect(x + 1, 44, 3, 2);
    }
    for (int x = 0; x <= reach; x += 8) {
        set_pixel(x, 8);
        set_pixel(127 - x, 54);
    }
}

void render_splash_reveal(int frame)
{
    const int half_width = (frame + 1) * 64 / kSplashRevealFrames;
    draw_auto_rgb_logo();
    const int lo = 64 - half_width;
    const int hi = 63 + half_width;
    // Mask hidden columns rather than enlarging the logo or discarding detail.
    for (int page = 0; page < kPages; ++page) {
        const std::size_t base = static_cast<std::size_t>(page) * kWidth;
        for (int x = 0; x < kWidth; ++x)
            if (x < lo || x > hi) s_fb[base + x] = 0;
    }
    if (lo > 0) draw_line(lo, 5, lo, 58);
    if (hi < 127) draw_line(hi, 5, hi, 58);
}

void run_startup_splash(const config::RemoteSnapshot& st,
                        const accessories::Snapshot& acc)
{
    if (!s_display_ready || !s_oled) return;
    const uint8_t target = effective_display_brightness(st.remote, acc);
    (void)oled_set_brightness(0);
    bool interrupted = false;
    for (int frame = 0; frame < kSplashIntroFrames; ++frame) {
        if (gpio_get_level(board::kRemoteEncoderSwitchGpio) == 0) { interrupted = true; break; }
        render_splash_intro(frame);
        if (oled_flush() != ESP_OK) return;
        const int contrast = static_cast<int>(target) * (frame + 1) / kSplashIntroFrames;
        (void)oled_set_brightness(static_cast<uint8_t>(contrast));
        vTaskDelay(pdMS_TO_TICKS(45));
    }
    if (!interrupted) {
        vTaskDelay(pdMS_TO_TICKS(200));
        for (int frame = 0; frame < kSplashRevealFrames; ++frame) {
            if (gpio_get_level(board::kRemoteEncoderSwitchGpio) == 0) { interrupted = true; break; }
            render_splash_reveal(frame);
            if (oled_flush() != ESP_OK) return;
            vTaskDelay(pdMS_TO_TICKS(32));
        }
    }
    if (!interrupted) vTaskDelay(pdMS_TO_TICKS(600));
    s_last_input_ms = now_ms();
    if (interrupted) {
        // The button that skips the splash must not also open the menu.
        s_switch_raw = false;
        s_switch_stable = false;
        s_switch_change_ms = s_last_input_ms;
        s_hold_action_sent = true;
        s_ignore_wake_press = true;
    }
    render_home(st);
    if (oled_flush() != ESP_OK) return;
    (void)oled_set_brightness(target);
    s_splash_pending = false;
    s_last_render_ms = s_last_input_ms;
    encoder_reset_pending();
}


const char* main_item(uint8_t i)
{
    static constexpr const char* items[] = {
        "SCENES", "ZONES", "QUICK COLOR", "BRIGHTNESS", "POWER",
        "QUICK SETTINGS", "DISPLAY", "INFO", "SAVE CHANGES", "HOME"
    };
    return items[i];
}

void render_main(const config::RemoteSnapshot& st)
{
    draw_list("MAIN MENU", 10, s_main_index, st.dirty, [](uint8_t i) { return main_item(i); });
}

void render_scenes(const config::RemoteSnapshot& st)
{
    draw_list("SCENES", static_cast<uint8_t>(st.scene_count + 1), s_scene_index, st.dirty,
              [&](uint8_t i) { return i == st.scene_count ? "BACK" : st.scene_names[i].data(); });
}

void render_zones(const config::RemoteSnapshot& st)
{
    draw_list("ZONES", static_cast<uint8_t>(st.zone_count + 1), s_zone_index, st.dirty,
              [&](uint8_t i) { return i == st.zone_count ? "BACK" : st.zone_names[i].data(); });
}

const char* zone_menu_item(uint8_t i)
{
    static constexpr const char* items[] = {"BRIGHTNESS", "EFFECT", "COLOR", "SPEED", "DIRECTION", "ENABLED", "BACK"};
    return items[i];
}

void render_zone_menu(const config::RemoteSnapshot& st)
{
    clear_fb();
    draw_header(st.zone_count ? st.zone_names[s_zone_index].data() : "ZONE", st.dirty);
    const auto& style = st.active_zone_styles[s_zone_index];
    uint8_t top = s_zone_menu_index >= kVisibleRows ? static_cast<uint8_t>(s_zone_menu_index - kVisibleRows + 1) : 0;
    for (uint8_t row = 0; row < kVisibleRows && top + row < 7; ++row) {
        const uint8_t i = top + row;
        const char* right = nullptr;
        char value[16]{};
        if (i == 0) { format_brightness(style.brightness, value); right = value; }
        else if (i == 1) { const auto* d = effects::find(style.effect.id); right = d ? d->name : "?"; }
        else if (i == 3) { std::snprintf(value, sizeof(value), "%ums", style.effect.speed_ms); right = value; }
        else if (i == 4) right = style.effect.reverse ? "REV" : "FWD";
        else if (i == 5) right = style.enabled ? "ON" : "OFF";
        draw_row(15 + row * 10, i == s_zone_menu_index, zone_menu_item(i), right);
    }
    draw_menu_footer();
}

void render_value_screen(const char* title, const char* value, const char* hint = "TURN  PRESS DONE")
{
    clear_fb();
    draw_header(title);
    draw_centered(22, value, std::strlen(value) > 10 ? 1 : 2);
    draw_centered(44, hint);
    draw_menu_footer();
}

void render_zone_brightness(const config::RemoteSnapshot& st)
{
    char value[16]{}; format_brightness(st.active_zone_styles[s_zone_index].brightness, value);
    render_value_screen("ZONE BRIGHTNESS", value);
}

void render_zone_effect(const config::RemoteSnapshot& st)
{
    const auto* d = effects::find(st.active_zone_styles[s_zone_index].effect.id);
    render_value_screen("ZONE EFFECT", d ? d->name : "UNKNOWN");
}

void render_zone_color(const config::RemoteSnapshot& st)
{
    const uint8_t idx = color_index(st.active_zone_styles[s_zone_index].effect.primary);
    render_value_screen("ZONE COLOR", kColors[idx].name);
}

void render_zone_speed(const config::RemoteSnapshot& st)
{
    char value[16]{};
    std::snprintf(value, sizeof(value), "%ums", st.active_zone_styles[s_zone_index].effect.speed_ms);
    render_value_screen("ZONE SPEED", value);
}

void render_zone_direction(const config::RemoteSnapshot& st)
{
    render_value_screen("DIRECTION", st.active_zone_styles[s_zone_index].effect.reverse ? "REVERSE" : "FORWARD");
}

const char* quick_target_name(const config::RemoteSnapshot& st)
{
    if (s_quick_target < 0) return "ALL";
    if (s_quick_target < st.zone_count) return st.zone_names[static_cast<std::size_t>(s_quick_target)].data();
    return "ALL";
}

void render_quick_color(const config::RemoteSnapshot& st)
{
    clear_fb(); draw_header("QUICK COLOR", st.dirty);
    const char* labels[] = {"TARGET", "COLOR", "APPLY", "BACK"};
    for (uint8_t i = 0; i < 4; ++i) {
        const char* right = nullptr;
        if (i == 0) right = quick_target_name(st);
        else if (i == 1) right = kColors[s_quick_color_index].name;
        draw_row(15 + i * 10, i == s_quick_menu_index, labels[i], right);
    }
    draw_menu_footer();
}

void render_quick_color_select(const config::RemoteSnapshot& st)
{
    draw_list("SELECT COLOR", static_cast<uint8_t>(kColors.size()), s_quick_color_index, st.dirty,
              [](uint8_t i) { return kColors[i].name; });
}

void render_master_brightness()
{
    char value[16]{}; format_brightness(s_master_brightness_candidate, value);
    render_value_screen("MASTER BRIGHT", value);
}

void render_power()
{
    render_value_screen("MASTER POWER", s_power_candidate ? "ON" : "OFF", "TURN  PRESS APPLY");
}

void render_quick_settings(const config::RemoteSnapshot& st)
{
    clear_fb(); draw_header("QUICK SETTINGS", st.dirty);
    const char* labels[] = {"START POWER", "START SCENE", "SOFT START", "SAVE", "REVERT", "BACK"};
    uint8_t top = s_settings_index >= kVisibleRows ? static_cast<uint8_t>(s_settings_index - kVisibleRows + 1) : 0;
    for (uint8_t row = 0; row < kVisibleRows && top + row < 6; ++row) {
        const uint8_t i = top + row;
        char value[16]{};
        const char* right = nullptr;
        if (i == 0) right = startup_power_name(st.startup.power_mode);
        else if (i == 1) right = st.startup.scene == rgb::kStartupSceneLast ? "LAST" : st.scene_names[st.startup.scene].data();
        else if (i == 2) { format_seconds(st.power_on_fade_ms, value); right = value; }
        draw_row(15 + row * 10, i == s_settings_index, labels[i], right);
    }
    draw_menu_footer();
}

void render_startup_power()
{
    render_value_screen("START POWER", startup_power_name(static_cast<rgb::StartupPowerMode>(s_startup_power_candidate)));
}

void render_startup_scene(const config::RemoteSnapshot& st)
{
    const char* value = s_startup_scene_candidate == 0 ? "LAST" : st.scene_names[s_startup_scene_candidate - 1].data();
    render_value_screen("START SCENE", value);
}

void render_soft_start()
{
    char value[16]{}; format_seconds(s_soft_start_candidate, value);
    render_value_screen("SOFT START", value);
}

enum class DisplayItem : uint8_t {
    Brightness,
    AutoDim,
    Sleep,
    Invert,
    Screensaver,
    Back,
};

uint8_t display_item_count(const accessories::Snapshot& acc)
{
    return (acc.rtc_enabled && acc.rtc_present && acc.rtc_time_valid) ? 5 : 4;
}

DisplayItem display_item_at(uint8_t index, const accessories::Snapshot& acc)
{
    const bool rtc_ready = acc.rtc_enabled && acc.rtc_present && acc.rtc_time_valid;
    if (index == 0) return DisplayItem::Brightness;
    if (index == 1) return DisplayItem::Sleep;
    if (index == 2) return DisplayItem::Invert;
    if (rtc_ready && index == 3) return DisplayItem::Screensaver;
    return DisplayItem::Back;
}

void format_idle_timeout(uint16_t seconds, char (&out)[16])
{
    if (seconds == 0) std::snprintf(out, sizeof(out), "NEVER");
    else if (seconds % 60 == 0) std::snprintf(out, sizeof(out), "%um", static_cast<unsigned>(seconds / 60));
    else std::snprintf(out, sizeof(out), "%us", static_cast<unsigned>(seconds));
}

void render_display(const config::RemoteSnapshot& st)
{
    const auto acc = accessories::snapshot();
    const uint8_t count = display_item_count(acc);
    if (s_display_index >= count) s_display_index = static_cast<uint8_t>(count - 1);

    clear_fb();
    draw_header("DISPLAY", st.dirty);

    const uint8_t top = s_display_index >= kVisibleRows
        ? static_cast<uint8_t>(s_display_index - kVisibleRows + 1)
        : 0;

    for (uint8_t row = 0; row < kVisibleRows && top + row < count; ++row) {
        const uint8_t i = static_cast<uint8_t>(top + row);
        char value[16]{};
        const char* label = "";
        const char* right = nullptr;

        switch (display_item_at(i, acc)) {
            case DisplayItem::Brightness:
                label = "BRIGHTNESS";
                std::snprintf(value, sizeof(value), "%u", st.remote.oled_contrast);
                right = value;
                break;
            case DisplayItem::AutoDim:
                label = "AUTO DIM";
                right = st.remote.oled_auto_dim ? "ON" : "OFF";
                break;
            case DisplayItem::Sleep:
                label = "SLEEP";
                format_idle_timeout(st.remote.oled_timeout_s, value);
                right = value;
                break;
            case DisplayItem::Invert:
                label = "INVERT";
                right = st.remote.oled_invert ? "YES" : "NO";
                break;
            case DisplayItem::Screensaver:
                label = "SCREENSAVER";
                if (st.remote.oled_screensaver_minutes == 0) right = "OFF";
                else {
                    std::snprintf(value, sizeof(value), "%um",
                                  static_cast<unsigned>(st.remote.oled_screensaver_minutes));
                    right = value;
                }
                break;
            case DisplayItem::Back:
                label = "BACK";
                break;
        }
        draw_row(15 + row * 10, i == s_display_index, label, right);
    }
    draw_menu_footer();
}

void render_display_brightness()
{
    char value[16]{};
    std::snprintf(value, sizeof(value), "%u", s_display_brightness_candidate);
    render_value_screen("OLED BRIGHTNESS", value);
}

void render_display_auto_dim()
{
    render_value_screen("AUTO DIM", s_display_auto_dim_candidate ? "ON" : "OFF");
}

void render_display_sleep()
{
    char value[16]{};
    format_idle_timeout(kTimeouts[s_display_timeout_index], value);
    render_value_screen("OLED SLEEP", value);
}

void render_display_invert()
{
    render_value_screen("OLED INVERT", s_display_invert_candidate ? "YES" : "NO");
}

void render_display_screensaver()
{
    char value[16]{};
    if (s_display_screensaver_candidate == 0) std::snprintf(value, sizeof(value), "OFF");
    else std::snprintf(value, sizeof(value), "%um", static_cast<unsigned>(s_display_screensaver_candidate));
    render_value_screen("SCREENSAVER", value);
}

void render_info(const config::RemoteSnapshot& st)
{
    clear_fb(); draw_header("SYSTEM INFO");
    if (s_info_page == 0) {
        const esp_app_desc_t* app = esp_app_get_description();
        char line[22]{};
        std::snprintf(line, sizeof(line), "FW %.15s", app ? app->version : "?"); draw_text(0, 16, line);
        std::snprintf(line, sizeof(line), "CH %u/8  ZONES %u", st.enabled_channels, st.zone_count); draw_text(0, 28, line);
        draw_text(0, 40, "SCENE"); draw_text(36, 40, st.scene_count ? st.scene_names[st.active_scene].data() : "NONE", 1, 14);
        draw_text(0, 52, "TURN FOR MORE");
    } else {
        char line[22]{};
        std::snprintf(line, sizeof(line), "OLED 0x%02X", s_oled_addr); draw_text(0, 16, line);
        draw_text(0, 28, "AP 192.168.4.1");
        draw_text(0, 40, st.dirty ? "UNSAVED CHANGES" : "CONFIG SAVED");
        draw_text(0, 52, "HOME: HOLD 1S");
    }
}

void render(const config::RemoteSnapshot& st)
{
    if (!s_display_ready || !s_oled_awake) return;
    if (now_ms() < s_notice_until_ms) {
        render_notice();
    } else {
        switch (s_screen) {
            case Screen::Home: render_home(st); break;
            case Screen::Main: render_main(st); break;
            case Screen::Scenes: render_scenes(st); break;
            case Screen::Zones: render_zones(st); break;
            case Screen::ZoneMenu: render_zone_menu(st); break;
            case Screen::ZoneBrightness: render_zone_brightness(st); break;
            case Screen::ZoneEffect: render_zone_effect(st); break;
            case Screen::ZoneColor: render_zone_color(st); break;
            case Screen::ZoneSpeed: render_zone_speed(st); break;
            case Screen::ZoneDirection: render_zone_direction(st); break;
            case Screen::QuickColor: render_quick_color(st); break;
            case Screen::QuickColorColor: render_quick_color_select(st); break;
            case Screen::MasterBrightness: render_master_brightness(); break;
            case Screen::Power: render_power(); break;
            case Screen::QuickSettings: render_quick_settings(st); break;
            case Screen::StartupPower: render_startup_power(); break;
            case Screen::StartupScene: render_startup_scene(st); break;
            case Screen::SoftStart: render_soft_start(); break;
            case Screen::Display: render_display(st); break;
            case Screen::DisplayBrightness: render_display_brightness(); break;
            case Screen::DisplayAutoDim: render_display_auto_dim(); break;
            case Screen::DisplaySleep: render_display_sleep(); break;
            case Screen::DisplayInvert: render_display_invert(); break;
            case Screen::DisplayScreensaver: render_display_screensaver(); break;
            case Screen::Info: render_info(st); break;
        }
    }
    if (oled_flush() != ESP_OK) {
        ESP_LOGW(TAG, "OLED write failed; preserving handle for bus recovery");
        display_defer_retry();
    }
}

// A hold leaves any nested menu immediately without discarding unsaved RAM
// configuration. A separate long hold on Home retains master power control.
void go_home()
{
    s_screen = Screen::Home;
    s_last_input_ms = now_ms();
}

void rotate_index(uint8_t& value, int delta, uint8_t count)
{
    if (!count) return;
    int next = static_cast<int>(value) + delta;
    while (next < 0) next += count;
    while (next >= count) next -= count;
    value = static_cast<uint8_t>(next);
}

void update_zone_style(std::size_t zone, rgb::ZoneStyle style)
{
    const esp_err_t err = config::set_active_zone_style(zone, style);
    if (err == ESP_ERR_NO_MEM) set_notice("4 CUSTOM SLOTS USED");
    else if (err != ESP_OK) set_notice("ZONE UPDATE FAILED");
}

void handle_rotate(int delta, const config::RemoteSnapshot& st)
{
    if (!delta) return;
    s_last_input_ms = now_ms();
    switch (s_screen) {
        case Screen::Home: {
            // The home screen is the daily-use lighting remote. Turning the
            // encoder adjusts the global brightness without opening menus.
            // As with other live controls, Save is explicit (NVS wear safety).
            const int adjusted = std::clamp<int>(
                static_cast<int>(st.master_brightness) + delta * 5, 0, 255);
            (void)config::set_master_brightness(static_cast<uint8_t>(adjusted));
            break;
        }
        case Screen::Main: rotate_index(s_main_index, delta, 10); break;
        case Screen::Scenes: rotate_index(s_scene_index, delta, static_cast<uint8_t>(st.scene_count + 1)); break;
        case Screen::Zones: rotate_index(s_zone_index, delta, static_cast<uint8_t>(st.zone_count + 1)); break;
        case Screen::ZoneMenu: rotate_index(s_zone_menu_index, delta, 7); break;
        case Screen::ZoneBrightness: {
            auto style = st.active_zone_styles[s_zone_index];
            style.brightness = static_cast<uint8_t>(std::clamp<int>(style.brightness + delta * 5, 0, 255));
            update_zone_style(s_zone_index, style);
            break;
        }
        case Screen::ZoneEffect: {
            auto style = st.active_zone_styles[s_zone_index];
            uint8_t idx = effect_index(style.effect.id);
            rotate_index(idx, delta, static_cast<uint8_t>(effects::count()));
            style.effect.id = effects::descriptor(idx).id;
            update_zone_style(s_zone_index, style);
            break;
        }
        case Screen::ZoneColor: {
            auto style = st.active_zone_styles[s_zone_index];
            uint8_t idx = color_index(style.effect.primary);
            rotate_index(idx, delta, static_cast<uint8_t>(kColors.size()));
            style.effect.primary = kColors[idx].color;
            update_zone_style(s_zone_index, style);
            break;
        }
        case Screen::ZoneSpeed: {
            auto style = st.active_zone_styles[s_zone_index];
            const int next = std::clamp<int>(static_cast<int>(style.effect.speed_ms) + delta * 100, 100, 10000);
            style.effect.speed_ms = static_cast<uint16_t>(next);
            update_zone_style(s_zone_index, style);
            break;
        }
        case Screen::ZoneDirection: {
            auto style = st.active_zone_styles[s_zone_index];
            style.effect.reverse = delta > 0;
            update_zone_style(s_zone_index, style);
            break;
        }
        case Screen::QuickColor: rotate_index(s_quick_menu_index, delta, 4); break;
        case Screen::QuickColorColor: rotate_index(s_quick_color_index, delta, static_cast<uint8_t>(kColors.size())); break;
        case Screen::MasterBrightness:
            s_master_brightness_candidate = static_cast<uint8_t>(std::clamp<int>(s_master_brightness_candidate + delta * 5, 0, 255));
            (void)config::set_master_brightness(s_master_brightness_candidate);
            break;
        case Screen::Power: s_power_candidate = delta > 0; break;
        case Screen::QuickSettings: rotate_index(s_settings_index, delta, 6); break;
        case Screen::StartupPower: rotate_index(s_startup_power_candidate, delta, 3); break;
        case Screen::StartupScene: rotate_index(s_startup_scene_candidate, delta, static_cast<uint8_t>(st.scene_count + 1)); break;
        case Screen::SoftStart:
            s_soft_start_candidate = static_cast<uint16_t>(std::clamp<int>(static_cast<int>(s_soft_start_candidate) + delta * 250, 0, 10000));
            (void)config::set_power_on_fade_ms(s_soft_start_candidate);
            break;
        case Screen::Display: {
            const auto acc = accessories::snapshot();
            rotate_index(s_display_index, delta, display_item_count(acc));
            break;
        }
        case Screen::DisplayBrightness:
            s_display_brightness_candidate = static_cast<uint8_t>(
                std::clamp<int>(static_cast<int>(s_display_brightness_candidate) + delta, 0, 255));
            (void)config::set_remote_display(s_display_brightness_candidate,
                                             st.remote.oled_timeout_s,
                                             st.remote.oled_invert,
                                             st.remote.oled_auto_dim,
                                             st.remote.oled_screensaver_minutes);
            break;
        case Screen::DisplayAutoDim:
            s_display_auto_dim_candidate = delta > 0;
            (void)config::set_remote_display(st.remote.oled_contrast,
                                             st.remote.oled_timeout_s,
                                             st.remote.oled_invert,
                                             s_display_auto_dim_candidate,
                                             st.remote.oled_screensaver_minutes);
            break;
        case Screen::DisplaySleep:
            rotate_index(s_display_timeout_index, delta, static_cast<uint8_t>(kTimeouts.size()));
            (void)config::set_remote_display(st.remote.oled_contrast,
                                             kTimeouts[s_display_timeout_index],
                                             st.remote.oled_invert,
                                             st.remote.oled_auto_dim,
                                             st.remote.oled_screensaver_minutes);
            break;
        case Screen::DisplayInvert:
            s_display_invert_candidate = delta > 0;
            (void)config::set_remote_display(st.remote.oled_contrast,
                                             st.remote.oled_timeout_s,
                                             s_display_invert_candidate,
                                             st.remote.oled_auto_dim,
                                             st.remote.oled_screensaver_minutes);
            break;
        case Screen::DisplayScreensaver:
            rotate_index(s_display_screensaver_candidate, delta,
                         static_cast<uint8_t>(kScreensaverMinutes.size()));
            s_display_screensaver_candidate =
                kScreensaverMinutes[s_display_screensaver_candidate];
            (void)config::set_remote_display(st.remote.oled_contrast,
                                             st.remote.oled_timeout_s,
                                             st.remote.oled_invert,
                                             st.remote.oled_auto_dim,
                                             s_display_screensaver_candidate);
            break;
        case Screen::Info: rotate_index(s_info_page, delta, 2); break;
    }
}

void handle_short_press(const config::RemoteSnapshot& st)
{
    s_last_input_ms = now_ms();
    switch (s_screen) {
        case Screen::Home:
            s_main_index = 0; s_screen = Screen::Main; break;
        case Screen::Main:
            switch (s_main_index) {
                case 0: s_scene_index = st.active_scene; s_screen = Screen::Scenes; break;
                case 1: s_zone_index = 0; s_screen = Screen::Zones; break;
                case 2: s_quick_menu_index = 0; s_quick_target = -1; s_screen = Screen::QuickColor; break;
                case 3: s_master_brightness_candidate = st.master_brightness; s_screen = Screen::MasterBrightness; break;
                case 4: s_power_candidate = st.master_power; s_screen = Screen::Power; break;
                case 5: s_settings_index = 0; s_screen = Screen::QuickSettings; break;
                case 6: s_display_index = 0; s_screen = Screen::Display; break;
                case 7: s_info_page = 0; s_screen = Screen::Info; break;
                case 8:
                    if (config::save() == ESP_OK) set_notice("SAVED TO NVS"); else set_notice("SAVE FAILED");
                    break;
                case 9: go_home(); break;
            }
            break;
        case Screen::Scenes:
            if (s_scene_index >= st.scene_count) s_screen = Screen::Main;
            else if (config::set_active_scene(s_scene_index) == ESP_OK) {
                set_notice("SCENE ACTIVE"); s_screen = Screen::Home;
            }
            break;
        case Screen::Zones:
            if (s_zone_index >= st.zone_count) s_screen = Screen::Main;
            else { s_zone_menu_index = 0; s_screen = Screen::ZoneMenu; }
            break;
        case Screen::ZoneMenu: {
            if (!st.zone_count) { s_screen = Screen::Zones; break; }
            auto style = st.active_zone_styles[s_zone_index];
            switch (s_zone_menu_index) {
                case 0: s_screen = Screen::ZoneBrightness; break;
                case 1: s_screen = Screen::ZoneEffect; break;
                case 2: s_screen = Screen::ZoneColor; break;
                case 3: s_screen = Screen::ZoneSpeed; break;
                case 4: s_screen = Screen::ZoneDirection; break;
                case 5: style.enabled = !style.enabled; update_zone_style(s_zone_index, style); break;
                default: s_screen = Screen::Zones; break;
            }
            break;
        }
        case Screen::ZoneBrightness:
        case Screen::ZoneEffect:
        case Screen::ZoneColor:
        case Screen::ZoneSpeed:
        case Screen::ZoneDirection: s_screen = Screen::ZoneMenu; break;
        case Screen::QuickColor:
            switch (s_quick_menu_index) {
                case 0:
                    if (st.zone_count) { ++s_quick_target; if (s_quick_target >= st.zone_count) s_quick_target = -1; }
                    break;
                case 1: s_screen = Screen::QuickColorColor; break;
                case 2: {
                    const esp_err_t err = config::apply_quick_color(s_quick_target, kColors[s_quick_color_index].color);
                    if (err == ESP_OK) { set_notice("COLOR APPLIED"); s_screen = Screen::Home; }
                    else if (err == ESP_ERR_NO_MEM) set_notice("4 CUSTOM SLOTS USED");
                    else set_notice("COLOR FAILED");
                    break;
                }
                default: s_screen = Screen::Main; break;
            }
            break;
        case Screen::QuickColorColor: s_screen = Screen::QuickColor; break;
        case Screen::MasterBrightness: s_screen = Screen::Main; break;
        case Screen::Power:
            if (config::set_master_power(s_power_candidate) == ESP_OK) set_notice(s_power_candidate ? "LIGHTING ON" : "LIGHTING OFF");
            s_screen = Screen::Home;
            break;
        case Screen::QuickSettings:
            switch (s_settings_index) {
                case 0: s_startup_power_candidate = static_cast<uint8_t>(st.startup.power_mode); s_screen = Screen::StartupPower; break;
                case 1: s_startup_scene_candidate = st.startup.scene == rgb::kStartupSceneLast ? 0 : static_cast<uint8_t>(st.startup.scene + 1); s_screen = Screen::StartupScene; break;
                case 2: s_soft_start_candidate = st.power_on_fade_ms; s_screen = Screen::SoftStart; break;
                case 3: if (config::save() == ESP_OK) set_notice("SAVED TO NVS"); else set_notice("SAVE FAILED"); break;
                case 4: if (config::revert() == ESP_OK) set_notice("CHANGES REVERTED"); else set_notice("REVERT FAILED"); break;
                default: s_screen = Screen::Main; break;
            }
            break;
        case Screen::StartupPower:
            (void)config::set_startup_power_mode(static_cast<rgb::StartupPowerMode>(s_startup_power_candidate));
            s_screen = Screen::QuickSettings;
            break;
        case Screen::StartupScene: {
            const uint8_t scene = s_startup_scene_candidate == 0 ? rgb::kStartupSceneLast : static_cast<uint8_t>(s_startup_scene_candidate - 1);
            (void)config::set_startup_scene(scene);
            s_screen = Screen::QuickSettings;
            break;
        }
        case Screen::SoftStart: s_screen = Screen::QuickSettings; break;
        case Screen::Display: {
            const auto acc = accessories::snapshot();
            switch (display_item_at(s_display_index, acc)) {
                case DisplayItem::Brightness:
                    s_display_brightness_candidate = st.remote.oled_contrast;
                    s_screen = Screen::DisplayBrightness;
                    break;
                case DisplayItem::AutoDim:
                    s_display_auto_dim_candidate = st.remote.oled_auto_dim;
                    s_screen = Screen::DisplayAutoDim;
                    break;
                case DisplayItem::Sleep:
                    s_display_timeout_index = nearest_timeout_index(st.remote.oled_timeout_s);
                    s_screen = Screen::DisplaySleep;
                    break;
                case DisplayItem::Invert:
                    s_display_invert_candidate = st.remote.oled_invert;
                    s_screen = Screen::DisplayInvert;
                    break;
                case DisplayItem::Screensaver:
                    s_display_screensaver_candidate =
                        std::min<uint8_t>(st.remote.oled_screensaver_minutes, 5);
                    s_screen = Screen::DisplayScreensaver;
                    break;
                case DisplayItem::Back:
                    s_screen = Screen::Main;
                    break;
            }
            break;
        }
        case Screen::DisplayBrightness:
        case Screen::DisplayAutoDim:
        case Screen::DisplaySleep:
        case Screen::DisplayInvert:
        case Screen::DisplayScreensaver: s_screen = Screen::Display; break;
        case Screen::Info: s_screen = Screen::Main; break;
    }
}

void task(void*)
{
    encoder_init();
    s_last_input_ms = now_ms();

    uint8_t last_zone_count = 0xFF;
    while (true) {
        auto st = config::remote_snapshot();
        // Phone edits can compact slot indices while the OLED menu is open.
        // Never leave a deleted zone's editor pointed at the next zone.
        if (last_zone_count != 0xFF && st.zone_count < last_zone_count) {
            switch (s_screen) {
                case Screen::ZoneMenu:
                case Screen::ZoneBrightness:
                case Screen::ZoneEffect:
                case Screen::ZoneColor:
                case Screen::ZoneSpeed:
                case Screen::ZoneDirection:
                    s_screen = st.zone_count ? Screen::Zones : Screen::Main;
                    break;
                default: break;
            }
        }
        last_zone_count = st.zone_count;
        if (s_screen == Screen::Zones) {
            if (s_zone_index > st.zone_count) s_zone_index = st.zone_count; // BACK row
        } else if (s_zone_index >= st.zone_count) {
            s_zone_index = st.zone_count ? st.zone_count - 1 : 0;
        }
        if (s_screen == Screen::Scenes) {
            if (s_scene_index > st.scene_count) s_scene_index = st.scene_count; // BACK row
        } else if (s_scene_index >= st.scene_count) {
            s_scene_index = st.scene_count ? st.scene_count - 1 : 0;
        }
        if (s_quick_target >= st.zone_count) s_quick_target = -1;
        if (s_startup_scene_candidate > st.scene_count) s_startup_scene_candidate = 0;
        auto acc = accessories::snapshot();
        int64_t now = now_ms();

        if (st.remote_enabled != s_last_enabled) {
            s_last_enabled = st.remote_enabled;
            s_screen = Screen::Home;
            encoder_reset_pending();
            reset_screensaver();
            if (!s_last_enabled) {
                display_shutdown();
                ESP_LOGI(TAG, "Remote Control disabled");
            } else {
                ESP_LOGI(TAG, "Remote Control enabled");
                s_last_display_attempt_ms = 0;
            }
        }

        if (!st.remote_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // A bus reset invalidates the SSD1306's assumed display state. Use
        // the same probe -> reinit -> visible-frame path as hot-plug recovery.
        // Never mark the screen restored based only on bus reset success.
        const uint32_t bus_generation = shared_i2c::recovery_generation();
        if (bus_generation != s_bus_generation_seen) {
            s_bus_generation_seen = bus_generation;
            if (s_oled) {
                display_defer_retry();
                s_last_display_attempt_ms = 0; // retry now after the reset
            }
        }

        // If the OLED is asleep there may be no normal I2C writes to reveal
        // that it was unplugged. A lightweight periodic ACK check catches
        // that case; NACK is an accessory failure, not a shared-bus fault.
        if (s_display_ready && s_oled && s_oled_addr &&
            (s_last_presence_check_ms == 0 ||
             now - s_last_presence_check_ms >= kOledPresenceCheckMs)) {
            s_last_presence_check_ms = now;
            const esp_err_t presence = shared_i2c::probe(s_oled_addr, kI2cTimeoutMs);
            if (presence == ESP_OK) {
                s_presence_failures = 0;
            } else if (++s_presence_failures >= kOledPresenceFailureLimit) {
                ESP_LOGW(TAG, "SSD1306 unresponsive (%s); awaiting re-detection",
                         esp_err_to_name(presence));
                display_defer_retry();
            }
        }

        if (!s_display_ready &&
            (s_last_display_attempt_ms == 0 || now - s_last_display_attempt_ms >= kRetryDisplayMs)) {
            s_last_display_attempt_ms = now;
            const esp_err_t err = display_start(st.remote);
            if (err != ESP_OK) ESP_LOGW(TAG, "SSD1306 not found (%s); retrying", esp_err_to_name(err));
        }

        if (s_display_ready && s_splash_pending) {
            run_startup_splash(st, acc);
            vTaskDelay(pdMS_TO_TICKS(kPollMs));
            continue;
        }

        if (s_display_ready) apply_display_config(st.remote, acc);

        int delta = encoder_step();
        ButtonEvent button = button_event();
        const bool activity = delta != 0 || s_switch_activity || button != ButtonEvent::None;

        if (activity && s_display_ready && s_screensaver_active) {
            // The wake click must not subsequently open a menu on release.
            if (gpio_get_level(board::kRemoteEncoderSwitchGpio) == 0) {
                s_ignore_wake_press = true;
                s_hold_action_sent = true;
            }
            const uint8_t target = effective_display_brightness(st.remote, acc);
            fade_to(0, 120);
            reset_screensaver();
            s_last_input_ms = now;
            s_screen = Screen::Home;
            render_home(st);
            if (oled_flush() == ESP_OK) fade_to(target, 120);
            delta = 0;
            button = ButtonEvent::None;
        } else if (activity && s_display_ready && !s_oled_awake) {
            if (gpio_get_level(board::kRemoteEncoderSwitchGpio) == 0) {
                s_ignore_wake_press = true;
                s_hold_action_sent = true;
            }
            oled_set_awake(true);
            apply_display_config(st.remote, acc);
            s_last_input_ms = now;
            delta = 0;
            button = ButtonEvent::None;
        }

        if (delta) handle_rotate(delta, st);
        if (button == ButtonEvent::Short) handle_short_press(st);
        else if (button == ButtonEvent::Long) { go_home(); set_notice("HOME"); }
        else if (button == ButtonEvent::VeryLong) {
            (void)config::set_master_power(!st.master_power);
            set_notice(st.master_power ? "MASTER OFF" : "MASTER ON");
            s_screen = Screen::Home;
            s_last_input_ms = now;
        }

        st = config::remote_snapshot();
        acc = accessories::snapshot();
        now = now_ms();

        if (s_screen != Screen::Home && now - s_last_input_ms >= kIdleHomeMs) {
            s_screen = Screen::Home;
        }

        const bool saver_ready = rtc_screensaver_ready(st.remote, acc);
        const int64_t idle_ms = now - s_last_input_ms;
        const int64_t saver_delay_ms =
            static_cast<int64_t>(st.remote.oled_screensaver_minutes) * 60 * 1000;

        if (s_screensaver_active && !saver_ready) {
            reset_screensaver();
        }

        if (!s_screensaver_active && saver_ready && s_oled_awake && idle_ms >= saver_delay_ms) {
            const uint8_t target = effective_display_brightness(st.remote, acc);
            fade_to(0, kScreensaverFadeMs);
            s_screensaver_active = true;
            s_screensaver_panel = SaverPanel::Clock;
            s_screensaver_started_ms = now;
            s_screensaver_panel_ms = now;
            render_screensaver(st, acc);
            if (oled_flush() == ESP_OK) fade_to(target, kScreensaverFadeMs);
            s_last_render_ms = now;
        }

        // Sleep remains the higher-priority power-saving behavior. With an RTC
        // screensaver enabled, the sleep timer starts when the screensaver
        // starts so the saver is still visible for the selected idle period.
        bool sleep_due = false;
        if (st.remote.oled_timeout_s > 0) {
            if (s_screensaver_active) {
                sleep_due = now - s_screensaver_started_ms >=
                    static_cast<int64_t>(st.remote.oled_timeout_s) * 1000;
            } else if (!saver_ready) {
                sleep_due = idle_ms >= static_cast<int64_t>(st.remote.oled_timeout_s) * 1000;
            }
        }

        if (s_display_ready && sleep_due) {
            if (s_oled_awake) fade_to(0, 120);
            oled_set_awake(false);
            reset_screensaver();
        } else if (s_display_ready && s_oled_awake && s_screensaver_active) {
            if (now - s_screensaver_panel_ms >= saver_panel_duration_ms()) {
                const uint8_t target = effective_display_brightness(st.remote, acc);
                fade_to(0, kScreensaverFadeMs);
                switch (s_screensaver_panel) {
                    case SaverPanel::Clock: s_screensaver_panel = SaverPanel::Logo; break;
                    case SaverPanel::Logo: s_screensaver_panel = SaverPanel::Scene; break;
                    case SaverPanel::Scene: s_screensaver_panel = SaverPanel::Clock; break;
                }
                s_screensaver_panel_ms = now;
                render_screensaver(st, acc);
                if (oled_flush() == ESP_OK) fade_to(target, kScreensaverFadeMs);
                else display_defer_retry();
                s_last_render_ms = now;
            } else if ((s_screensaver_panel == SaverPanel::Clock && now - s_last_render_ms >= 900) ||
                       (s_screensaver_panel == SaverPanel::Scene && now - s_last_render_ms >= kSaverSceneFrameMs)) {
                // Slow clock updates, animated scene at ~5.5fps. Flush only
                // changed SSD1306 ranges to spare the shared I2C bus.
                s_last_render_ms = now;
                render_screensaver(st, acc);
                if (oled_flush() != ESP_OK) {
                    ESP_LOGW(TAG, "OLED write failed during screensaver; preserving handle");
                    display_defer_retry();
                }
            }
        } else if (s_display_ready && s_oled_awake &&
                   (activity || now - s_last_render_ms >= kDisplayRefreshMs ||
                    now < s_notice_until_ms)) {
            s_last_render_ms = now;
            render(st);
        }

        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}

} // namespace

namespace remote_control {

esp_err_t start()
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(task, "remote_ctrl", 6144, nullptr, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

} // namespace remote_control
