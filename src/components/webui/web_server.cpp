#include "web_server.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include "cJSON.h"
#include "accessories.hpp"
#include "config_store.hpp"
#include "effects.hpp"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lighting.hpp"
#include "rgb_model.hpp"

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");
extern const unsigned char auto_rgb_logo_webp_start[] asm("_binary_auto_rgb_logo_webp_start");
extern const unsigned char auto_rgb_logo_webp_end[] asm("_binary_auto_rgb_logo_webp_end");

namespace {
constexpr const char* TAG = "webui";
httpd_handle_t s_server = nullptr;
rgb::DeviceConfig s_web_config{}; // HTTP server runs handlers on one task; keep large config off its stack.

void set_json(httpd_req_t* req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

esp_err_t send_json(httpd_req_t* req, cJSON* root)
{
    char* text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!text) return ESP_ERR_NO_MEM;
    set_json(req);
    const esp_err_t err = httpd_resp_sendstr(req, text);
    cJSON_free(text);
    return err;
}

esp_err_t send_error(httpd_req_t* req, const char* message, const char* status = "400 Bad Request")
{
    httpd_resp_set_status(req, status);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddFalseToObject(root, "ok");
    cJSON_AddStringToObject(root, "error", message);
    return send_json(req, root);
}

std::string read_body(httpd_req_t* req)
{
    if (req->content_len <= 0 || req->content_len > 4096) return {};
    std::string body(static_cast<std::size_t>(req->content_len), '\0');
    int received = 0;
    while (received < req->content_len) {
        const int r = httpd_req_recv(req, body.data() + received, req->content_len - received);
        if (r <= 0) return {};
        received += r;
    }
    return body;
}

cJSON* parse_body(httpd_req_t* req)
{
    const std::string body = read_body(req);
    return body.empty() ? nullptr : cJSON_ParseWithLength(body.data(), body.size());
}

const char* pixel_order_key(rgb::PixelOrder order)
{
    switch (order) {
        case rgb::PixelOrder::RGB: return "RGB";
        case rgb::PixelOrder::BRG: return "BRG";
        case rgb::PixelOrder::RBG: return "RBG";
        case rgb::PixelOrder::GBR: return "GBR";
        case rgb::PixelOrder::BGR: return "BGR";
        case rgb::PixelOrder::GRB:
        default: return "GRB";
    }
}

rgb::PixelOrder parse_pixel_order(const char* value)
{
    if (!value) return rgb::PixelOrder::GRB;
    if (!std::strcmp(value, "RGB")) return rgb::PixelOrder::RGB;
    if (!std::strcmp(value, "BRG")) return rgb::PixelOrder::BRG;
    if (!std::strcmp(value, "RBG")) return rgb::PixelOrder::RBG;
    if (!std::strcmp(value, "GBR")) return rgb::PixelOrder::GBR;
    if (!std::strcmp(value, "BGR")) return rgb::PixelOrder::BGR;
    return rgb::PixelOrder::GRB;
}

cJSON* color_json(rgb::Color c)
{
    char text[8];
    std::snprintf(text, sizeof(text), "#%02X%02X%02X", c.r, c.g, c.b);
    return cJSON_CreateString(text);
}

bool parse_color(const cJSON* item, rgb::Color& c)
{
    if (!cJSON_IsString(item) || !item->valuestring || std::strlen(item->valuestring) != 7 || item->valuestring[0] != '#') return false;
    unsigned r = 0, g = 0, b = 0;
    if (std::sscanf(item->valuestring + 1, "%02x%02x%02x", &r, &g, &b) != 3) return false;
    c = {static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)};
    return true;
}

cJSON* palette_json(const rgb::EffectConfig& fx)
{
    cJSON* arr = cJSON_CreateArray();
    const uint8_t count = static_cast<uint8_t>(std::clamp<int>(fx.palette_count, 1, static_cast<int>(rgb::kMaxPaletteColors)));
    cJSON_AddItemToArray(arr, color_json(fx.primary));
    if (count > 1) cJSON_AddItemToArray(arr, color_json(fx.secondary));
    for (uint8_t i = 2; i < count; ++i) cJSON_AddItemToArray(arr, color_json(fx.extra_colors[i - 2]));
    return arr;
}

void parse_palette(const cJSON* item, rgb::EffectConfig& fx)
{
    if (!cJSON_IsArray(item)) return;
    const int supplied = cJSON_GetArraySize(item);
    if (supplied <= 0) return;
    const int count = std::clamp(supplied, 1, static_cast<int>(rgb::kMaxPaletteColors));
    for (int i = 0; i < count; ++i) {
        rgb::Color color{};
        if (!parse_color(cJSON_GetArrayItem(item, i), color)) continue;
        if (i == 0) fx.primary = color;
        else if (i == 1) fx.secondary = color;
        else fx.extra_colors[static_cast<std::size_t>(i - 2)] = color;
    }
    fx.palette_count = static_cast<uint8_t>(count);
}

cJSON* effect_json(const rgb::EffectConfig& fx)
{
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "id", effects::key(fx.id));
    cJSON_AddItemToObject(obj, "primary", color_json(fx.primary));
    cJSON_AddItemToObject(obj, "secondary", color_json(fx.secondary));
    cJSON_AddNumberToObject(obj, "paletteCount", fx.palette_count);
    cJSON_AddItemToObject(obj, "palette", palette_json(fx));
    cJSON_AddNumberToObject(obj, "speedMs", fx.speed_ms);
    cJSON_AddNumberToObject(obj, "width", fx.width);
    cJSON_AddNumberToObject(obj, "intensity", fx.intensity);
    cJSON_AddNumberToObject(obj, "density", fx.density);
    cJSON_AddBoolToObject(obj, "reverse", fx.reverse);
    return obj;
}

cJSON* config_json(const rgb::DeviceConfig& cfg, uint32_t revision)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "schemaVersion", cfg.schema_version);
    cJSON_AddBoolToObject(root, "dirty", config::dirty());
    cJSON_AddNumberToObject(root, "revision", revision);
    cJSON* capabilities = cJSON_AddObjectToObject(root, "capabilities");
    cJSON_AddBoolToObject(capabilities, "deleteZones", true);
    cJSON_AddBoolToObject(capabilities, "deleteScenes", true);
    cJSON_AddBoolToObject(capabilities, "globalLighting", true);

    cJSON* system = cJSON_AddObjectToObject(root, "system");
    cJSON_AddStringToObject(system, "deviceName", cfg.system.device_name);
    cJSON_AddStringToObject(system, "ssid", cfg.system.ap_ssid);
    cJSON_AddNumberToObject(system, "masterBrightness", cfg.system.master_brightness);
    cJSON_AddNumberToObject(system, "activeScene", cfg.system.active_scene);
    cJSON_AddBoolToObject(system, "masterPower", cfg.power.master_enabled);
    cJSON_AddNumberToObject(system, "powerOnFadeMs", cfg.power.power_on_fade_ms);
    cJSON_AddBoolToObject(system, "remoteControlEnabled", cfg.remote.enabled);
    cJSON_AddNumberToObject(system, "startupPowerMode", static_cast<uint8_t>(cfg.startup.power_mode));
    cJSON_AddNumberToObject(system, "startupScene", cfg.startup.scene == rgb::kStartupSceneLast ? -1 : cfg.startup.scene);
    cJSON_AddNumberToObject(system, "remoteDisplayContrast", cfg.remote.oled_contrast);
    cJSON_AddNumberToObject(system, "remoteDisplayTimeout", cfg.remote.oled_timeout_s);
    cJSON_AddBoolToObject(system, "remoteDisplayInvert", cfg.remote.oled_invert);
    cJSON_AddBoolToObject(system, "remoteDisplayAutoDim", cfg.remote.oled_auto_dim);
    cJSON_AddNumberToObject(system, "remoteScreensaverMinutes", cfg.remote.oled_screensaver_minutes);
    cJSON_AddBoolToObject(system, "rtcEnabled", cfg.accessories.rtc_enabled);
    cJSON_AddBoolToObject(system, "ambientLightEnabled", cfg.accessories.als_enabled);

    cJSON* channels = cJSON_AddArrayToObject(root, "channels");
    for (std::size_t i = 0; i < rgb::kChannelCount; ++i) {
        const auto& ch = cfg.channels[i];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", i);
        cJSON_AddStringToObject(item, "name", ch.name);
        cJSON_AddNumberToObject(item, "ledCount", ch.led_count);
        cJSON_AddNumberToObject(item, "brightness", ch.brightness);
        cJSON_AddStringToObject(item, "order", pixel_order_key(ch.order));
        cJSON_AddBoolToObject(item, "enabled", ch.enabled);
        cJSON_AddBoolToObject(item, "reversed", ch.reversed);
        cJSON_AddItemToArray(channels, item);
    }

    cJSON* zones = cJSON_AddArrayToObject(root, "zones");
    for (std::size_t z = 0; z < cfg.zone_count; ++z) {
        const auto& zone = cfg.zones[z];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", z);
        cJSON_AddStringToObject(item, "name", zone.name);
        cJSON_AddBoolToObject(item, "enabled", zone.enabled);
        cJSON* segments = cJSON_AddArrayToObject(item, "segments");
        for (std::size_t s = 0; s < zone.segment_count; ++s) {
            const auto& seg = zone.segments[s];
            cJSON* segj = cJSON_CreateObject();
            cJSON_AddNumberToObject(segj, "channel", seg.channel);
            cJSON_AddNumberToObject(segj, "start", seg.start);
            cJSON_AddNumberToObject(segj, "length", seg.length);
            cJSON_AddBoolToObject(segj, "reversed", seg.reversed);
            cJSON_AddItemToArray(segments, segj);
        }
        cJSON_AddItemToArray(zones, item);
    }

    cJSON* slots = cJSON_AddObjectToObject(root, "sceneSlots");
    cJSON_AddNumberToObject(slots, "builtIn", rgb::kBuiltInSceneCount);
    cJSON_AddNumberToObject(slots, "customMax", rgb::kCustomSceneCount);
    cJSON_AddNumberToObject(slots, "customUsed",
                            cfg.scene_count > rgb::kBuiltInSceneCount
                                ? cfg.scene_count - rgb::kBuiltInSceneCount : 0);

    cJSON* scenes = cJSON_AddArrayToObject(root, "scenes");
    for (std::size_t s = 0; s < cfg.scene_count; ++s) {
        const auto& scene = cfg.scenes[s];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", s);
        cJSON_AddStringToObject(item, "name", scene.name);
        cJSON_AddBoolToObject(item, "enabled", scene.enabled);
        cJSON_AddBoolToObject(item, "builtIn", s < rgb::kBuiltInSceneCount);
        // -1 is a global effect target, never a numbered user zone.
        cJSON* global = cJSON_AddObjectToObject(item, "global");
        cJSON_AddNumberToObject(global, "zone", -1);
        cJSON_AddBoolToObject(global, "enabled", scene.all_lighting.enabled);
        cJSON_AddNumberToObject(global, "brightness", scene.all_lighting.brightness);
        cJSON_AddItemToObject(global, "effect", effect_json(scene.all_lighting.effect));
        cJSON* styles = cJSON_AddArrayToObject(item, "zones");
        for (std::size_t z = 0; z < cfg.zone_count; ++z) {
            cJSON* style = cJSON_CreateObject();
            cJSON_AddNumberToObject(style, "zone", z);
            cJSON_AddBoolToObject(style, "enabled", scene.zones[z].enabled);
            cJSON_AddNumberToObject(style, "brightness", scene.zones[z].brightness);
            cJSON_AddItemToObject(style, "effect", effect_json(scene.zones[z].effect));
            cJSON_AddItemToArray(styles, style);
        }
        cJSON_AddItemToArray(scenes, item);
    }
    return root;
}

esp_err_t root_get(httpd_req_t* req)
{
    ESP_LOGI(TAG, "Serve Auto-RGB root");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, reinterpret_cast<const char*>(index_html_start), index_html_end - index_html_start);
}

esp_err_t captive_probe_get(httpd_req_t* req)
{
    // Follow Espressif's captive-portal example closely. iOS in particular
    // expects a redirect response with non-empty content; a redirect with no
    // body can be treated as an ordinary navigation instead of captive login.
    ESP_LOGI(TAG, "Captive HTTP probe: %s", req->uri);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, "Redirect to the Auto-RGB captive portal", HTTPD_RESP_USE_STRLEN);
}

esp_err_t capport_get(httpd_req_t* req)
{
    // RFC 8908 captive-portal API advertised through DHCP option 114.
    // Keep reporting captive because Auto-RGB is intentionally a local-only AP.
    static constexpr char kCapportJson[] =
        "{\"captive\":true,"
        "\"user-portal-url\":\"http://192.168.4.1/\"}";

    httpd_resp_set_type(req, "application/captive+json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    return httpd_resp_send(req, kCapportJson, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captive_404(httpd_req_t* req, httpd_err_code_t)
{
    // API typos remain real 404s. Everything else is treated like XBOX-RGB's
    // onNotFound captive handler so arbitrary hostnames/paths still open setup.
    if (std::strncmp(req->uri, "/api/", 5) == 0) {
        return send_error(req, "Not found", "404 Not Found");
    }

    return captive_probe_get(req);
}

esp_err_t logo_get(httpd_req_t* req)
{
    httpd_resp_set_type(req, "image/webp");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, reinterpret_cast<const char*>(auto_rgb_logo_webp_start), auto_rgb_logo_webp_end - auto_rgb_logo_webp_start);
}

esp_err_t status_get(httpd_req_t* req)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddTrueToObject(root, "ok");
    cJSON_AddStringToObject(root, "version", esp_app_get_description()->version);
    cJSON_AddNumberToObject(root, "uptimeMs", static_cast<double>(esp_timer_get_time() / 1000));
    cJSON_AddBoolToObject(root, "dirty", config::dirty());
    cJSON_AddNumberToObject(root, "revision", config::revision());
    return send_json(req, root);
}

esp_err_t config_get(httpd_req_t* req)
{
    const uint32_t revision = config::snapshot_into_with_revision(s_web_config);
    return send_json(req, config_json(s_web_config, revision));
}

esp_err_t control_get(httpd_req_t* req)
{
    const auto st = config::control_snapshot();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "revision", st.revision);
    cJSON_AddBoolToObject(root, "dirty", st.dirty);
    cJSON_AddBoolToObject(root, "masterPower", st.master_power);
    cJSON_AddNumberToObject(root, "masterBrightness", st.master_brightness);
    cJSON_AddNumberToObject(root, "activeScene", st.active_scene);
    cJSON_AddBoolToObject(root, "remoteControlEnabled", st.remote_enabled);
    return send_json(req, root);
}

esp_err_t accessories_get(httpd_req_t* req)
{
    const auto st = accessories::snapshot();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "busReady", st.bus_ready);
    cJSON_AddBoolToObject(root, "fault", st.fault);

    cJSON* remote = cJSON_AddObjectToObject(root, "remote");
    cJSON_AddBoolToObject(remote, "enabled", st.remote_enabled);
    cJSON_AddBoolToObject(remote, "present", st.remote_present);
    cJSON_AddNumberToObject(remote, "address", st.oled_address);

    cJSON* rtc = cJSON_AddObjectToObject(root, "rtc");
    cJSON_AddBoolToObject(rtc, "enabled", st.rtc_enabled);
    cJSON_AddBoolToObject(rtc, "present", st.rtc_present);
    cJSON_AddBoolToObject(rtc, "timeValid", st.rtc_time_valid);
    if (st.rtc_present) {
        char stamp[32]{};
        std::snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u %02u:%02u:%02u",
                      static_cast<unsigned>(st.year), static_cast<unsigned>(st.month),
                      static_cast<unsigned>(st.day), static_cast<unsigned>(st.hour),
                      static_cast<unsigned>(st.minute), static_cast<unsigned>(st.second));
        cJSON_AddStringToObject(rtc, "time", stamp);
    }

    cJSON* als = cJSON_AddObjectToObject(root, "als");
    cJSON_AddBoolToObject(als, "enabled", st.als_enabled);
    cJSON_AddBoolToObject(als, "present", st.als_present);
    cJSON_AddNumberToObject(als, "raw", st.als_raw);
    cJSON_AddNumberToObject(als, "lux", st.lux);
    return send_json(req, root);
}

esp_err_t effects_get(httpd_req_t* req)
{
    cJSON* root = cJSON_CreateObject();
    cJSON* list = cJSON_AddArrayToObject(root, "effects");
    for (std::size_t i = 0; i < effects::count(); ++i) {
        const auto& d = effects::descriptor(i);
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "id", d.key);
        cJSON_AddStringToObject(item, "name", d.name);
        cJSON_AddBoolToObject(item, "usesPrimary", effects::uses(d, effects::Primary));
        cJSON_AddBoolToObject(item, "usesSecondary", effects::uses(d, effects::Secondary));
        cJSON_AddBoolToObject(item, "usesSpeed", effects::uses(d, effects::Speed));
        cJSON_AddBoolToObject(item, "usesWidth", effects::uses(d, effects::Width));
        cJSON_AddBoolToObject(item, "usesDensity", effects::uses(d, effects::Density));
        cJSON_AddBoolToObject(item, "usesReverse", effects::uses(d, effects::Reverse));
        cJSON_AddBoolToObject(item, "usesPalette", effects::uses(d, effects::Palette));
        cJSON_AddNumberToObject(item, "paletteMin", d.palette_min);
        cJSON_AddNumberToObject(item, "paletteMax", d.palette_max);
        cJSON_AddItemToArray(list, item);
    }
    return send_json(req, root);
}

esp_err_t channel_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kChannelCount)) {
        cJSON_Delete(root); return send_error(req, "Invalid channel id");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    config::snapshot_into(s_web_config);
    auto& cfg = s_web_config;
    auto ch = cfg.channels[id];

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) std::snprintf(ch.name, sizeof(ch.name), "%s", v->valuestring);
    if (cJSON* v = cJSON_GetObjectItem(root, "ledCount"); cJSON_IsNumber(v)) ch.led_count = std::clamp(v->valueint, 0, static_cast<int>(rgb::kMaxLedsPerChannel));
    if (cJSON* v = cJSON_GetObjectItem(root, "brightness"); cJSON_IsNumber(v)) ch.brightness = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) ch.enabled = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "reversed"); cJSON_IsBool(v)) ch.reversed = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "order"); cJSON_IsString(v)) ch.order = parse_pixel_order(v->valuestring);
    cJSON_Delete(root);

    if (config::update_channel(id, ch) != ESP_OK) return send_error(req, "Channel update rejected");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t zone_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kMaxZones)) {
        cJSON_Delete(root); return send_error(req, "Invalid zone id");
    }
    const std::size_t id = idj->valueint;
    config::snapshot_into(s_web_config);
    auto& cfg = s_web_config;
    rgb::ZoneConfig zone = id < cfg.zone_count ? cfg.zones[id] : rgb::ZoneConfig{};
    if (id >= cfg.zone_count) std::snprintf(zone.name, sizeof(zone.name), "Zone %u", static_cast<unsigned>(id + 1));

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) std::snprintf(zone.name, sizeof(zone.name), "%s", v->valuestring);
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) zone.enabled = cJSON_IsTrue(v);
    if (cJSON* segs = cJSON_GetObjectItem(root, "segments"); cJSON_IsArray(segs)) {
        zone.segment_count = 0;
        const int n = std::min(cJSON_GetArraySize(segs), static_cast<int>(rgb::kMaxSegmentsPerZone));
        for (int i = 0; i < n; ++i) {
            cJSON* sj = cJSON_GetArrayItem(segs, i);
            cJSON* cj = cJSON_GetObjectItem(sj, "channel");
            if (!cJSON_IsNumber(cj) || cj->valueint < 0 || cj->valueint >= static_cast<int>(rgb::kChannelCount)) continue;
            auto& seg = zone.segments[zone.segment_count++];
            seg.channel = static_cast<uint8_t>(cj->valueint);
            if (cJSON* v = cJSON_GetObjectItem(sj, "start"); cJSON_IsNumber(v)) seg.start = std::max(v->valueint, 0);
            if (cJSON* v = cJSON_GetObjectItem(sj, "length"); cJSON_IsNumber(v)) seg.length = std::max(v->valueint, 0);
            if (cJSON* v = cJSON_GetObjectItem(sj, "reversed"); cJSON_IsBool(v)) seg.reversed = cJSON_IsTrue(v);
        }
    }
    cJSON_Delete(root);
    if (config::update_zone(id, zone) != ESP_OK) return send_error(req, "Zone update rejected");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t zone_delete_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    const cJSON* id = cJSON_GetObjectItem(root, "id");
    const cJSON* rev = cJSON_GetObjectItem(root, "expectedRevision");
    const bool valid = cJSON_IsNumber(id) && id->valueint >= 0 &&
                       id->valueint < static_cast<int>(rgb::kMaxZones) &&
                       cJSON_IsNumber(rev) && rev->valuedouble >= 0 &&
                       rev->valuedouble <= 4294967295.0;
    if (!valid) { cJSON_Delete(root); return send_error(req, "Invalid zone or revision"); }
    const auto index = static_cast<std::size_t>(id->valueint);
    const auto expected = static_cast<uint32_t>(rev->valuedouble);
    cJSON_Delete(root);
    const esp_err_t result = config::remove_zone(index, expected);
    if (result == ESP_ERR_INVALID_STATE) return send_error(req, "Configuration changed; refresh and retry", "409 Conflict");
    if (result != ESP_OK) return send_error(req, "Zone deletion rejected");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t scene_delete_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    const cJSON* id = cJSON_GetObjectItem(root, "id");
    const cJSON* rev = cJSON_GetObjectItem(root, "expectedRevision");
    const bool valid = cJSON_IsNumber(id) &&
                       id->valueint >= static_cast<int>(rgb::kBuiltInSceneCount) &&
                       id->valueint < static_cast<int>(rgb::kMaxScenes) &&
                       cJSON_IsNumber(rev) && rev->valuedouble >= 0 &&
                       rev->valuedouble <= 4294967295.0;
    if (!valid) { cJSON_Delete(root); return send_error(req, "Cannot delete built-in scene or invalid revision"); }
    const auto index = static_cast<std::size_t>(id->valueint);
    const auto expected = static_cast<uint32_t>(rev->valuedouble);
    cJSON_Delete(root);
    const esp_err_t result = config::remove_custom_scene(index, expected);
    if (result == ESP_ERR_INVALID_STATE) return send_error(req, "Configuration changed; refresh and retry", "409 Conflict");
    if (result != ESP_OK) return send_error(req, "Scene deletion rejected");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t scene_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kMaxScenes)) {
        cJSON_Delete(root); return send_error(req, "Invalid scene id");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    if (id < rgb::kBuiltInSceneCount) {
        cJSON_Delete(root);
        return send_error(req, "Built-in demo scenes are read-only");
    }

    config::snapshot_into(s_web_config);
    auto& cfg = s_web_config;
    if (id > cfg.scene_count) {
        cJSON_Delete(root);
        return send_error(req, "Scene slots must be created in order");
    }

    rgb::SceneConfig scene{};
    if (cJSON* clone = cJSON_GetObjectItem(root, "cloneFrom"); cJSON_IsNumber(clone)) {
        const int source = clone->valueint;
        if (source < 0 || source >= static_cast<int>(cfg.scene_count)) {
            cJSON_Delete(root); return send_error(req, "Invalid source scene");
        }
        scene = cfg.scenes[static_cast<std::size_t>(source)];

        // Append " Copy" without relying on snprintf truncation. GCC 14 treats
        // format truncation warnings as errors in the ESP-IDF build.
        constexpr char kCopySuffix[] = " Copy";
        constexpr std::size_t kSuffixChars = sizeof(kCopySuffix) - 1;
        constexpr std::size_t kMaxBaseChars = sizeof(scene.name) - 1 - kSuffixChars;
        const char* source_name = cfg.scenes[static_cast<std::size_t>(source)].name;
        const std::size_t base_len = strnlen(source_name, kMaxBaseChars);
        std::memcpy(scene.name, source_name, base_len);
        std::memcpy(scene.name + base_len, kCopySuffix, sizeof(kCopySuffix));
    } else if (id < cfg.scene_count) {
        scene = cfg.scenes[id];
    } else {
        std::snprintf(scene.name, sizeof(scene.name), "Custom %u",
                      static_cast<unsigned>(id - rgb::kBuiltInSceneCount + 1));
    }

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) std::snprintf(scene.name, sizeof(scene.name), "%s", v->valuestring);
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) scene.enabled = cJSON_IsTrue(v);

    if (cJSON* zj = cJSON_GetObjectItem(root, "zone"); cJSON_IsNumber(zj)) {
        const int z = zj->valueint;
        if (z >= -1 && z < static_cast<int>(rgb::kMaxZones)) {
            auto& style = z == -1 ? scene.all_lighting : scene.zones[static_cast<std::size_t>(z)];
            if (cJSON* v = cJSON_GetObjectItem(root, "zoneEnabled"); cJSON_IsBool(v)) style.enabled = cJSON_IsTrue(v);
            if (cJSON* v = cJSON_GetObjectItem(root, "brightness"); cJSON_IsNumber(v)) style.brightness = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
            if (cJSON* fx = cJSON_GetObjectItem(root, "effect"); cJSON_IsObject(fx)) {
                if (cJSON* v = cJSON_GetObjectItem(fx, "id"); cJSON_IsString(v)) {
                    if (const auto* d = effects::find_key(v->valuestring)) style.effect.id = d->id;
                }
                parse_color(cJSON_GetObjectItem(fx, "primary"), style.effect.primary);
                parse_color(cJSON_GetObjectItem(fx, "secondary"), style.effect.secondary);
                parse_palette(cJSON_GetObjectItem(fx, "palette"), style.effect);
                if (cJSON* v = cJSON_GetObjectItem(fx, "paletteCount"); cJSON_IsNumber(v)) {
                    style.effect.palette_count = static_cast<uint8_t>(std::clamp(v->valueint, 1, static_cast<int>(rgb::kMaxPaletteColors)));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "speedMs"); cJSON_IsNumber(v)) style.effect.speed_ms = std::clamp(v->valueint, 100, 10000);
                if (cJSON* v = cJSON_GetObjectItem(fx, "width"); cJSON_IsNumber(v)) style.effect.width = static_cast<uint8_t>(std::clamp(v->valueint, 1, 64));
                if (cJSON* v = cJSON_GetObjectItem(fx, "intensity"); cJSON_IsNumber(v)) style.effect.intensity = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
                if (cJSON* v = cJSON_GetObjectItem(fx, "density"); cJSON_IsNumber(v)) style.effect.density = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
                if (cJSON* v = cJSON_GetObjectItem(fx, "reverse"); cJSON_IsBool(v)) style.effect.reverse = cJSON_IsTrue(v);
            }
        }
    }
    cJSON_Delete(root);
    if (config::update_scene(id, scene) != ESP_OK) return send_error(req, "Scene update rejected");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t master_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* v = cJSON_GetObjectItem(root, "value");
    if (!cJSON_IsNumber(v)) { cJSON_Delete(root); return send_error(req, "Missing value"); }
    const uint8_t value = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
    cJSON_Delete(root);
    config::set_master_brightness(value);
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t master_power_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* v = cJSON_GetObjectItem(root, "enabled");
    if (!cJSON_IsBool(v)) { cJSON_Delete(root); return send_error(req, "Missing enabled state"); }
    const bool enabled = cJSON_IsTrue(v);
    cJSON_Delete(root);
    if (config::set_master_power(enabled) != ESP_OK) return send_error(req, "Power state rejected");
    cJSON* out = cJSON_CreateObject();
    cJSON_AddTrueToObject(out, "ok");
    cJSON_AddBoolToObject(out, "enabled", enabled);
    return send_json(req, out);
}

esp_err_t select_scene_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* v = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(v)) { cJSON_Delete(root); return send_error(req, "Missing id"); }
    const int id = v->valueint;
    cJSON_Delete(root);
    if (id < 0 || config::set_active_scene(static_cast<uint8_t>(id)) != ESP_OK) return send_error(req, "Invalid scene");
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t identify_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* v = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(v) || v->valueint < 0 || v->valueint >= static_cast<int>(rgb::kChannelCount)) {
        cJSON_Delete(root); return send_error(req, "Invalid channel");
    }
    lighting::identify(v->valueint);
    cJSON_Delete(root);
    cJSON* out = cJSON_CreateObject(); cJSON_AddTrueToObject(out, "ok"); return send_json(req, out);
}

esp_err_t length_preview_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    cJSON* countj = cJSON_GetObjectItem(root, "count");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kChannelCount) ||
        !cJSON_IsNumber(countj) || countj->valueint < 0 || countj->valueint > static_cast<int>(rgb::kMaxLedsPerChannel)) {
        cJSON_Delete(root);
        return send_error(req, "Invalid channel or count");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    const std::size_t count = static_cast<std::size_t>(countj->valueint);
    cJSON_Delete(root);
    if (count == 0) lighting::stop_length_preview();
    else lighting::preview_length(id, count);
    cJSON* out = cJSON_CreateObject();
    cJSON_AddTrueToObject(out, "ok");
    cJSON_AddNumberToObject(out, "count", count);
    return send_json(req, out);
}

esp_err_t system_post(httpd_req_t* req)
{
    cJSON* root = parse_body(req);
    if (!root) return send_error(req, "Invalid JSON");
    config::snapshot_into(s_web_config);
    auto& cfg = s_web_config;
    auto sys = cfg.system;
    int fade_ms = cfg.power.power_on_fade_ms;
    bool remote_enabled = cfg.remote.enabled;
    int startup_power_mode = static_cast<int>(cfg.startup.power_mode);
    int startup_scene = cfg.startup.scene == rgb::kStartupSceneLast ? -1 : cfg.startup.scene;
    int display_contrast = cfg.remote.oled_contrast;
    int display_timeout = cfg.remote.oled_timeout_s;
    bool display_invert = cfg.remote.oled_invert;
    bool display_auto_dim = cfg.remote.oled_auto_dim;
    int display_screensaver_minutes = cfg.remote.oled_screensaver_minutes;
    bool rtc_enabled = cfg.accessories.rtc_enabled;
    bool als_enabled = cfg.accessories.als_enabled;
    bool reboot_required = false;

    if (cJSON* v = cJSON_GetObjectItem(root, "deviceName"); cJSON_IsString(v)) std::snprintf(sys.device_name, sizeof(sys.device_name), "%s", v->valuestring);
    if (cJSON* v = cJSON_GetObjectItem(root, "ssid"); cJSON_IsString(v)) {
        reboot_required = std::strncmp(sys.ap_ssid, v->valuestring, sizeof(sys.ap_ssid)) != 0;
        std::snprintf(sys.ap_ssid, sizeof(sys.ap_ssid), "%s", v->valuestring);
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "powerOnFadeMs"); cJSON_IsNumber(v)) fade_ms = std::clamp(v->valueint, 0, 10000);
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteControlEnabled"); cJSON_IsBool(v)) remote_enabled = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "startupPowerMode"); cJSON_IsNumber(v)) startup_power_mode = std::clamp(v->valueint, 0, 2);
    if (cJSON* v = cJSON_GetObjectItem(root, "startupScene"); cJSON_IsNumber(v)) startup_scene = v->valueint;
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteDisplayContrast"); cJSON_IsNumber(v)) display_contrast = std::clamp(v->valueint, 0, 255);
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteDisplayTimeout"); cJSON_IsNumber(v)) display_timeout = std::clamp(v->valueint, 0, 3600);
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteDisplayInvert"); cJSON_IsBool(v)) display_invert = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteDisplayAutoDim"); cJSON_IsBool(v)) display_auto_dim = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "remoteScreensaverMinutes"); cJSON_IsNumber(v)) display_screensaver_minutes = std::clamp(v->valueint, 0, 5);
    if (cJSON* v = cJSON_GetObjectItem(root, "rtcEnabled"); cJSON_IsBool(v)) rtc_enabled = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "ambientLightEnabled"); cJSON_IsBool(v)) als_enabled = cJSON_IsTrue(v);
    sys.ap_password[0] = '\0';
    cJSON_Delete(root);

    if (config::update_system(sys) != ESP_OK) return send_error(req, "System settings rejected");
    if (config::set_power_on_fade_ms(static_cast<uint16_t>(fade_ms)) != ESP_OK) return send_error(req, "Power-on fade rejected");
    if (config::set_remote_control_enabled(remote_enabled) != ESP_OK) return send_error(req, "Remote Control setting rejected");
    if (config::set_startup_power_mode(static_cast<rgb::StartupPowerMode>(startup_power_mode)) != ESP_OK) return send_error(req, "Startup power setting rejected");
    if (startup_scene < -1 || (startup_scene >= 0 && startup_scene >= cfg.scene_count)) return send_error(req, "Startup scene rejected");
    const uint8_t startup_scene_value = startup_scene < 0 ? rgb::kStartupSceneLast : static_cast<uint8_t>(startup_scene);
    if (config::set_startup_scene(startup_scene_value) != ESP_OK) return send_error(req, "Startup scene rejected");
    if (config::set_remote_display(static_cast<uint8_t>(display_contrast),
                                   static_cast<uint16_t>(display_timeout),
                                   display_invert,
                                   display_auto_dim,
                                   static_cast<uint8_t>(display_screensaver_minutes)) != ESP_OK) {
        return send_error(req, "Remote display setting rejected");
    }
    if (config::set_accessories_enabled(rtc_enabled, als_enabled) != ESP_OK) return send_error(req, "Accessory settings rejected");

    cJSON* out = cJSON_CreateObject();
    cJSON_AddTrueToObject(out, "ok");
    cJSON_AddBoolToObject(out, "rebootRequired", reboot_required);
    return send_json(req, out);
}

esp_err_t simple_action(httpd_req_t* req, esp_err_t (*action)())
{
    const esp_err_t err = action();
    if (err != ESP_OK) return send_error(req, esp_err_to_name(err), "500 Internal Server Error");
    cJSON* root = cJSON_CreateObject(); cJSON_AddTrueToObject(root, "ok"); return send_json(req, root);
}

esp_err_t save_post(httpd_req_t* req) { return simple_action(req, config::save); }
esp_err_t revert_post(httpd_req_t* req) { return simple_action(req, config::revert); }

esp_err_t factory_post(httpd_req_t* req)
{
    const esp_err_t err = config::factory_reset(true);
    if (err != ESP_OK) return send_error(req, esp_err_to_name(err), "500 Internal Server Error");
    cJSON* root = cJSON_CreateObject(); cJSON_AddTrueToObject(root, "ok"); cJSON_AddBoolToObject(root, "rebootRecommended", true); return send_json(req, root);
}

void add_uri(httpd_handle_t server, const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*))
{
    httpd_uri_t item{};
    item.uri = uri;
    item.method = method;
    item.handler = handler;
    httpd_register_uri_handler(server, &item);
}

} // namespace

namespace webui {

esp_err_t start()
{
    if (s_server) return ESP_OK;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 40; // Includes the zone/scene deletion routes
    cfg.stack_size = 8192;

    // Phones can abandon captive-portal sockets without a graceful close when
    // the user leaves the sheet or Wi-Fi roams. Let the HTTP server evict the
    // least-recently-used session instead of rejecting the next connection.
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 3;
    cfg.send_wait_timeout = 3;
    const esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;

    add_uri(s_server, "/", HTTP_GET, root_get);
    add_uri(s_server, "/assets/auto-rgb-logo.webp", HTTP_GET, logo_get);

    // Use both captive mechanisms: the legacy wildcard-DNS/probe path that
    // XBOX-RGB has proven reliable, plus the standards-based CAPPORT URI
    // advertised in DHCP option 114. ESP-IDF documents these as complementary.
    // The client OS still decides whether to display its captive assistant.
    add_uri(s_server, "/generate_204", HTTP_GET, captive_probe_get);       // Android
    add_uri(s_server, "/gen_204", HTTP_GET, captive_probe_get);            // Android variants
    add_uri(s_server, "/hotspot-detect.html", HTTP_GET, captive_probe_get);// Apple
    add_uri(s_server, "/library/test/success.html", HTTP_GET, captive_probe_get); // Apple
    add_uri(s_server, "/redirect", HTTP_GET, captive_probe_get);           // Apple/legacy
    add_uri(s_server, "/connecttest.txt", HTTP_GET, captive_probe_get);     // Windows
    add_uri(s_server, "/ncsi.txt", HTTP_GET, captive_probe_get);           // Windows
    add_uri(s_server, "/canonical.html", HTTP_GET, captive_probe_get);
    add_uri(s_server, "/success.txt", HTTP_GET, captive_probe_get);
    add_uri(s_server, "/captiveportal", HTTP_GET, captive_probe_get);
    add_uri(s_server, "/captive-portal", HTTP_GET, capport_get);
    add_uri(s_server, "/kindle-wifi/wifistub.html", HTTP_GET, captive_probe_get);
    add_uri(s_server, "/api/status", HTTP_GET, status_get);
    add_uri(s_server, "/api/config", HTTP_GET, config_get);
    add_uri(s_server, "/api/control", HTTP_GET, control_get);
    add_uri(s_server, "/api/accessories", HTTP_GET, accessories_get);
    add_uri(s_server, "/api/effects", HTTP_GET, effects_get);
    add_uri(s_server, "/api/channel", HTTP_POST, channel_post);
    add_uri(s_server, "/api/zone", HTTP_POST, zone_post);
    add_uri(s_server, "/api/zone/delete", HTTP_POST, zone_delete_post);
    add_uri(s_server, "/api/scene", HTTP_POST, scene_post);
    add_uri(s_server, "/api/scene/delete", HTTP_POST, scene_delete_post);
    add_uri(s_server, "/api/master", HTTP_POST, master_post);
    add_uri(s_server, "/api/master-power", HTTP_POST, master_power_post);
    add_uri(s_server, "/api/select-scene", HTTP_POST, select_scene_post);
    add_uri(s_server, "/api/identify", HTTP_POST, identify_post);
    add_uri(s_server, "/api/length-preview", HTTP_POST, length_preview_post);
    add_uri(s_server, "/api/system", HTTP_POST, system_post);
    add_uri(s_server, "/api/save", HTTP_POST, save_post);
    add_uri(s_server, "/api/revert", HTTP_POST, revert_post);
    add_uri(s_server, "/api/factory-reset", HTTP_POST, factory_post);

    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, captive_404);

    ESP_LOGI(TAG, "Web UI/captive portal started on http://192.168.4.1/");
    return ESP_OK;
}

} // namespace webui
