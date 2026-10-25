#include "ble_control.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "config_store.hpp"
#include "accessories.hpp"
#include "effects.hpp"
#include "lighting.hpp"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_BT_NIMBLE_ENABLED

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace {

constexpr const char* TAG = "ble_control";
constexpr char kDeviceName[] = "Auto-RGB";

// Canonical UUIDs:
// Service  8f0d0001-4b6a-4a86-9e2d-4155544f5247
// Info     8f0d0002-4b6a-4a86-9e2d-4155544f5247
// State    8f0d0003-4b6a-4a86-9e2d-4155544f5247
// RPC RX   8f0d0004-4b6a-4a86-9e2d-4155544f5247
// RPC TX   8f0d0005-4b6a-4a86-9e2d-4155544f5247
//
// BLE_UUID128_INIT takes little-endian UUID bytes.
static const ble_uuid128_t kServiceUuid =
    BLE_UUID128_INIT(0x47,0x52,0x4f,0x54,0x55,0x41,0x2d,0x9e,
                     0x86,0x4a,0x6a,0x4b,0x01,0x00,0x0d,0x8f);
static const ble_uuid128_t kInfoUuid =
    BLE_UUID128_INIT(0x47,0x52,0x4f,0x54,0x55,0x41,0x2d,0x9e,
                     0x86,0x4a,0x6a,0x4b,0x02,0x00,0x0d,0x8f);
static const ble_uuid128_t kStateUuid =
    BLE_UUID128_INIT(0x47,0x52,0x4f,0x54,0x55,0x41,0x2d,0x9e,
                     0x86,0x4a,0x6a,0x4b,0x03,0x00,0x0d,0x8f);
static const ble_uuid128_t kRpcRxUuid =
    BLE_UUID128_INIT(0x47,0x52,0x4f,0x54,0x55,0x41,0x2d,0x9e,
                     0x86,0x4a,0x6a,0x4b,0x04,0x00,0x0d,0x8f);
static const ble_uuid128_t kRpcTxUuid =
    BLE_UUID128_INIT(0x47,0x52,0x4f,0x54,0x55,0x41,0x2d,0x9e,
                     0x86,0x4a,0x6a,0x4b,0x05,0x00,0x0d,0x8f);

constexpr uint8_t kFrameMagic = 0xA5;
constexpr uint8_t kFrameStart = 0x01;
constexpr uint8_t kFrameEnd = 0x02;
constexpr std::size_t kFrameBytes = 20;      // works with default ATT MTU 23
constexpr std::size_t kFrameHeaderBytes = 4;
constexpr std::size_t kFramePayloadBytes = kFrameBytes - kFrameHeaderBytes;
constexpr std::size_t kRpcMaxBytes = 4096;
// Avoid heap growth/reallocations while serializing small JSON responses.
// Larger legacy replies still use cJSON's dynamic fallback.
constexpr std::size_t kRpcRenderBytes = 8192;

struct __attribute__((packed)) InfoPacket {
    uint8_t protocol{ble_control::kProtocolVersion};
    uint8_t reserved{0};
    uint16_t schema{rgb::kConfigSchemaVersion};
    uint16_t max_rpc{kRpcMaxBytes};
    uint16_t reserved2{0};
    char firmware[12]{};
};

struct __attribute__((packed)) StatePacket {
    uint8_t protocol{ble_control::kProtocolVersion};
    uint8_t flags{0}; // bit0 master power, bit1 dirty
    uint8_t master_brightness{0};
    uint8_t active_scene{0};
    uint8_t scene_count{0};
    uint8_t enabled_channels{0};
    uint16_t reserved{0};
    uint32_t revision{0};
};

struct RpcRequest {
    uint8_t message_id{0};
    uint16_t length{0};
    uint32_t generation{0};
};

QueueHandle_t s_rpc_queue = nullptr;
TaskHandle_t s_worker_task = nullptr;
TaskHandle_t s_host_task = nullptr;

volatile bool s_started = false;
volatile bool s_connected = false;
volatile bool s_stack_synced = false;
volatile bool s_state_subscribed = false;
volatile bool s_rpc_subscribed = false;
volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
volatile uint32_t s_connection_generation = 0;
volatile uint32_t s_connect_count = 0;
volatile uint32_t s_disconnect_count = 0;
volatile uint32_t s_host_reset_count = 0;
volatile uint32_t s_adv_start_count = 0;
volatile uint32_t s_notify_failure_count = 0;
volatile uint32_t s_aborted_rpc_responses = 0;
volatile int s_last_disconnect_reason = 0;
volatile int s_last_reset_reason = 0;
volatile uint16_t s_last_mtu = 23;
int64_t s_last_adv_attempt_us = 0;

uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

uint16_t s_info_handle = 0;
uint16_t s_state_handle = 0;
uint16_t s_rpc_rx_handle = 0;
uint16_t s_rpc_tx_handle = 0;

InfoPacket s_info{};
StatePacket s_last_state{};
std::array<uint8_t, kFrameBytes> s_last_tx_frame{};
std::size_t s_last_tx_len = 0;

std::array<char, kRpcMaxBytes> s_rx{};
std::array<char, kRpcMaxBytes> s_request_json{};
std::array<char, kRpcRenderBytes> s_response_json{}; // Worker task only; not a stack allocation.
std::size_t s_rx_len = 0;
uint8_t s_rx_message_id = 0;
uint8_t s_rx_next_sequence = 0;
bool s_rx_active = false;
volatile bool s_request_pending = false;

void refresh_info()
{
    s_info = {};
    s_info.protocol = ble_control::kProtocolVersion;
    s_info.schema = rgb::kConfigSchemaVersion;
    s_info.max_rpc = static_cast<uint16_t>(kRpcMaxBytes);
    const char* version = esp_app_get_description()->version;
    std::snprintf(s_info.firmware, sizeof(s_info.firmware), "%.11s", version ? version : "");
}

StatePacket make_state()
{
    const auto st = config::control_snapshot();
    StatePacket out{};
    out.flags = (st.master_power ? 0x01 : 0x00) | (st.dirty ? 0x02 : 0x00);
    out.master_brightness = st.master_brightness;
    out.active_scene = st.active_scene;
    out.scene_count = st.scene_count;
    out.revision = st.revision;

    // Do NOT create a full DeviceConfig on the NimBLE host stack.
    // DeviceConfig is several KB and state_access() runs directly in the
    // nimble_host task. The previous local snapshot could exhaust the 4 KB
    // host stack as soon as a Web Bluetooth client read the State
    // characteristic. enabled_channels is not consumed by the Phase-1 page,
    // so leave it reserved until we expose the count through a lightweight
    // config snapshot API.
    out.enabled_channels = 0;
    return out;
}

int append_to_mbuf(struct os_mbuf* om, const void* data, std::size_t len)
{
    return os_mbuf_append(om, data, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int info_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    refresh_info();
    return append_to_mbuf(ctxt->om, &s_info, sizeof(s_info));
}

int state_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    s_last_state = make_state();
    return append_to_mbuf(ctxt->om, &s_last_state, sizeof(s_last_state));
}

void reset_rx()
{
    s_rx_len = 0;
    s_rx_message_id = 0;
    s_rx_next_sequence = 0;
    s_rx_active = false;
    s_rx[0] = '\0';
}

int rpc_rx_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    const uint16_t packet_len = OS_MBUF_PKTLEN(ctxt->om);
    if (packet_len < kFrameHeaderBytes || packet_len > 244) {
        reset_rx();
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    std::array<uint8_t, 244> flat{};
    uint16_t copied = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, flat.data(), packet_len, &copied) != 0 || copied != packet_len) {
        reset_rx();
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (flat[0] != kFrameMagic) {
        reset_rx();
        return BLE_ATT_ERR_UNLIKELY;
    }

    const uint8_t message_id = flat[1];
    const uint8_t flags = flat[2];
    const uint8_t sequence = flat[3];
    const bool start = (flags & kFrameStart) != 0;
    const bool end = (flags & kFrameEnd) != 0;

    if (start) {
        if (s_request_pending) return BLE_ATT_ERR_INSUFFICIENT_RES;
        reset_rx();
        s_rx_active = true;
        s_rx_message_id = message_id;
    }

    if (!s_rx_active || s_rx_message_id != message_id || sequence != s_rx_next_sequence) {
        reset_rx();
        return BLE_ATT_ERR_UNLIKELY;
    }

    const std::size_t payload_len = packet_len - kFrameHeaderBytes;
    if (s_rx_len + payload_len >= s_rx.size()) {
        reset_rx();
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (payload_len) {
        std::memcpy(s_rx.data() + s_rx_len, flat.data() + kFrameHeaderBytes, payload_len);
        s_rx_len += payload_len;
    }
    s_rx[s_rx_len] = '\0';
    ++s_rx_next_sequence;

    if (end) {
        if (!s_rpc_queue || s_request_pending) {
            reset_rx();
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        // Copy the completed request away from the NimBLE RX assembly buffer.
        // This lets disconnect/reset handling safely clear s_rx immediately
        // without racing the worker while it parses a request.
        std::memcpy(s_request_json.data(), s_rx.data(), s_rx_len + 1);

        RpcRequest request{};
        request.message_id = s_rx_message_id;
        request.length = static_cast<uint16_t>(s_rx_len);
        request.generation = s_connection_generation;
        s_request_pending = true;

        reset_rx();

        if (xQueueSend(s_rpc_queue, &request, 0) != pdTRUE) {
            s_request_pending = false;
            s_request_json[0] = '\0';
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    return 0;
}

int rpc_tx_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return append_to_mbuf(ctxt->om, s_last_tx_frame.data(), s_last_tx_len);
}

static const struct ble_gatt_svc_def kGattServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &kInfoUuid.u,
                .access_cb = info_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &s_info_handle,
            },
            {
                .uuid = &kStateUuid.u,
                .access_cb = state_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_state_handle,
            },
            {
                .uuid = &kRpcRxUuid.u,
                .access_cb = rpc_rx_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &s_rpc_rx_handle,
            },
            {
                .uuid = &kRpcTxUuid.u,
                .access_cb = rpc_tx_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_rpc_tx_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

void register_cb(struct ble_gatt_register_ctxt* ctxt, void*)
{
    char buf[BLE_UUID_STR_LEN]{};
    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGD(TAG, "service %s handle=%u",
                     ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                     static_cast<unsigned>(ctxt->svc.handle));
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGD(TAG, "characteristic %s value_handle=%u",
                     ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                     static_cast<unsigned>(ctxt->chr.val_handle));
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            break;
        default:
            break;
    }
}

void advertise();

int gap_event(struct ble_gap_event* event, void*)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ++s_connection_generation;
                ++s_connect_count;
                s_conn_handle = event->connect.conn_handle;
                s_connected = true;
                s_state_subscribed = false;
                s_rpc_subscribed = false;
                reset_rx();
                ESP_LOGI(TAG, "BLE client connected; handle=%u generation=%lu",
                         static_cast<unsigned>(event->connect.conn_handle),
                         static_cast<unsigned long>(s_connection_generation));
            } else {
                ESP_LOGW(TAG, "BLE connect failed; status=%d", event->connect.status);
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE client disconnected; reason=%d", event->disconnect.reason);
            ++s_disconnect_count;
            s_last_disconnect_reason = event->disconnect.reason;
            ++s_connection_generation; // invalidate any response still in flight
            s_connected = false;
            s_state_subscribed = false;
            s_rpc_subscribed = false;
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            reset_rx();
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            // Maintenance in worker_task() will restart advertising. Keeping
            // the retry out of this callback avoids recursive failure loops.
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_state_handle) {
                s_state_subscribed = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "state notifications %s", s_state_subscribed ? "enabled" : "disabled");
            }
            if (event->subscribe.attr_handle == s_rpc_tx_handle) {
                s_rpc_subscribed = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "RPC notifications %s", s_rpc_subscribed ? "enabled" : "disabled");
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            s_last_mtu = event->mtu.value;
            ESP_LOGI(TAG, "BLE MTU=%u", static_cast<unsigned>(event->mtu.value));
            return 0;

        case BLE_GAP_EVENT_NOTIFY_TX:
            if (event->notify_tx.status != 0 && event->notify_tx.status != BLE_HS_EDONE) {
                ESP_LOGD(TAG, "notify tx status=%d handle=%u",
                         event->notify_tx.status,
                         static_cast<unsigned>(event->notify_tx.attr_handle));
            }
            return 0;

        default:
            return 0;
    }
}

void advertise()
{
    if (!s_stack_synced || s_connected || ble_gap_adv_active()) return;
    s_last_adv_attempt_us = esp_timer_get_time();

    struct ble_hs_adv_fields adv{};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.uuids128 = const_cast<ble_uuid128_t*>(&kServiceUuid);
    adv.num_uuids128 = 1;
    adv.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        ESP_LOGE(TAG, "set advertising fields failed: %d", rc);
        return;
    }

    const char* name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields rsp{};
    rsp.name = reinterpret_cast<uint8_t*>(const_cast<char*>(name));
    rsp.name_len = static_cast<uint8_t>(std::strlen(name));
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "set scan response failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params{};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = BLE_GAP_ADV_ITVL_MS(250);
    params.itvl_max = BLE_GAP_ADV_ITVL_MS(300);

    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER, &params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "advertising start failed: %d", rc);
    } else {
        ++s_adv_start_count;
        ESP_LOGI(TAG, "BLE advertising as %s", name);
    }
}

void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset: %d", reason);
    ++s_host_reset_count;
    s_last_reset_reason = reason;
    ++s_connection_generation;
    s_stack_synced = false;
    s_connected = false;
    s_state_subscribed = false;
    s_rpc_subscribed = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    reset_rx();
}

void on_sync()
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "no BLE identity address: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "address inference failed: %d", rc);
        return;
    }
    s_stack_synced = true;
    advertise();
}

void host_task(void*)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    vTaskDelete(nullptr);
}

bool notify_raw(uint16_t value_handle, const void* data, std::size_t len)
{
    if (!data || value_handle == 0) return false;

    // NimBLE owns and consumes the mbuf passed to notify_custom, even on
    // error. Give the radio sufficient time to drain a congested TX queue.
    // One 5.2 KB scene page can contain >300 default-MTU notifications.
    // We do not wait forever, because the phone must be able to recover.
    for (int attempt = 0; attempt < 125; ++attempt) {
        if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
        struct os_mbuf* om = ble_hs_mbuf_from_flat(data, static_cast<uint16_t>(len));
        if (!om) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const int rc = ble_gatts_notify_custom(s_conn_handle, value_handle, om);
        if (rc == 0) return true;
        if (rc == BLE_HS_ENOMEM) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ++s_notify_failure_count;
        ESP_LOGW(TAG, "BLE notify failed handle=%u rc=%d",
                 static_cast<unsigned>(value_handle), rc);
        return false;
    }

    ++s_notify_failure_count;
    ESP_LOGW(TAG, "BLE notification congestion did not clear after ~1.25s");
    return false;
}

void notify_state()
{
    if (!s_state_subscribed) return;
    s_last_state = make_state();
    (void)notify_raw(s_state_handle, &s_last_state, sizeof(s_last_state));
}

void send_rpc_response(uint8_t message_id, uint32_t generation, const char* json)
{
    if (!json || !s_rpc_subscribed || generation != s_connection_generation) return;

    const std::size_t total = std::strlen(json);
    std::size_t offset = 0;
    uint8_t sequence = 0;

    do {
        if (generation != s_connection_generation || !s_connected || !s_rpc_subscribed) return;
        std::array<uint8_t, kFrameBytes> frame{};
        frame[0] = kFrameMagic;
        frame[1] = message_id;
        frame[2] = 0;
        frame[3] = sequence++;

        if (offset == 0) frame[2] |= kFrameStart;
        const std::size_t count = std::min<std::size_t>(kFramePayloadBytes, total - offset);
        if (offset + count >= total) frame[2] |= kFrameEnd;

        if (count) std::memcpy(frame.data() + kFrameHeaderBytes, json + offset, count);
        const std::size_t frame_len = kFrameHeaderBytes + count;

        s_last_tx_frame = frame;
        s_last_tx_len = frame_len;

        // Never silently abandon a partial response. The iPhone cannot
        // reconstruct a lost notification and must be told to reconnect.
        if (!notify_raw(s_rpc_tx_handle, frame.data(), frame_len)) {
            if (generation == s_connection_generation && s_connected &&
                s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                ++s_aborted_rpc_responses;
                ESP_LOGW(TAG, "BLE RPC aborted after %u/%u bytes; resetting link",
                         static_cast<unsigned>(offset), static_cast<unsigned>(total));
                const int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                if (rc != 0) {
                    ESP_LOGW(TAG, "BLE terminate request failed: %d", rc);
                }
            }
            return;
        }
        offset += count;
        // Conservative pacing of default-MTU notifications avoids most bursts.
        if (offset < total) vTaskDelay(pdMS_TO_TICKS(4));
    } while (offset < total);

    // Empty response still needs a complete frame.
    if (total == 0 && generation == s_connection_generation && s_connected && s_rpc_subscribed) {
        std::array<uint8_t, kFrameHeaderBytes> frame{kFrameMagic, message_id,
                                                     static_cast<uint8_t>(kFrameStart | kFrameEnd), 0};
        s_last_tx_len = frame.size();
        std::copy(frame.begin(), frame.end(), s_last_tx_frame.begin());
        (void)notify_raw(s_rpc_tx_handle, frame.data(), frame.size());
    }
}

// -----------------------------------------------------------------------------
// Hosted WebUI API bridge
// -----------------------------------------------------------------------------
// The hosted BLE UI intentionally speaks the same /api/* contract as the
// embedded HTTP UI.  This keeps the existing mobile application logic intact;
// only the transport changes.

rgb::DeviceConfig s_api_config{}; // Keep the several-KB config off task stacks.

cJSON* api_error(const char* message)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddFalseToObject(root, "ok");
    cJSON_AddStringToObject(root, "error", message ? message : "error");
    return root;
}

cJSON* api_ok()
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddTrueToObject(root, "ok");
    return root;
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
    char text[8]{};
    std::snprintf(text, sizeof(text), "#%02X%02X%02X", c.r, c.g, c.b);
    return cJSON_CreateString(text);
}

bool parse_color(const cJSON* item, rgb::Color& c)
{
    if (!cJSON_IsString(item) || !item->valuestring ||
        std::strlen(item->valuestring) != 7 || item->valuestring[0] != '#') {
        return false;
    }
    unsigned r = 0, g = 0, b = 0;
    if (std::sscanf(item->valuestring + 1, "%02x%02x%02x", &r, &g, &b) != 3) return false;
    c = {static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)};
    return true;
}

cJSON* palette_json(const rgb::EffectConfig& fx)
{
    cJSON* arr = cJSON_CreateArray();
    const uint8_t count = static_cast<uint8_t>(
        std::clamp<int>(fx.palette_count, 1, static_cast<int>(rgb::kMaxPaletteColors)));
    cJSON_AddItemToArray(arr, color_json(fx.primary));
    if (count > 1) cJSON_AddItemToArray(arr, color_json(fx.secondary));
    for (uint8_t i = 2; i < count; ++i) {
        cJSON_AddItemToArray(arr, color_json(fx.extra_colors[i - 2]));
    }
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

// A single scene is a bounded BLE response, even with all eight user zones.
// All Lighting remains global (zone = -1); it is never a numbered zone.
cJSON* scene_json(const rgb::DeviceConfig& cfg, std::size_t index)
{
    const auto& scene = cfg.scenes[index];
    cJSON* item = cJSON_CreateObject();
    cJSON_AddNumberToObject(item, "id", index);
    cJSON_AddStringToObject(item, "name", scene.name);
    cJSON_AddBoolToObject(item, "enabled", scene.enabled);
    cJSON_AddBoolToObject(item, "builtIn", index < rgb::kBuiltInSceneCount);
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
    return item;
}

cJSON* config_json(const rgb::DeviceConfig& cfg, uint32_t revision, bool include_scenes)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "schemaVersion", cfg.schema_version);
    cJSON_AddBoolToObject(root, "dirty", config::dirty());
    cJSON_AddNumberToObject(root, "revision", revision);
    cJSON* capabilities = cJSON_AddObjectToObject(root, "capabilities");
    cJSON_AddBoolToObject(capabilities, "deleteZones", true);
    cJSON_AddBoolToObject(capabilities, "deleteScenes", true);
    cJSON_AddBoolToObject(capabilities, "globalLighting", true);
    cJSON_AddBoolToObject(capabilities, "pagedConfig", true);

    cJSON* system = cJSON_AddObjectToObject(root, "system");
    cJSON_AddStringToObject(system, "deviceName", cfg.system.device_name);
    cJSON_AddStringToObject(system, "ssid", cfg.system.ap_ssid);
    cJSON_AddNumberToObject(system, "masterBrightness", cfg.system.master_brightness);
    cJSON_AddNumberToObject(system, "activeScene", cfg.system.active_scene);
    cJSON_AddBoolToObject(system, "masterPower", cfg.power.master_enabled);
    cJSON_AddNumberToObject(system, "powerOnFadeMs", cfg.power.power_on_fade_ms);
    cJSON_AddBoolToObject(system, "remoteControlEnabled", cfg.remote.enabled);
    cJSON_AddNumberToObject(system, "startupPowerMode", static_cast<uint8_t>(cfg.startup.power_mode));
    cJSON_AddNumberToObject(system, "startupScene",
                            cfg.startup.scene == rgb::kStartupSceneLast ? -1 : cfg.startup.scene);
    cJSON_AddNumberToObject(system, "remoteDisplayContrast", cfg.remote.oled_contrast);
    cJSON_AddNumberToObject(system, "remoteDisplayTimeout", cfg.remote.oled_timeout_s);
    cJSON_AddBoolToObject(system, "remoteDisplayInvert", cfg.remote.oled_invert);
    cJSON_AddBoolToObject(system, "remoteDisplayAutoDim", cfg.remote.oled_auto_dim);
    cJSON_AddNumberToObject(system, "remoteScreensaverMinutes", cfg.remote.oled_screensaver_minutes);
    cJSON_AddBoolToObject(system, "rtcEnabled", cfg.accessories.rtc_enabled);
    cJSON_AddBoolToObject(system, "ambientLightEnabled", cfg.accessories.als_enabled);
    cJSON_AddNumberToObject(system, "ambientLightSensitivity",
                            static_cast<uint8_t>(cfg.accessories.als_sensitivity));

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

    cJSON_AddNumberToObject(root, "sceneCount", cfg.scene_count);
    cJSON* scenes = cJSON_AddArrayToObject(root, "scenes");
    if (include_scenes) {
        for (std::size_t index = 0; index < cfg.scene_count; ++index) {
            cJSON_AddItemToArray(scenes, scene_json(cfg, index));
        }
    }
    return root;
}
cJSON* api_status()
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddTrueToObject(root, "ok");
    cJSON_AddStringToObject(root, "version", esp_app_get_description()->version);
    cJSON_AddNumberToObject(root, "uptimeMs", static_cast<double>(esp_timer_get_time() / 1000));
    cJSON_AddBoolToObject(root, "dirty", config::dirty());
    cJSON_AddNumberToObject(root, "revision", config::revision());
    return root;
}

cJSON* api_config(const cJSON* body)
{
    // Native BLE clients request a compact core followed by one scene at a
    // time. The original full /api/config shape remains available to older
    // clients (including the hosted WebUI) for backward compatibility.
    const cJSON* part_item = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "part") : nullptr;
    const char* part = cJSON_IsString(part_item) ? part_item->valuestring : nullptr;
    if (part && std::strcmp(part, "core") && std::strcmp(part, "scene")) {
        return api_error("Invalid config part");
    }
    if (part && !std::strcmp(part, "scene")) {
        const cJSON* id = cJSON_GetObjectItem(body, "id");
        if (!cJSON_IsNumber(id) || id->valuedouble != id->valueint ||
            id->valueint < 0 || id->valueint >= static_cast<int>(rgb::kMaxScenes)) {
            return api_error("Invalid scene id");
        }
        const uint32_t revision = config::snapshot_into_with_revision(s_api_config);
        if (id->valueint >= s_api_config.scene_count) return api_error("Scene does not exist");
        cJSON* root = api_ok();
        if (!root) return nullptr;
        cJSON_AddNumberToObject(root, "revision", revision);
        cJSON_AddItemToObject(root, "scene", scene_json(s_api_config,
                                                        static_cast<std::size_t>(id->valueint)));
        return root;
    }
    const uint32_t revision = config::snapshot_into_with_revision(s_api_config);
    return config_json(s_api_config, revision, part == nullptr);
}

cJSON* api_control()
{
    const auto st = config::control_snapshot();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "revision", st.revision);
    cJSON_AddBoolToObject(root, "dirty", st.dirty);
    cJSON_AddBoolToObject(root, "masterPower", st.master_power);
    cJSON_AddNumberToObject(root, "masterBrightness", st.master_brightness);
    cJSON_AddNumberToObject(root, "activeScene", st.active_scene);
    cJSON_AddBoolToObject(root, "remoteControlEnabled", st.remote_enabled);
    return root;
}

cJSON* api_ble_diagnostics()
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "connected", s_connected);
    cJSON_AddBoolToObject(root, "advertising",
                          s_stack_synced && !s_connected && ble_gap_adv_active());
    cJSON_AddBoolToObject(root, "stackSynced", s_stack_synced);
    cJSON_AddNumberToObject(root, "protocol", ble_control::kProtocolVersion);
    cJSON_AddNumberToObject(root, "schema", rgb::kConfigSchemaVersion);
    cJSON_AddNumberToObject(root, "mtu", s_last_mtu);
    cJSON_AddNumberToObject(root, "connections", s_connect_count);
    cJSON_AddNumberToObject(root, "disconnects", s_disconnect_count);
    cJSON_AddNumberToObject(root, "lastDisconnectReason", s_last_disconnect_reason);
    cJSON_AddNumberToObject(root, "hostResets", s_host_reset_count);
    cJSON_AddNumberToObject(root, "lastResetReason", s_last_reset_reason);
    cJSON_AddNumberToObject(root, "advertisingStarts", s_adv_start_count);
    cJSON_AddNumberToObject(root, "notifyFailures", s_notify_failure_count);
    cJSON_AddNumberToObject(root, "abortedRpcResponses", s_aborted_rpc_responses);
    cJSON_AddNumberToObject(root, "generation", s_connection_generation);
    cJSON_AddNumberToObject(root, "hostStackHighWater",
                            s_host_task ? uxTaskGetStackHighWaterMark(s_host_task) : 0);
    cJSON_AddNumberToObject(root, "workerStackHighWater",
                            s_worker_task ? uxTaskGetStackHighWaterMark(s_worker_task) : 0);
    return root;
}

cJSON* api_accessories()
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
    cJSON_AddNumberToObject(als, "sensitivity", static_cast<uint8_t>(st.als_sensitivity));
    cJSON_AddNumberToObject(als, "raw", st.als_raw);
    cJSON_AddNumberToObject(als, "lux", st.lux);
    return root;
}

cJSON* api_effects()
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
    return root;
}

cJSON* api_channel(const cJSON* root)
{
    if (!cJSON_IsObject(root)) return api_error("Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 ||
        idj->valueint >= static_cast<int>(rgb::kChannelCount)) {
        return api_error("Invalid channel id");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    config::snapshot_into(s_api_config);
    auto ch = s_api_config.channels[id];

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) {
        std::snprintf(ch.name, sizeof(ch.name), "%s", v->valuestring);
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "ledCount"); cJSON_IsNumber(v)) {
        ch.led_count = static_cast<uint16_t>(std::clamp(v->valueint, 0, static_cast<int>(rgb::kMaxLedsPerChannel)));
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "brightness"); cJSON_IsNumber(v)) {
        ch.brightness = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) ch.enabled = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "reversed"); cJSON_IsBool(v)) ch.reversed = cJSON_IsTrue(v);
    if (cJSON* v = cJSON_GetObjectItem(root, "order"); cJSON_IsString(v)) ch.order = parse_pixel_order(v->valuestring);

    return config::update_channel(id, ch) == ESP_OK ? api_ok() : api_error("Channel update rejected");
}

cJSON* api_zone(const cJSON* root)
{
    if (!cJSON_IsObject(root)) return api_error("Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kMaxZones)) {
        return api_error("Invalid zone id");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    config::snapshot_into(s_api_config);
    rgb::ZoneConfig zone = id < s_api_config.zone_count ? s_api_config.zones[id] : rgb::ZoneConfig{};
    if (id >= s_api_config.zone_count) {
        std::snprintf(zone.name, sizeof(zone.name), "Zone %u", static_cast<unsigned>(id + 1));
    }

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) {
        std::snprintf(zone.name, sizeof(zone.name), "%s", v->valuestring);
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) zone.enabled = cJSON_IsTrue(v);
    if (cJSON* segs = cJSON_GetObjectItem(root, "segments"); cJSON_IsArray(segs)) {
        zone.segment_count = 0;
        const int n = std::min(cJSON_GetArraySize(segs), static_cast<int>(rgb::kMaxSegmentsPerZone));
        for (int i = 0; i < n; ++i) {
            cJSON* sj = cJSON_GetArrayItem(segs, i);
            cJSON* cj = cJSON_GetObjectItem(sj, "channel");
            if (!cJSON_IsNumber(cj) || cj->valueint < 0 ||
                cj->valueint >= static_cast<int>(rgb::kChannelCount)) continue;
            auto& seg = zone.segments[zone.segment_count++];
            seg.channel = static_cast<uint8_t>(cj->valueint);
            if (cJSON* v = cJSON_GetObjectItem(sj, "start"); cJSON_IsNumber(v)) {
                seg.start = static_cast<uint16_t>(std::max(v->valueint, 0));
            }
            if (cJSON* v = cJSON_GetObjectItem(sj, "length"); cJSON_IsNumber(v)) {
                seg.length = static_cast<uint16_t>(std::max(v->valueint, 0));
            }
            if (cJSON* v = cJSON_GetObjectItem(sj, "reversed"); cJSON_IsBool(v)) seg.reversed = cJSON_IsTrue(v);
        }
    }

    return config::update_zone(id, zone) == ESP_OK ? api_ok() : api_error("Zone update rejected");
}

cJSON* api_scene(const cJSON* root)
{
    if (!cJSON_IsObject(root)) return api_error("Invalid JSON");
    cJSON* idj = cJSON_GetObjectItem(root, "id");
    if (!cJSON_IsNumber(idj) || idj->valueint < 0 || idj->valueint >= static_cast<int>(rgb::kMaxScenes)) {
        return api_error("Invalid scene id");
    }
    const std::size_t id = static_cast<std::size_t>(idj->valueint);
    if (id < rgb::kBuiltInSceneCount) return api_error("Built-in demo scenes are read-only");

    config::snapshot_into(s_api_config);
    if (id > s_api_config.scene_count) return api_error("Scene slots must be created in order");

    rgb::SceneConfig scene{};
    if (cJSON* clone = cJSON_GetObjectItem(root, "cloneFrom"); cJSON_IsNumber(clone)) {
        const int source = clone->valueint;
        if (source < 0 || source >= static_cast<int>(s_api_config.scene_count)) {
            return api_error("Invalid source scene");
        }
        scene = s_api_config.scenes[static_cast<std::size_t>(source)];
        constexpr char kCopySuffix[] = " Copy";
        constexpr std::size_t kSuffixChars = sizeof(kCopySuffix) - 1;
        constexpr std::size_t kMaxBaseChars = sizeof(scene.name) - 1 - kSuffixChars;
        const char* source_name = s_api_config.scenes[static_cast<std::size_t>(source)].name;
        const std::size_t base_len = strnlen(source_name, kMaxBaseChars);
        std::memcpy(scene.name, source_name, base_len);
        std::memcpy(scene.name + base_len, kCopySuffix, sizeof(kCopySuffix));
    } else if (id < s_api_config.scene_count) {
        scene = s_api_config.scenes[id];
    } else {
        std::snprintf(scene.name, sizeof(scene.name), "Custom %u",
                      static_cast<unsigned>(id - rgb::kBuiltInSceneCount + 1));
    }

    if (cJSON* v = cJSON_GetObjectItem(root, "name"); cJSON_IsString(v)) {
        std::snprintf(scene.name, sizeof(scene.name), "%s", v->valuestring);
    }
    if (cJSON* v = cJSON_GetObjectItem(root, "enabled"); cJSON_IsBool(v)) scene.enabled = cJSON_IsTrue(v);

    if (cJSON* zj = cJSON_GetObjectItem(root, "zone"); cJSON_IsNumber(zj)) {
        const int z = zj->valueint;
        if (z >= -1 && z < static_cast<int>(rgb::kMaxZones)) {
            auto& style = z == -1 ? scene.all_lighting : scene.zones[static_cast<std::size_t>(z)];
            if (cJSON* v = cJSON_GetObjectItem(root, "zoneEnabled"); cJSON_IsBool(v)) style.enabled = cJSON_IsTrue(v);
            if (cJSON* v = cJSON_GetObjectItem(root, "brightness"); cJSON_IsNumber(v)) {
                style.brightness = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
            }
            if (cJSON* fx = cJSON_GetObjectItem(root, "effect"); cJSON_IsObject(fx)) {
                if (cJSON* v = cJSON_GetObjectItem(fx, "id"); cJSON_IsString(v)) {
                    if (const auto* d = effects::find_key(v->valuestring)) style.effect.id = d->id;
                }
                parse_color(cJSON_GetObjectItem(fx, "primary"), style.effect.primary);
                parse_color(cJSON_GetObjectItem(fx, "secondary"), style.effect.secondary);
                parse_palette(cJSON_GetObjectItem(fx, "palette"), style.effect);
                if (cJSON* v = cJSON_GetObjectItem(fx, "paletteCount"); cJSON_IsNumber(v)) {
                    style.effect.palette_count = static_cast<uint8_t>(
                        std::clamp(v->valueint, 1, static_cast<int>(rgb::kMaxPaletteColors)));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "speedMs"); cJSON_IsNumber(v)) {
                    style.effect.speed_ms = static_cast<uint16_t>(std::clamp(v->valueint, 100, 10000));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "width"); cJSON_IsNumber(v)) {
                    style.effect.width = static_cast<uint8_t>(std::clamp(v->valueint, 1, 64));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "intensity"); cJSON_IsNumber(v)) {
                    style.effect.intensity = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "density"); cJSON_IsNumber(v)) {
                    style.effect.density = static_cast<uint8_t>(std::clamp(v->valueint, 0, 255));
                }
                if (cJSON* v = cJSON_GetObjectItem(fx, "reverse"); cJSON_IsBool(v)) {
                    style.effect.reverse = cJSON_IsTrue(v);
                }
            }
        }
    }

    return config::update_scene(id, scene) == ESP_OK ? api_ok() : api_error("Scene update rejected");
}

// Same revision-checked removal contract as Wi-Fi HTTP. This shifts entries
// inside one locked update, including per-scene zone effect assignments.
cJSON* api_zone_delete(const cJSON* body)
{
    if (!cJSON_IsObject(body)) return api_error("Invalid JSON");
    const cJSON* id = cJSON_GetObjectItem(body, "id");
    const cJSON* rev = cJSON_GetObjectItem(body, "expectedRevision");
    if (!cJSON_IsNumber(id) || id->valueint < 0 ||
        id->valueint >= static_cast<int>(rgb::kMaxZones) ||
        !cJSON_IsNumber(rev) || rev->valuedouble < 0 || rev->valuedouble > 4294967295.0) {
        return api_error("Invalid zone or revision");
    }
    const esp_err_t result = config::remove_zone(static_cast<std::size_t>(id->valueint),
                                                  static_cast<uint32_t>(rev->valuedouble));
    if (result == ESP_ERR_INVALID_STATE) return api_error("Configuration changed; refresh and retry");
    return result == ESP_OK ? api_ok() : api_error("Zone deletion rejected");
}

cJSON* api_scene_delete(const cJSON* body)
{
    if (!cJSON_IsObject(body)) return api_error("Invalid JSON");
    const cJSON* id = cJSON_GetObjectItem(body, "id");
    const cJSON* rev = cJSON_GetObjectItem(body, "expectedRevision");
    if (!cJSON_IsNumber(id) ||
        id->valueint < static_cast<int>(rgb::kBuiltInSceneCount) ||
        id->valueint >= static_cast<int>(rgb::kMaxScenes) ||
        !cJSON_IsNumber(rev) || rev->valuedouble < 0 || rev->valuedouble > 4294967295.0) {
        return api_error("Cannot delete built-in scene or invalid revision");
    }
    const esp_err_t result = config::remove_custom_scene(static_cast<std::size_t>(id->valueint),
                                                          static_cast<uint32_t>(rev->valuedouble));
    if (result == ESP_ERR_INVALID_STATE) return api_error("Configuration changed; refresh and retry");
    return result == ESP_OK ? api_ok() : api_error("Scene deletion rejected");
}

cJSON* api_system(const cJSON* root)
{
    if (!cJSON_IsObject(root)) return api_error("Invalid JSON");
    config::snapshot_into(s_api_config);
    auto sys = s_api_config.system;
    int fade_ms = s_api_config.power.power_on_fade_ms;
    bool remote_enabled = s_api_config.remote.enabled;
    int startup_power_mode = static_cast<int>(s_api_config.startup.power_mode);
    int startup_scene = s_api_config.startup.scene == rgb::kStartupSceneLast ? -1 : s_api_config.startup.scene;
    int display_contrast = s_api_config.remote.oled_contrast;
    int display_timeout = s_api_config.remote.oled_timeout_s;
    bool display_invert = s_api_config.remote.oled_invert;
    bool display_auto_dim = s_api_config.remote.oled_auto_dim;
    int display_screensaver_minutes = s_api_config.remote.oled_screensaver_minutes;
    bool rtc_enabled = s_api_config.accessories.rtc_enabled;
    bool als_enabled = s_api_config.accessories.als_enabled;
    int als_sensitivity = static_cast<int>(s_api_config.accessories.als_sensitivity);
    bool reboot_required = false;

    if (cJSON* v = cJSON_GetObjectItem(root, "deviceName"); cJSON_IsString(v)) {
        std::snprintf(sys.device_name, sizeof(sys.device_name), "%s", v->valuestring);
    }
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
    if (cJSON* v = cJSON_GetObjectItem(root, "ambientLightSensitivity"); cJSON_IsNumber(v)) als_sensitivity = std::clamp(v->valueint, 0, 3);
    sys.ap_password[0] = '\0';

    if (config::update_system(sys) != ESP_OK) return api_error("System settings rejected");
    if (config::set_power_on_fade_ms(static_cast<uint16_t>(fade_ms)) != ESP_OK) return api_error("Power-on fade rejected");
    if (config::set_remote_control_enabled(remote_enabled) != ESP_OK) return api_error("Remote Control setting rejected");
    if (config::set_startup_power_mode(static_cast<rgb::StartupPowerMode>(startup_power_mode)) != ESP_OK) return api_error("Startup power setting rejected");
    if (startup_scene < -1 || (startup_scene >= 0 && startup_scene >= s_api_config.scene_count)) return api_error("Startup scene rejected");
    const uint8_t startup_scene_value = startup_scene < 0 ? rgb::kStartupSceneLast : static_cast<uint8_t>(startup_scene);
    if (config::set_startup_scene(startup_scene_value) != ESP_OK) return api_error("Startup scene rejected");
    if (config::set_remote_display(static_cast<uint8_t>(display_contrast),
                                   static_cast<uint16_t>(display_timeout),
                                   display_invert,
                                   display_auto_dim,
                                   static_cast<uint8_t>(display_screensaver_minutes)) != ESP_OK) {
        return api_error("Remote display setting rejected");
    }
    if (config::set_accessories_enabled(rtc_enabled, als_enabled) != ESP_OK) return api_error("Accessory settings rejected");
    if (config::set_als_sensitivity(static_cast<rgb::AlsSensitivity>(als_sensitivity)) != ESP_OK) return api_error("ALS sensitivity rejected");

    cJSON* out = api_ok();
    cJSON_AddBoolToObject(out, "rebootRequired", reboot_required);
    return out;
}

cJSON* api_rtc_set(const cJSON* root)
{
    if (!cJSON_IsObject(root)) return api_error("Invalid JSON");
    auto number = [&](const char* key, int min_value, int max_value, int& out) -> bool {
        cJSON* item = cJSON_GetObjectItem(root, key);
        if (!cJSON_IsNumber(item) || item->valueint < min_value || item->valueint > max_value) return false;
        out = item->valueint;
        return true;
    };

    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    const bool valid = number("year", 2000, 2199, year) &&
                       number("month", 1, 12, month) &&
                       number("day", 1, 31, day) &&
                       number("hour", 0, 23, hour) &&
                       number("minute", 0, 59, minute) &&
                       number("second", 0, 59, second);
    if (!valid) return api_error("Invalid RTC date/time");

    accessories::RtcDateTime value{};
    value.year = static_cast<uint16_t>(year);
    value.month = static_cast<uint8_t>(month);
    value.day = static_cast<uint8_t>(day);
    value.hour = static_cast<uint8_t>(hour);
    value.minute = static_cast<uint8_t>(minute);
    value.second = static_cast<uint8_t>(second);
    const esp_err_t err = accessories::set_rtc_time(value);
    return err == ESP_OK ? api_ok() : api_error(esp_err_to_name(err));
}

cJSON* api_dispatch(const char* path, const cJSON* body)
{
    if (!path) return api_error("missing_path");

    if (!std::strcmp(path, "/api/status")) return api_status();
    if (!std::strcmp(path, "/api/config")) return api_config(body);
    if (!std::strcmp(path, "/api/control")) return api_control();
    if (!std::strcmp(path, "/api/ble")) return api_ble_diagnostics();
    if (!std::strcmp(path, "/api/accessories")) return api_accessories();
    if (!std::strcmp(path, "/api/effects")) return api_effects();
    if (!std::strcmp(path, "/api/channel")) return api_channel(body);
    if (!std::strcmp(path, "/api/zone")) return api_zone(body);
    if (!std::strcmp(path, "/api/scene")) return api_scene(body);
    if (!std::strcmp(path, "/api/zone/delete")) return api_zone_delete(body);
    if (!std::strcmp(path, "/api/scene/delete")) return api_scene_delete(body);

    if (!std::strcmp(path, "/api/master")) {
        cJSON* v = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "value") : nullptr;
        if (!cJSON_IsNumber(v)) return api_error("Missing value");
        config::set_master_brightness(static_cast<uint8_t>(std::clamp(v->valueint, 0, 255)));
        return api_ok();
    }

    if (!std::strcmp(path, "/api/master-power")) {
        cJSON* v = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "enabled") : nullptr;
        if (!cJSON_IsBool(v)) return api_error("Missing enabled state");
        const bool enabled = cJSON_IsTrue(v);
        if (config::set_master_power(enabled) != ESP_OK) return api_error("Power state rejected");
        cJSON* out = api_ok();
        cJSON_AddBoolToObject(out, "enabled", enabled);
        return out;
    }

    if (!std::strcmp(path, "/api/select-scene")) {
        cJSON* v = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "id") : nullptr;
        if (!cJSON_IsNumber(v) || v->valueint < 0 ||
            config::set_active_scene(static_cast<uint8_t>(v->valueint)) != ESP_OK) {
            return api_error("Invalid scene");
        }
        return api_ok();
    }

    if (!std::strcmp(path, "/api/identify")) {
        cJSON* v = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "id") : nullptr;
        if (!cJSON_IsNumber(v) || v->valueint < 0 ||
            v->valueint >= static_cast<int>(rgb::kChannelCount)) {
            return api_error("Invalid channel");
        }
        lighting::identify(static_cast<std::size_t>(v->valueint));
        return api_ok();
    }

    if (!std::strcmp(path, "/api/length-preview")) {
        cJSON* idj = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "id") : nullptr;
        cJSON* countj = cJSON_IsObject(body) ? cJSON_GetObjectItem(body, "count") : nullptr;
        if (!cJSON_IsNumber(idj) || idj->valueint < 0 ||
            idj->valueint >= static_cast<int>(rgb::kChannelCount) ||
            !cJSON_IsNumber(countj) || countj->valueint < 0 ||
            countj->valueint > static_cast<int>(rgb::kMaxLedsPerChannel)) {
            return api_error("Invalid channel or count");
        }
        const std::size_t id = static_cast<std::size_t>(idj->valueint);
        const std::size_t count = static_cast<std::size_t>(countj->valueint);
        if (count == 0) lighting::stop_length_preview();
        else lighting::preview_length(id, count);
        cJSON* out = api_ok();
        cJSON_AddNumberToObject(out, "count", count);
        return out;
    }

    if (!std::strcmp(path, "/api/system")) return api_system(body);
    if (!std::strcmp(path, "/api/rtc/set")) return api_rtc_set(body);

    if (!std::strcmp(path, "/api/save")) {
        const esp_err_t err = config::save();
        return err == ESP_OK ? api_ok() : api_error(esp_err_to_name(err));
    }
    if (!std::strcmp(path, "/api/revert")) {
        const esp_err_t err = config::revert();
        return err == ESP_OK ? api_ok() : api_error(esp_err_to_name(err));
    }
    if (!std::strcmp(path, "/api/factory-reset")) {
        const esp_err_t err = config::factory_reset(true);
        if (err != ESP_OK) return api_error(esp_err_to_name(err));
        cJSON* out = api_ok();
        cJSON_AddBoolToObject(out, "rebootRecommended", true);
        return out;
    }

    return api_error("unsupported_path");
}


void add_state_json(cJSON* root)
{
    const auto st = config::control_snapshot();
    cJSON_AddNumberToObject(root, "revision", st.revision);
    cJSON_AddBoolToObject(root, "dirty", st.dirty);
    cJSON_AddBoolToObject(root, "masterPower", st.master_power);
    cJSON_AddNumberToObject(root, "masterBrightness", st.master_brightness);
    cJSON_AddNumberToObject(root, "activeScene", st.active_scene);
    cJSON_AddNumberToObject(root, "sceneCount", st.scene_count);

    cJSON* scenes = cJSON_AddArrayToObject(root, "scenes");
    for (uint8_t i = 0; i < st.scene_count && i < rgb::kMaxScenes; ++i) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", i);
        cJSON_AddStringToObject(item, "name", st.scene_names[i].data());
        cJSON_AddItemToArray(scenes, item);
    }
}

void handle_rpc(const RpcRequest& request)
{
    // Requests are copied into s_request_json before the host callback queues
    // them, so a disconnect/reset can safely clear the RX assembly state.
    cJSON* in = cJSON_ParseWithLength(s_request_json.data(), request.length);
    cJSON* out = nullptr;

    if (!in) {
        out = api_error("invalid_json");
    } else {
        cJSON* opj = cJSON_GetObjectItem(in, "op");
        const char* op = cJSON_IsString(opj) ? opj->valuestring : nullptr;

        if (!op) {
            out = api_error("missing_op");
        } else if (std::strcmp(op, "api") == 0) {
            cJSON* pathj = cJSON_GetObjectItem(in, "path");
            const char* path = cJSON_IsString(pathj) ? pathj->valuestring : nullptr;
            cJSON* body = cJSON_GetObjectItem(in, "body");
            out = api_dispatch(path, body);
        } else if (std::strcmp(op, "ping") == 0) {
            out = api_ok();
            cJSON_AddStringToObject(out, "op", "pong");
            cJSON_AddNumberToObject(out, "protocol", ble_control::kProtocolVersion);
        } else if (std::strcmp(op, "getInfo") == 0) {
            const auto sys = config::system_snapshot();
            out = api_ok();
            cJSON_AddNumberToObject(out, "protocol", ble_control::kProtocolVersion);
            cJSON_AddNumberToObject(out, "schema", rgb::kConfigSchemaVersion);
            cJSON_AddStringToObject(out, "firmware", esp_app_get_description()->version);
            cJSON_AddStringToObject(out, "deviceName", sys.device_name);
        } else if (std::strcmp(op, "getState") == 0) {
            out = api_ok();
            add_state_json(out);
        } else if (std::strcmp(op, "setPower") == 0) {
            cJSON* value = cJSON_GetObjectItem(in, "value");
            if (!cJSON_IsBool(value)) {
                out = api_error("invalid_value");
            } else {
                const esp_err_t err = config::set_master_power(cJSON_IsTrue(value));
                if (err == ESP_OK) {
                    out = api_ok();
                    add_state_json(out);
                } else {
                    out = api_error(esp_err_to_name(err));
                }
            }
        } else if (std::strcmp(op, "setBrightness") == 0) {
            cJSON* value = cJSON_GetObjectItem(in, "value");
            if (!cJSON_IsNumber(value)) {
                out = api_error("invalid_value");
            } else {
                const int v = std::clamp(value->valueint, 0, 255);
                const esp_err_t err = config::set_master_brightness(static_cast<uint8_t>(v));
                if (err == ESP_OK) {
                    out = api_ok();
                    add_state_json(out);
                } else {
                    out = api_error(esp_err_to_name(err));
                }
            }
        } else if (std::strcmp(op, "selectScene") == 0) {
            cJSON* id = cJSON_GetObjectItem(in, "id");
            if (!cJSON_IsNumber(id) || id->valueint < 0 || id->valueint >= static_cast<int>(rgb::kMaxScenes)) {
                out = api_error("invalid_scene");
            } else {
                const esp_err_t err = config::set_active_scene(static_cast<uint8_t>(id->valueint));
                if (err == ESP_OK) {
                    out = api_ok();
                    add_state_json(out);
                } else {
                    out = api_error(esp_err_to_name(err));
                }
            }
        } else if (std::strcmp(op, "save") == 0) {
            const esp_err_t err = config::save();
            out = err == ESP_OK ? api_ok() : api_error(esp_err_to_name(err));
        } else if (std::strcmp(op, "revert") == 0) {
            const esp_err_t err = config::revert();
            out = err == ESP_OK ? api_ok() : api_error(esp_err_to_name(err));
        } else {
            out = api_error("unsupported_op");
        }
    }

    if (in) cJSON_Delete(in);
    if (!out) {
        send_rpc_response(request.message_id, request.generation,
                          "{\"ok\":false,\"error\":\"out_of_memory\"}");
        notify_state();
        return;
    }

    // The majority of configuration writes return exactly {"ok":true}.
    // Never allocate a second heap buffer merely to print that acknowledgment.
    const cJSON* only = out->child;
    if (only && !only->next && only->string &&
        std::strcmp(only->string, "ok") == 0 && cJSON_IsTrue(only)) {
        cJSON_Delete(out);
        send_rpc_response(request.message_id, request.generation, "{\"ok\":true}");
        notify_state();
        return;
    }

    // Avoid cJSON_PrintUnformatted's repeated buffer reallocations for bounded
    // per-scene/core replies. This is a global worker-only buffer, NOT 8 KB
    // on a FreeRTOS task stack. The fallback preserves existing large replies.
    if (cJSON_PrintPreallocated(out, s_response_json.data(),
                                static_cast<int>(s_response_json.size()), false)) {
        cJSON_Delete(out);
        send_rpc_response(request.message_id, request.generation, s_response_json.data());
    } else {
        char* response = cJSON_PrintUnformatted(out);
        cJSON_Delete(out);
        if (response) {
            send_rpc_response(request.message_id, request.generation, response);
            cJSON_free(response);
        } else {
            ESP_LOGE(TAG, "RPC response JSON allocation failed; request length=%u",
                     static_cast<unsigned>(request.length));
            send_rpc_response(request.message_id, request.generation,
                              "{\"ok\":false,\"error\":\"response_encode_failed\"}");
        }
    }
    notify_state();
}

void worker_task(void*)
{
    uint32_t last_revision = 0;
    while (true) {
        RpcRequest request{};
        if (xQueueReceive(s_rpc_queue, &request, pdMS_TO_TICKS(100)) == pdTRUE) {
            handle_rpc(request);
            s_request_json[0] = '\0';
            s_request_pending = false;
        }

        const uint32_t revision = config::revision();
        if (revision != last_revision) {
            last_revision = revision;
            notify_state();
        }

        // Self-heal the common "client vanished / advertising stopped" case.
        // Do not depend on a phone reconnect or a controller reboot to kick
        // advertising back into life.
        if (s_stack_synced && !s_connected && !ble_gap_adv_active()) {
            const int64_t now = esp_timer_get_time();
            if ((now - s_last_adv_attempt_us) >= 1000000) {
                advertise();
            }
        }
    }
}

} // namespace

#endif // CONFIG_BT_NIMBLE_ENABLED

namespace ble_control {

esp_err_t start()
{
#if !CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGW("ble_control", "NimBLE is disabled in sdkconfig; BLE control not started");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_started) return ESP_OK;

    if (!s_rpc_queue) {
        s_rpc_queue = xQueueCreate(1, sizeof(RpcRequest));
        if (!s_rpc_queue) return ESP_ERR_NO_MEM;
    }

    refresh_info();

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(kDeviceName);
    if (rc != 0) {
        ESP_LOGE(TAG, "device name setup failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(kGattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT count failed: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(kGattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT service add failed: %d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = register_cb;

    if (xTaskCreate(host_task, "nimble_host", 8192, nullptr, 5, &s_host_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(worker_task, "ble_control", 10240, nullptr, 4, &s_worker_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Auto-RGB BLE control protocol v%u started", kProtocolVersion);
    return ESP_OK;
#endif
}

bool connected()
{
#if CONFIG_BT_NIMBLE_ENABLED
    return s_connected;
#else
    return false;
#endif
}

} // namespace ble_control
