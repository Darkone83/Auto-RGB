#include "wifi_ap.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "config_store.hpp"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"

namespace {
constexpr const char* TAG = "wifi_ap";
constexpr uint16_t kDnsPort = 53;
constexpr std::size_t kDnsPacketMax = 512;

esp_netif_t* s_ap_netif = nullptr;
TaskHandle_t s_dns_task = nullptr;
uint32_t s_ap_ip_addr = 0;

bool find_dns_question_end(const uint8_t* packet, std::size_t length, std::size_t& question_end)
{
    if (!packet || length < 17) return false; // header + at least root/type/class

    std::size_t pos = 12;
    while (pos < length) {
        const uint8_t label_len = packet[pos++];
        if (label_len == 0) break;
        if ((label_len & 0xC0) != 0 || pos + label_len > length) return false;
        pos += label_len;
    }

    if (pos + 4 > length) return false;
    question_end = pos + 4;
    return true;
}

void dns_server_task(void*)
{
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Captive DNS socket creation failed");
        s_dns_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(kDnsPort);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        ESP_LOGE(TAG, "Captive DNS bind failed");
        close(sock);
        s_dns_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS wildcard active on UDP/53");

    std::array<uint8_t, kDnsPacketMax> packet{};
    while (true) {
        sockaddr_in source{};
        socklen_t source_len = sizeof(source);
        const int received = recvfrom(sock, packet.data(), packet.size(), 0,
                                      reinterpret_cast<sockaddr*>(&source), &source_len);
        if (received <= 0) continue;

        const std::size_t length = static_cast<std::size_t>(received);
        if (length < 12) continue;

        // Only answer standard queries with at least one question.
        const uint16_t qdcount = static_cast<uint16_t>((packet[4] << 8) | packet[5]);
        if (qdcount != 1 || (packet[2] & 0x80) != 0) continue;

        std::size_t question_end = 0;
        if (!find_dns_question_end(packet.data(), length, question_end)) continue;

        // Keep this at INFO while captive behavior is being validated. If a
        // phone joins and this never appears, the client did not issue a DNS
        // captive/connectivity probe for this association.
        ESP_LOGI(TAG, "Captive DNS query received (%u bytes)", static_cast<unsigned>(length));

        const std::size_t type_offset = question_end - 4;
        const uint16_t qtype = static_cast<uint16_t>((packet[type_offset] << 8) | packet[type_offset + 1]);
        const uint16_t qclass = static_cast<uint16_t>((packet[type_offset + 2] << 8) | packet[type_offset + 3]);
        const bool answer_ipv4 = qclass == 1 && (qtype == 1 || qtype == 255); // A or ANY

        // Response, authoritative, preserving the client's recursion-desired bit.
        packet[2] = static_cast<uint8_t>(0x84 | (packet[2] & 0x01));
        packet[3] = 0x00;
        packet[6] = 0x00;
        packet[7] = answer_ipv4 ? 0x01 : 0x00; // ANCOUNT
        packet[8] = packet[9] = packet[10] = packet[11] = 0x00;

        std::size_t response_len = question_end;
        if (answer_ipv4 && response_len + 16 <= packet.size()) {
            uint8_t* a = packet.data() + response_len;
            a[0] = 0xC0; a[1] = 0x0C; // compressed name -> first question
            a[2] = 0x00; a[3] = 0x01; // TYPE A
            a[4] = 0x00; a[5] = 0x01; // CLASS IN
            a[6] = a[7] = a[8] = a[9] = 0x00; // TTL 0: do not retain captive DNS after disconnect
            a[10] = 0x00; a[11] = 0x04;
            std::memcpy(a + 12, &s_ap_ip_addr, 4);
            response_len += 16;
        }

        sendto(sock, packet.data(), response_len, 0,
               reinterpret_cast<sockaddr*>(&source), source_len);
    }
}

esp_err_t start_captive_dns()
{
    if (s_dns_task) return ESP_OK;
    if (xTaskCreate(dns_server_task, "captive_dns", 4096, nullptr, 4, &s_dns_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void capture_ap_ip(esp_netif_t* ap_netif)
{
    if (!ap_netif) return;

    esp_netif_ip_info_t ip_info{};
    if (esp_netif_get_ip_info(ap_netif, &ip_info) != ESP_OK) return;
    s_ap_ip_addr = ip_info.ip.addr;

    char ip_addr[16]{};
    inet_ntoa_r(ip_info.ip.addr, ip_addr, sizeof(ip_addr));
    ESP_LOGI(TAG, "Captive portal AP IP: %s", ip_addr);
}

// Explicitly advertise the SoftAP itself as the DNS server.  The captive
// portal depends on clients sending their connectivity-check DNS queries to
// our wildcard DNS responder; do not leave that to DHCP client heuristics.
esp_err_t configure_dhcp_captive(esp_netif_t* ap_netif)
{
    if (!ap_netif || s_ap_ip_addr == 0) return ESP_ERR_INVALID_STATE;

    const esp_err_t stop_err = esp_netif_dhcps_stop(ap_netif);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "DHCP stop before captive DNS configuration: %s", esp_err_to_name(stop_err));
    }

    // Match the proven XBOX-RGB model: the AP itself is the DNS server and
    // wildcard DNS resolves every hostname back to the controller.
    uint8_t offer_dns = 0x02; // DHCP server OFFER_DNS bit in IDF 5.x
    ESP_RETURN_ON_ERROR(
        esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET,
                               ESP_NETIF_DOMAIN_NAME_SERVER,
                               &offer_dns, sizeof(offer_dns)),
        TAG, "DHCP DNS offer configuration failed");

    esp_netif_dns_info_t dns{};
    dns.ip.type = IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = s_ap_ip_addr;
    ESP_RETURN_ON_ERROR(
        esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns),
        TAG, "SoftAP DNS address configuration failed");

    const esp_err_t start_err = esp_netif_dhcps_start(ap_netif);
    if (start_err != ESP_OK && start_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_RETURN_ON_ERROR(start_err, TAG, "DHCP restart after DNS configuration failed");
    }

    char ip_addr[16]{};
    inet_ntoa_r(s_ap_ip_addr, ip_addr, sizeof(ip_addr));
    ESP_LOGI(TAG, "DHCP advertises captive DNS server: %s", ip_addr);
    return ESP_OK;
}

void wifi_event_handler(void*, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        const auto* ev = static_cast<wifi_event_ap_staconnected_t*>(event_data);
        ESP_LOGI(TAG, "Client joined AP, AID=%u", static_cast<unsigned>(ev ? ev->aid : 0));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const auto* ev = static_cast<wifi_event_ap_stadisconnected_t*>(event_data);
        ESP_LOGI(TAG, "Client left AP, AID=%u", static_cast<unsigned>(ev ? ev->aid : 0));
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ESP_LOGI(TAG, "DHCP lease assigned; waiting for captive DNS/HTTP probe");
    }
}


} // namespace

namespace wifi_ap {

esp_err_t start()
{
    const auto system = config::system_snapshot();

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (!s_ap_netif) return ESP_FAIL;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init failed");

    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr),
        TAG, "Wi-Fi event handler registration failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_event_handler, nullptr),
        TAG, "IP event handler registration failed");

    wifi_config_t ap{};
    std::strncpy(reinterpret_cast<char*>(ap.ap.ssid), system.ap_ssid, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len = static_cast<uint8_t>(std::strlen(system.ap_ssid));
    ap.ap.password[0] = '\0';
    ap.ap.channel = 6;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "set AP+STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap), TAG, "set AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    capture_ap_ip(s_ap_netif);
    ESP_RETURN_ON_ERROR(configure_dhcp_captive(s_ap_netif), TAG, "captive DHCP/DNS setup failed");
    ESP_RETURN_ON_ERROR(start_captive_dns(), TAG, "captive DNS start failed");

    ESP_LOGI(TAG, "Open SoftAP started: %s", system.ap_ssid);
    ESP_LOGI(TAG, "AP+STA mode active for captive compatibility; no external STA connection is configured");
    ESP_LOGI(TAG, "Captive mode: XBOX-RGB parity (open AP + wildcard DNS + HTTP 200 portal shim)");
    return ESP_OK;
}

} // namespace wifi_ap
