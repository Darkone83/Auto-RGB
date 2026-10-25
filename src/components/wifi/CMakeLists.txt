idf_component_register(
    SRCS "wifi_ap.cpp"
    INCLUDE_DIRS "include"
    REQUIRES config esp_wifi esp_netif esp_event lwip freertos
)