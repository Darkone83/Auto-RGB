idf_component_register(
    SRCS "ble_control.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        config
        bt
        json
        esp_app_format
        accessories
        effects
        lighting
        esp_timer
)
