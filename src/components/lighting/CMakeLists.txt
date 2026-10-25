idf_component_register(
    SRCS "lighting.cpp"
    INCLUDE_DIRS "include"
    REQUIRES model config effects led_driver accessories esp_timer freertos
)
