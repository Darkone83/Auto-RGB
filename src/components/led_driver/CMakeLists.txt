idf_component_register(
    SRCS "led_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES model board esp_driver_rmt esp_driver_gpio freertos
)