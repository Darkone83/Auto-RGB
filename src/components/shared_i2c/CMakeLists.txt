idf_component_register(
    SRCS "shared_i2c.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board esp_driver_gpio esp_driver_i2c esp_rom freertos
)
