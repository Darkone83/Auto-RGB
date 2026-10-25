idf_component_register(
    SRCS "rtc_ds3231m.cpp"
    INCLUDE_DIRS "include"
    REQUIRES shared_i2c esp_driver_i2c
)
