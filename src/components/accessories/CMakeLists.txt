idf_component_register(
    SRCS "accessories.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        board
        config
        model
        shared_i2c
        rtc_ds3231m
        led_driver
        esp_driver_gpio
        esp_driver_i2c
        esp_timer
        freertos
)
