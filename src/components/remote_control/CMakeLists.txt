idf_component_register(
    SRCS "remote_control.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        board
        config
        accessories
        effects
        shared_i2c
        esp_app_format
        esp_driver_gpio
        esp_driver_i2c
        esp_timer
)