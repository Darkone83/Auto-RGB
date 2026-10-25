idf_component_register(
    SRCS "app_core.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        config
        lighting
        led_driver
        wifi
        webui
        shared_i2c
        accessories
        remote_control
        ble_control
)
