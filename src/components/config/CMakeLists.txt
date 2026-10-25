idf_component_register(
    SRCS "config_store.cpp"
    INCLUDE_DIRS "include"
    REQUIRES model nvs_flash freertos
)
