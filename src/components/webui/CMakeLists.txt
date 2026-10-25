idf_component_register(
    SRCS "web_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server json esp_app_format esp_timer config effects lighting model accessories
    EMBED_FILES "index.html" "auto_rgb_logo.webp"
)
