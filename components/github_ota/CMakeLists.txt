idf_component_register(
    SRCS "github_ota.c"
    INCLUDE_DIRS "."
    REQUIRES
        app_update
        esp_http_client
        esp_https_ota
        esp-tls
        json
        nvs_flash
)
