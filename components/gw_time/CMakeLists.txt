idf_component_register(
    SRCS "gw_time.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer
    PRIV_REQUIRES esp_netif
)
