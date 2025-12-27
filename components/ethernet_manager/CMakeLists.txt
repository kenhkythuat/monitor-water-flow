idf_component_register(
    SRCS "ethernet_manager.c"
    INCLUDE_DIRS "."
    REQUIRES esp_netif esp_event esp_eth ethernet_init
)
