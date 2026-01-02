idf_component_register(
    SRCS "modbus_pzem.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer esp_event esp_netif
)
