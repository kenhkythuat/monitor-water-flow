idf_component_register(
    SRCS "water_meter_modbus.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer
)
