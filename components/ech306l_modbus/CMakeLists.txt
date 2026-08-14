idf_component_register(
    SRCS "ech306l_modbus.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_common
)