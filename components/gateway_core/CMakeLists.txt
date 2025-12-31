idf_component_register(
    SRCS "gateway_core.c"
    INCLUDE_DIRS "."
    REQUIRES freertos status_led
)
