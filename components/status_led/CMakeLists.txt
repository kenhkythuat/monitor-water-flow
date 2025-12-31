idf_component_register(
    SRCS "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos gateway_core
)
