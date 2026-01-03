#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char device_id[32];      // ví dụ "cs_000001"
    uint32_t number_device;  // ví dụ 5
} gateway_config_t;

esp_err_t gateway_config_init(void);

// getters
const char *gateway_config_device_id(void);
uint32_t gateway_config_number_device(void);

// setters (nếu sau này muốn cấu hình qua web/app)
esp_err_t gateway_config_set_device_id(const char *id);
esp_err_t gateway_config_set_number_device(uint32_t n);
esp_err_t gateway_config_get_all(gateway_config_t *out);

#ifdef __cplusplus
}
#endif
