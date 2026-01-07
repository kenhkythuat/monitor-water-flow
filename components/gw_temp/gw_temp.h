#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t gw_temp_init(void);
esp_err_t gw_temp_read_celsius(float *out_c);
esp_err_t gw_temp_deinit(void);

#ifdef __cplusplus
}
#endif
