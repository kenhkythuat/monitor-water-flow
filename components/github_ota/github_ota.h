#ifndef GITHUB_OTA_H
#define GITHUB_OTA_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void github_ota_mark_app_valid_after_boot(void);
esp_err_t github_ota_start_version_check(void);
void github_ota_update(const char *url);

#ifdef __cplusplus
}
#endif

#endif
