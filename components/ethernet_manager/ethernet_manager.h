#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ethernet_manager_start(void);


bool ethernet_manager_link_up(void);
bool ethernet_manager_has_ip(void);
esp_err_t ethernet_manager_get_ip(esp_netif_ip_info_t *out);

#ifdef __cplusplus
}
#endif
