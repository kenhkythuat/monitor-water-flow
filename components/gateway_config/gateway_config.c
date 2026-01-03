#include "gateway_config.h"
#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "gw_cfg";
static const char *NVS_NS = "gwcfg";

static gateway_config_t s_cfg = {
    .device_id = "cs_000001",
    .number_device = 5,
};

static esp_err_t nvs_read_str(nvs_handle_t h, const char *key, char *out, size_t out_sz)
{
    size_t len = out_sz;
    esp_err_t e = nvs_get_str(h, key, out, &len);
    if (e == ESP_OK)
        return ESP_OK;
    return e;
}

esp_err_t gateway_config_init(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;

    // device_id
    char id[32] = {0};
    e = nvs_read_str(h, "device_id", id, sizeof(id));
    if (e == ESP_OK && id[0] != '\0')
    {
        strncpy(s_cfg.device_id, id, sizeof(s_cfg.device_id) - 1);
    }
    else
    {
        // nếu chưa có trong NVS -> ghi default để ổn định
        ESP_LOGW(TAG, "device_id missing -> write default=%s", s_cfg.device_id);
        nvs_set_str(h, "device_id", s_cfg.device_id);
    }

    // number_device
    uint32_t n = 0;
    e = nvs_get_u32(h, "number_device", &n);
    if (e == ESP_OK && n > 0)
    {
        s_cfg.number_device = n;
    }
    else
    {
        ESP_LOGW(TAG, "number_device missing -> write default=%lu", (unsigned long)s_cfg.number_device);
        nvs_set_u32(h, "number_device", s_cfg.number_device);
    }

    nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "Loaded: device_id=%s number_device=%lu",
             s_cfg.device_id, (unsigned long)s_cfg.number_device);
    printf("Loaded: device_id=%s number_device=%lu",
           s_cfg.device_id, (unsigned long)s_cfg.number_device);
    return ESP_OK;
}

const char *gateway_config_device_id(void) { return s_cfg.device_id; }
uint32_t gateway_config_number_device(void) { return s_cfg.number_device; }

esp_err_t gateway_config_set_device_id(const char *id)
{
    if (!id || id[0] == '\0')
        return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;

    strncpy(s_cfg.device_id, id, sizeof(s_cfg.device_id) - 1);
    s_cfg.device_id[sizeof(s_cfg.device_id) - 1] = 0;

    e = nvs_set_str(h, "device_id", s_cfg.device_id);
    if (e == ESP_OK)
        e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t gateway_config_set_number_device(uint32_t n)
{
    if (n == 0)
        return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;

    s_cfg.number_device = n;
    e = nvs_set_u32(h, "number_device", n);
    if (e == ESP_OK)
        e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t gateway_config_get_all(gateway_config_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;
    *out = s_cfg;
    return ESP_OK;
}
