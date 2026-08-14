#include "gateway_config.h"
#include <string.h>
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "gw_cfg";

// Ưu tiên namespace mới theo nvs.csv của bạn, fallback namespace cũ
static const char *NVS_NS_PRIMARY  = "nodeconfig";
static const char *NVS_NS_FALLBACK = "gwcfg";

static gateway_config_t s_cfg = {
    .device_id = "gw_000001",
    .number_device = 1,

    .mqtt_uri = "mqtt://161.248.146.170:1883",
    .mqtt_user = "thuanphat",
    .mqtt_pass = "",
    .mqtt_client_id = "apm_000001",
};

static esp_err_t nvs_read_str(nvs_handle_t h, const char *key, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return ESP_ERR_INVALID_ARG;
    size_t len = out_sz;
    esp_err_t e = nvs_get_str(h, key, out, &len);
    return e;
}

static void strlcpy_safe(char *dst, const char *src, size_t dst_sz)
{
    if (!dst || dst_sz == 0) return;
    if (!src) { dst[0] = 0; return; }
    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = 0;
}

static esp_err_t open_any_namespace(nvs_handle_t *out_h, const char **out_ns_used)
{
    if (!out_h) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS_PRIMARY, NVS_READWRITE, &h);
    if (e == ESP_OK) {
        if (out_ns_used) *out_ns_used = NVS_NS_PRIMARY;
        *out_h = h;
        return ESP_OK;
    }

    e = nvs_open(NVS_NS_FALLBACK, NVS_READWRITE, &h);
    if (e == ESP_OK) {
        if (out_ns_used) *out_ns_used = NVS_NS_FALLBACK;
        *out_h = h;
        return ESP_OK;
    }

    return e;
}

// helper: đọc nếu có, nếu không có thì ghi default vào NVS
static void load_or_write_str(nvs_handle_t h, const char *key, char *dst, size_t dst_sz, const char *def)
{
    char tmp[128] = {0};
    esp_err_t e = nvs_read_str(h, key, tmp, sizeof(tmp));
    if (e == ESP_OK && tmp[0] != '\0') {
        strlcpy_safe(dst, tmp, dst_sz);
    } else {
        strlcpy_safe(dst, def, dst_sz);
        nvs_set_str(h, key, dst);
        ESP_LOGW(TAG, "%s missing -> write default", key);
    }
}

static void load_or_write_u32(nvs_handle_t h, const char *key, uint32_t *dst, uint32_t def)
{
    uint32_t v = 0;
    esp_err_t e = nvs_get_u32(h, key, &v);
    if (e == ESP_OK && v > 0) {
        *dst = v;
    } else {
        *dst = def;
        nvs_set_u32(h, key, *dst);
        ESP_LOGW(TAG, "%s missing -> write default=%lu", key, (unsigned long)*dst);
    }
}

esp_err_t gateway_config_init(void)
{
    nvs_handle_t h;
    const char *ns_used = NULL;

    esp_err_t e = open_any_namespace(&h, &ns_used);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(e));
        return e;
    }

    // load config (nếu thiếu thì ghi default vào NVS)
    load_or_write_str(h, "device_id",      s_cfg.device_id,      sizeof(s_cfg.device_id),      s_cfg.device_id);
    load_or_write_u32(h, "number_device",  &s_cfg.number_device, s_cfg.number_device);

    load_or_write_str(h, "mqtt_uri",       s_cfg.mqtt_uri,       sizeof(s_cfg.mqtt_uri),       s_cfg.mqtt_uri);
    load_or_write_str(h, "mqtt_user",      s_cfg.mqtt_user,      sizeof(s_cfg.mqtt_user),      s_cfg.mqtt_user);
    load_or_write_str(h, "mqtt_pass",      s_cfg.mqtt_pass,      sizeof(s_cfg.mqtt_pass),      s_cfg.mqtt_pass);
    load_or_write_str(h, "mqtt_client_id", s_cfg.mqtt_client_id, sizeof(s_cfg.mqtt_client_id), s_cfg.mqtt_client_id);

    nvs_commit(h);
    nvs_close(h);

    // log (KHÔNG log pass)
    ESP_LOGI(TAG, "Loaded(ns=%s): device_id=%s number_device=%lu mqtt_uri=%s mqtt_user=%s mqtt_client_id=%s",
             ns_used ? ns_used : "?",
             s_cfg.device_id,
             (unsigned long)s_cfg.number_device,
             s_cfg.mqtt_uri,
             s_cfg.mqtt_user,
             s_cfg.mqtt_client_id);

    return ESP_OK;
}

/* getters */
const char *gateway_config_device_id(void)      { return s_cfg.device_id; }
uint32_t    gateway_config_number_device(void)  { return s_cfg.number_device; }

const char *gateway_config_mqtt_uri(void)       { return s_cfg.mqtt_uri; }
const char *gateway_config_mqtt_user(void)      { return s_cfg.mqtt_user; }
const char *gateway_config_mqtt_pass(void)      { return s_cfg.mqtt_pass; }
const char *gateway_config_mqtt_client_id(void) { return s_cfg.mqtt_client_id; }

/* setters */
static esp_err_t set_str_key(const char *key, const char *val, char *dst, size_t dst_sz)
{
    if (!val || val[0] == '\0') return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    const char *ns_used = NULL;
    esp_err_t e = open_any_namespace(&h, &ns_used);
    if (e != ESP_OK) return e;

    strlcpy_safe(dst, val, dst_sz);
    e = nvs_set_str(h, key, dst);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t set_u32_key(const char *key, uint32_t val, uint32_t *dst)
{
    if (val == 0) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    const char *ns_used = NULL;
    esp_err_t e = open_any_namespace(&h, &ns_used);
    if (e != ESP_OK) return e;

    *dst = val;
    e = nvs_set_u32(h, key, val);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t gateway_config_set_device_id(const char *id)      { return set_str_key("device_id", id, s_cfg.device_id, sizeof(s_cfg.device_id)); }
esp_err_t gateway_config_set_number_device(uint32_t n)      { return set_u32_key("number_device", n, &s_cfg.number_device); }

esp_err_t gateway_config_set_mqtt_uri(const char *uri)      { return set_str_key("mqtt_uri", uri, s_cfg.mqtt_uri, sizeof(s_cfg.mqtt_uri)); }
esp_err_t gateway_config_set_mqtt_user(const char *user)    { return set_str_key("mqtt_user", user, s_cfg.mqtt_user, sizeof(s_cfg.mqtt_user)); }
esp_err_t gateway_config_set_mqtt_pass(const char *pass)    { return set_str_key("mqtt_pass", pass, s_cfg.mqtt_pass, sizeof(s_cfg.mqtt_pass)); }
esp_err_t gateway_config_set_mqtt_client_id(const char *cid){ return set_str_key("mqtt_client_id", cid, s_cfg.mqtt_client_id, sizeof(s_cfg.mqtt_client_id)); }

esp_err_t gateway_config_get_all(gateway_config_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    *out = s_cfg;
    return ESP_OK;
}
