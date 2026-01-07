#include "gw_temp.h"

#include "esp_log.h"
#include "esp_check.h"

#if __has_include("soc/soc_caps.h")
#include "soc/soc_caps.h"
#endif

#if __has_include("driver/temperature_sensor.h")
#include "driver/temperature_sensor.h"
#endif

static const char *TAG = "gw_temp";

#if defined(SOC_TEMP_SENSOR_SUPPORTED) && SOC_TEMP_SENSOR_SUPPORTED

static temperature_sensor_handle_t s_tsens = NULL;

esp_err_t gw_temp_init(void)
{
    if (s_tsens) return ESP_OK;

    // Chọn khoảng đo hợp lý để giảm sai số; ví dụ 20~80°C
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 80);

    ESP_RETURN_ON_ERROR(temperature_sensor_install(&cfg, &s_tsens), TAG, "install failed");
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(s_tsens), TAG, "enable failed");

    ESP_LOGI(TAG, "chip temperature sensor ready");
    return ESP_OK;
}

esp_err_t gw_temp_read_celsius(float *out_c)
{
    if (!out_c) return ESP_ERR_INVALID_ARG;
    if (!s_tsens) return ESP_ERR_INVALID_STATE;

    // Driver không tự thread-safe, nếu nhiều task gọi thì bạn tự bọc mutex bên ngoài. :contentReference[oaicite:1]{index=1}
    return temperature_sensor_get_celsius(s_tsens, out_c);
}

esp_err_t gw_temp_deinit(void)
{
    if (!s_tsens) return ESP_OK;

    esp_err_t err = ESP_OK;
    err |= temperature_sensor_disable(s_tsens);
    err |= temperature_sensor_uninstall(s_tsens);
    s_tsens = NULL;

    if (err == ESP_OK) ESP_LOGI(TAG, "chip temperature sensor deinit");
    return err;
}

#else

esp_err_t gw_temp_init(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t gw_temp_read_celsius(float *out_c)
{
    (void)out_c;
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t gw_temp_deinit(void) { return ESP_ERR_NOT_SUPPORTED; }

#endif
