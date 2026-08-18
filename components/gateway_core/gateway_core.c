#include "gateway_core.h"
#include "status_led.h"
#include "esp_log.h"

static const char *TAG = "GW";

static gateway_state_t g_state = GW_STATE_BOOT;

const char *gateway_state_str(gateway_state_t s)
{
    switch (s) {
    case GW_STATE_BOOT: return "BOOT";
    case GW_STATE_NO_INTERNET: return "NO_INTERNET";
    case GW_STATE_ETH_CONNECTING: return "ETH_CONNECTING";
    case GW_STATE_ETH_ONLINE: return "ETH_ONLINE";
    case GW_STATE_WIFI_CONNECTING: return "WIFI_CONNECTING";
    case GW_STATE_WIFI_ONLINE: return "WIFI_ONLINE";
    case GW_STATE_AP_MODE: return "AP_MODE";
    case GW_STATE_MQTT_CONNECTING: return "MQTT_CONNECTING";
    case GW_STATE_MQTT_CONNECTED: return "MQTT_CONNECTED";
    case GW_STATE_SENDING_DATA: return "SENDING_DATA";
    default: return "UNKNOWN";
    }
}

void gateway_core_init(void)
{
    // có thể để trống, hoặc set BOOT ngay
    gateway_set_state(GW_STATE_BOOT);
}

void gateway_set_state(gateway_state_t s)
{
    if (g_state == s) {
        status_led_on_state(s);
        return;
    }
    g_state = s;
    ESP_LOGI(TAG, "STATE -> %s", gateway_state_str(s));
    status_led_on_state(s);
}

gateway_state_t gateway_get_state(void)
{
    return g_state;
}
