#include "gw_time.h"

#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"

static const char *TAG = "gw_time";

static bool s_sntp_started = false;
static bool s_tz_set = false;

// Threshold epoch ~ 2023-11 (để nhận biết time đã sync)
#define EPOCH_VALID_MIN   (1700000000UL)

void gw_time_set_tz(const char *tz)
{
    if (!tz || tz[0] == '\0') return;
    setenv("TZ", tz, 1);
    tzset();
    s_tz_set = true;
    ESP_LOGI(TAG, "TZ set: %s", tz);
}

void gw_time_init(void)
{
    if (s_sntp_started) return;

    // (tuỳ chọn) set TZ mặc định VN nếu bạn muốn log giờ local:
    // gw_time_set_tz("UTC-7"); // POSIX: UTC-7 nghĩa là GMT+7

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    s_sntp_started = true;
    ESP_LOGI(TAG, "SNTP started");
}

bool gw_time_is_synced(void)
{
    time_t now = 0;
    time(&now);
    return ((uint32_t)now >= EPOCH_VALID_MIN);
}

uint32_t gw_time_unix(void)
{
    time_t now = 0;
    time(&now);

    if ((uint32_t)now >= EPOCH_VALID_MIN) {
        return (uint32_t)now;
    }

    // fallback: uptime seconds (để payload luôn tăng, tránh 0)
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}
