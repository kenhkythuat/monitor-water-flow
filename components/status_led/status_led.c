#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "status_led";

static status_led_cfg_t s_cfg;
static volatile gateway_state_t s_state = GW_STATE_BOOT;

// Dùng để tạo “xung xanh lá 1s” khi gửi data
static volatile uint32_t s_green_req_ms = 0;

// Timing
#define TICK_MS                 100
#define BLUE_BLINK_PERIOD_MS    3000   // “chớp tắt 3S”
#define RED_BLINK_PERIOD_MS     2000   // “chớp đỏ 1S”
#define GREEN_PULSE_MS          1000   


void status_led_pulse_green(uint32_t ms)
{
    if (ms == 0) ms = GREEN_PULSE_MS;
    s_green_req_ms = ms;
}

static inline void led_write(int gpio, bool on)
{
    if (gpio < 0) return;
    int level = s_cfg.active_high ? (on ? 1 : 0) : (on ? 0 : 1);
    gpio_set_level(gpio, level);
}

static inline void leds_all_off(void)
{
    led_write(s_cfg.red_gpio,   false);
    led_write(s_cfg.blue_gpio,  false);
    led_write(s_cfg.green_gpio, false);
}


static bool state_has_internet(gateway_state_t st)
{
    // “có mạng internet bất kì thì sáng luôn” -> dùng LED BLUE sáng liên tục
    switch (st) {
        case GW_STATE_ETH_ONLINE:
        case GW_STATE_WIFI_ONLINE:
        case GW_STATE_MQTT_CONNECTING:
        case GW_STATE_MQTT_CONNECTED:
            return true;
        default:
            return false;
    }
}

static void status_led_task(void *arg)
{
    (void)arg;

    uint32_t t_blue = 0;
    uint32_t t_red  = 0;

    bool blue_blink_level = false;
    bool red_blink_level  = false;

    uint32_t green_pulse_left = 0;

    gateway_state_t last_state = (gateway_state_t)(-1);

    while (1) {
        gateway_state_t st = s_state;

        if (st != last_state) {
            last_state = st;
            t_blue = 0;
            t_red  = 0;
        }
        uint32_t req = s_green_req_ms;
        if (req > 0) {
            s_green_req_ms = 0;
            green_pulse_left = req;
        }

        // ======= MẶC ĐỊNH =======
        bool red_on   = true;   // ✅ RED luôn sáng
        bool blue_on  = false;
        bool green_on = false;

        // GREEN pulse 1s
        if (green_pulse_left > 0) {
            green_on = true;
            if (green_pulse_left > TICK_MS) green_pulse_left -= TICK_MS;
            else green_pulse_left = 0;
        }

        // ======= RED chỉ chớp khi AP_MODE (config) =======
        if (st == GW_STATE_AP_MODE) {
            t_red += TICK_MS;
            if (t_red >= RED_BLINK_PERIOD_MS) {
                t_red = 0;
                red_blink_level = !red_blink_level;
            }
            red_on = red_blink_level;
        }

        // ======= BLUE theo internet =======
        if (st == GW_STATE_NO_INTERNET || st == GW_STATE_ETH_CONNECTING || st == GW_STATE_WIFI_CONNECTING) {
            // chớp 3s khi chưa online / đang connect
            t_blue += TICK_MS;
            if (t_blue >= BLUE_BLINK_PERIOD_MS) {
                t_blue = 0;
                blue_blink_level = !blue_blink_level;
            }
            blue_on = blue_blink_level;
        } else {
            // có mạng bất kỳ -> sáng luôn
            if (state_has_internet(st) || st == GW_STATE_ETH_ONLINE || st == GW_STATE_WIFI_ONLINE || st == GW_STATE_SENDING_DATA) {
                blue_on = true;
            }
        }

        // SENDING_DATA vẫn online => blue ON
        if (st == GW_STATE_SENDING_DATA) {
            blue_on = true;
        }

        // ======= SET GPIO =======
        led_write(s_cfg.red_gpio,   red_on);
        led_write(s_cfg.blue_gpio,  blue_on);
        led_write(s_cfg.green_gpio, green_on);

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}



esp_err_t status_led_init(const status_led_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;

    gpio_config_t io = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (s_cfg.red_gpio >= 0)   io.pin_bit_mask |= (1ULL << s_cfg.red_gpio);
    if (s_cfg.blue_gpio >= 0)  io.pin_bit_mask |= (1ULL << s_cfg.blue_gpio);
    if (s_cfg.green_gpio >= 0) io.pin_bit_mask |= (1ULL << s_cfg.green_gpio);

    ESP_ERROR_CHECK(gpio_config(&io));
    leds_all_off();

    // Boot state mặc định
    s_state = GW_STATE_BOOT;

    BaseType_t ok = xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 4, NULL);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "status_led init ok (R=%d B=%d G=%d)",
             s_cfg.red_gpio, s_cfg.blue_gpio, s_cfg.green_gpio);
    return ESP_OK;
}

void status_led_on_state(gateway_state_t state)
{
    s_state = state;
}




gateway_state_t status_led_get_state(void)
{
    return s_state;
}
