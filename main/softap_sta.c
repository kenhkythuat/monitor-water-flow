// main.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include <sys/param.h>
#include "cJSON.h"
#include "mqtt_client.h"
#include "ethernet_manager.h"

#define WIFI_RESET_BUTTON_GPIO 41
#define WIFI_RESET_HOLD_TIME_MS 10000
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define LED_RED_GPIO 13
#define LED_BLUE_GPIO 14
#define LED_GREEN_GPIO 21

static const char *TAG = "wifi_setup";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static int pending_ack_id = -1;
static bool ack_flag = false;
static char ack_id[64] = {0};

static esp_mqtt_client_handle_t mqtt_client = NULL; // client toàn cục
static bool mqtt_task_started = false;              // để tránh tạo nhiều lần

// forward
void wifi_init_ap(void);
void wifi_init_sta(const char *ssid, const char *pass);
void wifi_reset_to_ap_mode(void);
httpd_handle_t start_webserver(void);

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data);

// new
static EventGroupHandle_t s_net_event_group;
#define NET_WIFI_OK_BIT BIT0
#define NET_ETH_OK_BIT BIT1

typedef enum
{
    GW_STATE_BOOT = 0,
    GW_STATE_NO_INTERNET,

    GW_STATE_ETH_CONNECTING,
    GW_STATE_ETH_ONLINE,

    GW_STATE_WIFI_CONNECTING,
    GW_STATE_WIFI_ONLINE,
    GW_STATE_AP_MODE,

    GW_STATE_MQTT_CONNECTING,
    GW_STATE_MQTT_CONNECTED,

    GW_STATE_SENDING_DATA,
} gateway_state_t;

static gateway_state_t g_state = GW_STATE_BOOT;

static const char *gw_state_str(gateway_state_t s)
{
    switch (s)
    {
    case GW_STATE_BOOT:
        return "BOOT";
    case GW_STATE_NO_INTERNET:
        return "NO_INTERNET";
    case GW_STATE_ETH_CONNECTING:
        return "ETH_CONNECTING";
    case GW_STATE_ETH_ONLINE:
        return "ETH_ONLINE";
    case GW_STATE_WIFI_CONNECTING:
        return "WIFI_CONNECTING";
    case GW_STATE_WIFI_ONLINE:
        return "WIFI_ONLINE";
    case GW_STATE_AP_MODE:
        return "AP_MODE";
    case GW_STATE_MQTT_CONNECTING:
        return "MQTT_CONNECTING";
    case GW_STATE_MQTT_CONNECTED:
        return "MQTT_CONNECTED";
    case GW_STATE_SENDING_DATA:
        return "SENDING_DATA";
    default:
        return "UNKNOWN";
    }
}

static void gateway_set_state(gateway_state_t s)
{
    if (g_state == s)
        return;
    g_state = s;
    ESP_LOGI("GW", "STATE -> %s", gw_state_str(s));
}

static void mqtt_manager_start(void)
{
    if (mqtt_client)
        return;

    gateway_set_state(GW_STATE_MQTT_CONNECTING);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://72.61.140.234:1883",
        .credentials.username = "thuanphat",
        .credentials.authentication.password = "123456789",
        .credentials.client_id = "phat123",
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client)
    {
        ESP_LOGE("MQTT", "Failed to init client");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK)
    {
        ESP_LOGE("MQTT", "Failed to start: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return;
    }
}

static void mqtt_manager_stop(void)
{
    if (!mqtt_client)
        return;
    esp_mqtt_client_stop(mqtt_client);
    esp_mqtt_client_destroy(mqtt_client);
    mqtt_client = NULL;
    ESP_LOGW("MQTT", "MQTT stopped (no internet)");
}

static void net_monitor_task(void *arg)
{
    bool last_eth_ip = false;

    while (1)
    {
        bool eth_ip = ethernet_manager_has_ip();
        if (eth_ip != last_eth_ip)
        {
            last_eth_ip = eth_ip;
            if (eth_ip)
                xEventGroupSetBits(s_net_event_group, NET_ETH_OK_BIT);
            else
                xEventGroupClearBits(s_net_event_group, NET_ETH_OK_BIT);
        }

        EventBits_t b = xEventGroupGetBits(s_net_event_group);
        bool any_net = (b & (NET_WIFI_OK_BIT | NET_ETH_OK_BIT));

        if (!any_net)
        {
            gateway_set_state(GW_STATE_NO_INTERNET);
            gpio_set_level(LED_RED_GPIO, 1);
            gpio_set_level(LED_BLUE_GPIO, 0);
            mqtt_manager_stop();
        }
        else
        {
            gpio_set_level(LED_RED_GPIO, 0);
            gpio_set_level(LED_BLUE_GPIO, 1);

            if (eth_ip)
                gateway_set_state(GW_STATE_ETH_ONLINE);
            else
                gateway_set_state(GW_STATE_WIFI_ONLINE);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// off new

/* ---------------- HTTP server handlers ---------------- */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html =
        "<!doctype html>\n"
        "<html>\n"
        "<head>\n"
        "  <meta charset='utf-8'>\n"
        "  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
        "  <title>ESP32 WiFi Setup</title>\n"
        "  <style>\n"
        "    body{font-family: Arial, Helvetica, sans-serif; padding:20px}\n"
        "    .card{max-width:420px;margin:auto;padding:16px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,0.1)}\n"
        "    input{width:100%;padding:8px;margin:8px 0}\n"
        "    button{padding:10px 18px}\n"
        "    #status{margin-top:12px}\n"
        "  </style>\n"
        "</head>\n"
        "<body>\n"
        "  <div class='card'>\n"
        "    <h3>ESP32 Wi-Fi Setup</h3>\n"
        "    <p>Nhập SSID và mật khẩu mạng Wi-Fi bạn muốn ESP32 kết nối.</p>\n"
        "    <label>SSID</label>\n"
        "    <input id='ssid' placeholder='Your WiFi SSID'>\n"
        "    <label>Password</label>\n"
        "    <input id='pass' type='password' placeholder='Your WiFi password (can be empty)'>\n"
        "    <button id='btn'>Connect</button>\n"
        "    <div id='status'></div>\n"
        "  </div>\n"
        "  <script>\n"
        "    const btn = document.getElementById('btn');\n"
        "    const status = document.getElementById('status');\n"
        "    btn.onclick = async () => {\n"
        "      const ssid = document.getElementById('ssid').value;\n"
        "      const password = document.getElementById('pass').value;\n"
        "      if(!ssid){ status.innerText='Vui lòng nhập SSID'; return; }\n"
        "      status.innerText = 'Sending...';\n"
        "      try{\n"
        "        const res = await fetch('/connect', {\n"
        "          method:'POST',\n"
        "          headers: {'Content-Type':'application/json'},\n"
        "          body: JSON.stringify({ssid,password})\n"
        "        });\n"
        "        const j = await res.json();\n"
        "        if(j.success){\n"
        "          status.innerHTML = 'Đã kết nối thành công tới ' + j.ip + '. ESP sẽ rời AP.';\n"
        "        } else {\n"
        "          status.innerText = 'Kết nối thất bại: ' + (j.error || 'unknown');\n"
        "        }\n"
        "      } catch(e){ status.innerText = 'Lỗi mạng: ' + e; }\n"
        "    }\n"
        "  </script>\n"
        "</body>\n"
        "</html>\n";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t connect_post_handler(httpd_req_t *req)
{
    // read body safely
    const int BUF_SIZE = 512;
    char *buf = (char *)malloc(BUF_SIZE);
    if (!buf)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "alloc fail");
        return ESP_FAIL;
    }
    memset(buf, 0, BUF_SIZE);

    int ret = 0;
    int received = 0;
    int remaining = req->content_len;
    while (remaining > 0 && received < BUF_SIZE - 1)
    {
        ret = httpd_req_recv(req, buf + received, MIN(remaining, BUF_SIZE - 1 - received));
        if (ret <= 0)
        {
            free(buf);
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv fail");
            return ESP_FAIL;
        }
        received += ret;
        remaining -= ret;
    }
    buf[received] = '\0';

    // parse json
    cJSON *json = cJSON_Parse(buf);
    free(buf);
    if (!json)
    {
        ESP_LOGE(TAG, "Invalid JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    const cJSON *ssid_json = cJSON_GetObjectItemCaseSensitive(json, "ssid");
    const cJSON *pass_json = cJSON_GetObjectItemCaseSensitive(json, "password");
    // ESP_LOGI(TAG, "Connecting to SSID='%s' PASSWORD='%s'", ssid_json, pass_json);

    if (!cJSON_IsString(ssid_json) || (ssid_json->valuestring == NULL) || strlen(ssid_json->valuestring) == 0)
    {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID missing");
        return ESP_FAIL;
    }

    char ssid[64] = {0};
    char pass[128] = {0};
    strncpy(ssid, ssid_json->valuestring, sizeof(ssid) - 1);
    if (cJSON_IsString(pass_json) && pass_json->valuestring)
    {
        strncpy(pass, pass_json->valuestring, sizeof(pass) - 1);
    }

    // do NOT log password in plain text
    ESP_LOGI(TAG, "Received connecting request: ssid=%s pass_len=%d", ssid, (int)strlen(pass));

    // save to NVS
    nvs_handle_t nvs;
    if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_str(nvs, "wifi_ssid", ssid);
        nvs_set_str(nvs, "wifi_pass", pass);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    else
    {
        ESP_LOGW(TAG, "Cannot open NVS to save creds");
    }

    // send JSON response
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "success", true);
    cJSON_AddStringToObject(resp, "ip", "192.168.4.1"); // in AP mode
    char *resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    free(resp_str);
    cJSON_Delete(resp);

    cJSON_Delete(json);

    // small delay then restart (so NVS commit flush)
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();

    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL};
        httpd_uri_t connect_uri = {
            .uri = "/connect",
            .method = HTTP_POST,
            .handler = connect_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &connect_uri);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to start HTTP server");
    }
    return server;
}
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id)
    {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "Connected to ThingsBoard");
        gateway_set_state(GW_STATE_MQTT_CONNECTED);

        // Thêm subscribe cho topic command
        esp_mqtt_client_subscribe(event->client,
                                  "tbmq/cs_000001/port01/command", 0);
        ESP_LOGI("MQTT", "Subscribed to tbmq/cs_000001/port01/command");

        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "Subscribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI("MQTT", "Before connect");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "Published OK, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
    {
        char topic_buf[128] = {0};
        snprintf(topic_buf, sizeof(topic_buf),
                 "%.*s", event->topic_len, event->topic);

        char data_buf[256] = {0};
        snprintf(data_buf, sizeof(data_buf),
                 "%.*s", event->data_len, event->data);

        ESP_LOGI("MQTT", "Received topic: %s", topic_buf);
        ESP_LOGI("MQTT", "Received data : %s", data_buf);

        /* =====================================================
           XỬ LÝ TOPIC COMMAND → TẠO NHIỆM VỤ GỬI ACK
           ===================================================== */
        if (strcmp(topic_buf, "tbmq/cs_000001/port01/command") == 0)
        {
            cJSON *json = cJSON_Parse(data_buf);
            if (json)
            {
                cJSON *id = cJSON_GetObjectItem(json, "id");

                if (cJSON_IsString(id) && id->valuestring != NULL)
                {
                    // Copy ID vào biến global
                    strncpy(ack_id, id->valuestring, sizeof(ack_id) - 1);

                    ack_flag = true;

                    ESP_LOGI("MQTT", "Parsed CMD id=%s -> ACK queued", ack_id);
                }
                else
                {
                    ESP_LOGW("MQTT", "No valid \"id\" in command!");
                }

                cJSON_Delete(json);
            }
            else
            {
                ESP_LOGE("MQTT", "Invalid JSON payload!");
            }
        }

        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW("MQTT", "Disconnected");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE("MQTT", "Error");
        break;

    default:
        ESP_LOGW("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

/* ---------------- MQTT Task ---------------- */
static void mqtt_publish_task(void *pvParameters)
{
    while (1)
    {
        // Chờ có ít nhất 1 mạng OK (WiFi hoặc ETH)
        xEventGroupWaitBits(s_net_event_group,
                            NET_WIFI_OK_BIT | NET_ETH_OK_BIT,
                            pdFALSE, pdFALSE,
                            portMAX_DELAY);

        // Có mạng -> đảm bảo MQTT chạy
        mqtt_manager_start();

        // Nếu MQTT chưa kịp tạo, chờ chút
        if (!mqtt_client)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Publish
        gateway_set_state(GW_STATE_SENDING_DATA);
        gpio_set_level(LED_GREEN_GPIO, 1);

        const char *topic = "tbmq/cs_000001/port01/telemetry";
        const char *payload = "\"voltage\": 48.5,\"current\": 6.2,\"energy\": 1.24";
        int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);

        if (msg_id >= 0)
        {
            ESP_LOGI("MQTT", "Published msg_id=%d", msg_id);
        }
        else
        {
            ESP_LOGW("MQTT", "Publish failed");
        }

        gpio_set_level(LED_GREEN_GPIO, 0);

        // quay lại trạng thái “online” theo interface ưu tiên
        if (ethernet_manager_has_ip())
            gateway_set_state(GW_STATE_ETH_ONLINE);
        else if (xEventGroupGetBits(s_net_event_group) & NET_WIFI_OK_BIT)
            gateway_set_state(GW_STATE_WIFI_ONLINE);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ---------------- WIFI event handler ---------------- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            gateway_set_state(GW_STATE_WIFI_CONNECTING);
            esp_wifi_connect();
            ESP_LOGI(TAG, "Wi-Fi STA started, connecting...");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Wi-Fi disconnected");
            xEventGroupClearBits(s_net_event_group, NET_WIFI_OK_BIT);
            if (mqtt_client)
            {
                esp_mqtt_client_stop(mqtt_client);
                esp_mqtt_client_destroy(mqtt_client);
                mqtt_client = NULL;
                mqtt_task_started = false;
                ESP_LOGI(TAG, "MQTT client stopped due to Wi-Fi lost");
                gpio_set_level(LED_BLUE_GPIO, 0);
            }

            if (s_retry_num < 5)
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/5)", s_retry_num);
            }
            else
            {
                // Không vào AP nữa. RESET chip để tự kết nối lại
                ESP_LOGE(TAG, "Wi-Fi cannot connect after retries → restarting...");
                esp_restart();
            }
            break;
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGE(TAG, "------case IP_EVENT_STA_GOT_IP---------");
            xEventGroupSetBits(s_net_event_group, NET_WIFI_OK_BIT);
            gateway_set_state(GW_STATE_WIFI_ONLINE);
            gpio_set_level(LED_BLUE_GPIO, 1);

            break;

        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // ✅ quan trọng: báo “WiFi có internet”
        xEventGroupSetBits(s_net_event_group, NET_WIFI_OK_BIT);

        gateway_set_state(GW_STATE_WIFI_ONLINE);
        gpio_set_level(LED_BLUE_GPIO, 1);
    }

}

/* ---------------- wifi init sta/ap/reset ---------------- */
void wifi_init_sta(const char *ssid, const char *pass)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. SSID:%s", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to AP success");
        gpio_set_level(LED_BLUE_GPIO, 1);
    }
    else
    {
        ESP_LOGW(TAG, "Wi-Fi connect failed but staying in STA mode and retry forever...");
    }
}

void wifi_init_ap(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "SETUP WIFI EV CHARGING STATION",
            .ssid_len = 0,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP Mode started. SSID:%s PASS:%s",
             (char *)ap_config.ap.ssid,
             strlen((char *)ap_config.ap.password) ? (char *)ap_config.ap.password : "(none)");

    start_webserver();
}

void wifi_reset_to_ap_mode(void)
{
    nvs_handle_t nvs;
    if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_erase_key(nvs, "wifi_ssid");
        nvs_erase_key(nvs, "wifi_pass");
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_deinit());
    ESP_LOGW(TAG, "Switching to AP mode for setup...");
    wifi_init_ap();
}

/* ---------------- button task ---------------- */
void wifi_reset_button_task(void *pvParameters)
{
    // --- Cấu hình GPIO5 làm INPUT ---
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << WIFI_RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf_in);

    // --- Cấu hình GPIO10, 38, 39, 40 làm OUTPUT ---
    gpio_config_t io_conf_out = {
        .pin_bit_mask =
            (1ULL << GPIO_NUM_10) | (1ULL << LED_RED_GPIO) | (1ULL << LED_BLUE_GPIO) | (1ULL << LED_GREEN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf_out);

    // --- Đặt mức logic 1 cho GPIO10 ---
    gpio_set_level(GPIO_NUM_10, 1);
    gpio_set_level(LED_RED_GPIO, 1);
    gpio_set_level(LED_BLUE_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 0);

    TickType_t press_start = 0;
    bool pressed = false;
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Button level at start: %d", gpio_get_level(WIFI_RESET_BUTTON_GPIO));

    while (1)
    {
        int level = gpio_get_level(WIFI_RESET_BUTTON_GPIO);
        if (level == 0 && !pressed)
        {
            pressed = true;
            press_start = xTaskGetTickCount();
        }
        else if (level == 1 && pressed)
        {
            pressed = false;
        }

        if (pressed)
        {
            TickType_t elapsed = xTaskGetTickCount() - press_start;
            if (elapsed >= pdMS_TO_TICKS(WIFI_RESET_HOLD_TIME_MS))
            {
                ESP_LOGW(TAG, "Button held 10s -> Reset Wi-Fi config");
                wifi_reset_to_ap_mode();
                pressed = false;
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void mqtt_ack_task(void *arg)
{
    while (1)
    {
        if (ack_flag && mqtt_client != NULL)
        {
            ack_flag = false;

            char ack_json[128];
            snprintf(ack_json, sizeof(ack_json),
                     "{\"id\":\"%s\"}", ack_id);

            esp_mqtt_client_publish(mqtt_client,
                                    "tbmq/cs_000001/port01/ack",
                                    ack_json,
                                    0,
                                    1,
                                    0);

            ESP_LOGI("ACK", "ACK sent: %s", ack_json);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------- main ---------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_net_event_group = xEventGroupCreate();
    gateway_set_state(GW_STATE_BOOT);

    // Start Ethernet trước (ưu tiên dây LAN nếu có)
    gateway_set_state(GW_STATE_ETH_CONNECTING);
    ESP_ERROR_CHECK(ethernet_manager_start());

    xTaskCreate(wifi_reset_button_task, "wifi_reset_button_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_ack_task, "mqtt_ack_task", 4096, NULL, 5, NULL);

    // MQTT publish task tạo 1 lần từ boot (tự chờ internet)
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 6, NULL);

    // Monitor mạng (ETH/WiFi) + quản lý state + stop mqtt khi mất internet
    xTaskCreate(net_monitor_task, "net_monitor_task", 4096, NULL, 7, NULL);

    // load saved credentials
    nvs_handle_t nvs;
    char ssid[64] = {0}, pass[128] = {0};
    size_t ssid_len = sizeof(ssid), pass_len = sizeof(pass);
    esp_err_t ssid_ok = ESP_FAIL, pass_ok = ESP_FAIL;

    if (nvs_open("storage", NVS_READONLY, &nvs) == ESP_OK)
    {
        ssid_ok = nvs_get_str(nvs, "wifi_ssid", ssid, &ssid_len);
        pass_ok = nvs_get_str(nvs, "wifi_pass", pass, &pass_len);
        nvs_close(nvs);
    }

    if (ssid_ok == ESP_OK && ssid[0] != '\0')
    {
        ESP_LOGI(TAG, "Found saved Wi-Fi SSID: %s", ssid);
        gateway_set_state(GW_STATE_WIFI_CONNECTING);
        wifi_init_sta(ssid, pass);
    }
    else
    {
        ESP_LOGI(TAG, "No saved Wi-Fi, starting in AP mode...");
        gateway_set_state(GW_STATE_AP_MODE);
        wifi_init_ap();
    }
}
