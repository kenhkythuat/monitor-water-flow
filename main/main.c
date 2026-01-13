// main.c
#include <stdio.h>
#include "gateway_state.h"
#include "gateway_core.h"
#include "status_led.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
#include "esp_timer.h"
#include "mqtt_client.h"
#include "ethernet_manager.h"
#include "modbus_pzem.h"
#include "gateway_config.h"
#include "gw_time.h"
#include "gw_temp.h"


#define WIFI_RESET_BUTTON_GPIO 41
#define WIFI_RESET_HOLD_TIME_MS 10000
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define LED_RED_GPIO 21
#define LED_BLUE_GPIO 14
#define LED_GREEN_GPIO 13

static const char *TAG = "wifi_setup";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static int pending_ack_id = -1;
static bool ack_flag = false;
static char ack_id[64] = {0};

static esp_mqtt_client_handle_t mqtt_client = NULL; // client toàn cục
static bool mqtt_task_started = false;              // để tránh tạo nhiều lần
static esp_netif_t *s_wifi_netif = NULL;

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

// variable modbus_rtu form stm32
static uint8_t *s_slave_ids = NULL;
static size_t s_slave_count = 0;

// check error wifi
#define MQTT_CONNECTED_BIT (1 << 0)
static SemaphoreHandle_t s_mqtt_start_lock;
static bool s_mqtt_starting = false;

static EventGroupHandle_t s_mqtt_event_group;
static volatile bool s_mqtt_auth_failed = false;
static volatile uint32_t s_last_puback_ms = 0;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static esp_err_t build_slave_list_from_nvs(void)
{
    uint32_t n = gateway_config_number_device(); // ví dụ 3

    // Giới hạn hợp lý (tuỳ bạn chỉnh)
    if (n == 0)
        n = 1;
    if (n > 32)
        n = 32;

    // giải phóng nếu đã có
    if (s_slave_ids)
    {
        free(s_slave_ids);
        s_slave_ids = NULL;
        s_slave_count = 0;
    }

    s_slave_ids = (uint8_t *)calloc(n, sizeof(uint8_t));
    if (!s_slave_ids)
        return ESP_ERR_NO_MEM;

    for (uint32_t i = 0; i < n; i++)
    {
        s_slave_ids[i] = (uint8_t)(0x01 + i); // 0x01..0x01+n-1
    }
    s_slave_count = n;

    ESP_LOGI(TAG, "Modbus slaves from NVS: count=%u, addr=0x%02X..0x%02X",
             (unsigned)s_slave_count, s_slave_ids[0], s_slave_ids[s_slave_count - 1]);

    return ESP_OK;
}
typedef struct
{
    uint8_t port; // port01..portNN
    uint8_t slave;
    uint16_t reg;
    uint16_t value;
    char cmd[24];
    float max_current;
    int timeout_s;
} gw_cmd_t;

static QueueHandle_t s_cmd_q;

// restart trễ để kịp gửi ACK/flush log
static void delayed_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

// static void mqtt_manager_start(void)
// {
//     if (mqtt_client)
//         return;

//     gateway_set_state(GW_STATE_MQTT_CONNECTING);

//     esp_mqtt_client_config_t mqtt_cfg = {
//         .broker.address.uri = gateway_config_mqtt_uri(),
//         .credentials.username = gateway_config_mqtt_user(),
//         .credentials.authentication.password = gateway_config_mqtt_pass(),
//         .credentials.client_id = gateway_config_mqtt_client_id(),
//         .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
//     };

//     mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//     if (!mqtt_client)
//     {
//         ESP_LOGE("MQTT", "Failed to init client");
//         return;
//     }

//     esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
//     esp_err_t err = esp_mqtt_client_start(mqtt_client);
//     if (err != ESP_OK)
//     {
//         ESP_LOGE("MQTT", "Failed to start: %s", esp_err_to_name(err));
//         esp_mqtt_client_destroy(mqtt_client);
//         mqtt_client = NULL;
//         return;
//     }
// }
void mqtt_manager_init(void)
{
    if (!s_mqtt_start_lock) {
        s_mqtt_start_lock = xSemaphoreCreateMutex();
    }
}

void mqtt_manager_start(void)
{
    if (!s_mqtt_start_lock) mqtt_manager_init();

    // Fast path (không lock) – OK nhưng vẫn cần lock để tránh race
    if (mqtt_client) return;

    xSemaphoreTake(s_mqtt_start_lock, portMAX_DELAY);

    // Double-check sau khi đã lock
    if (mqtt_client || s_mqtt_starting) {
        xSemaphoreGive(s_mqtt_start_lock);
        return;
    }

    s_mqtt_starting = true;
    xSemaphoreGive(s_mqtt_start_lock);

    gateway_set_state(GW_STATE_MQTT_CONNECTING);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = gateway_config_mqtt_uri(),
        .credentials.username = gateway_config_mqtt_user(),
        .credentials.authentication.password = gateway_config_mqtt_pass(),
        .credentials.client_id = gateway_config_mqtt_client_id(),
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (!client) {
        ESP_LOGE("MQTT", "Failed to init client");
        xSemaphoreTake(s_mqtt_start_lock, portMAX_DELAY);
        s_mqtt_starting = false;
        xSemaphoreGive(s_mqtt_start_lock);
        return;
    }

    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);

    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        ESP_LOGE("MQTT", "Failed to start: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(client);

        xSemaphoreTake(s_mqtt_start_lock, portMAX_DELAY);
        s_mqtt_starting = false;
        xSemaphoreGive(s_mqtt_start_lock);
        return;
    }

    // publish handle “chính thức” sau khi start OK
    xSemaphoreTake(s_mqtt_start_lock, portMAX_DELAY);
    mqtt_client = client;
    s_mqtt_starting = false;
    xSemaphoreGive(s_mqtt_start_lock);
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
    bool last_any_online = false;

    while (1)
    {
        bool eth_ip = ethernet_manager_has_ip();

        if (eth_ip) xEventGroupSetBits(s_net_event_group, NET_ETH_OK_BIT);
        else        xEventGroupClearBits(s_net_event_group, NET_ETH_OK_BIT);

        bool wifi_ip = (xEventGroupGetBits(s_net_event_group) & NET_WIFI_OK_BIT);
        bool any_online = eth_ip || wifi_ip;

        // ✅ Transition OFFLINE -> ONLINE
        if (any_online && !last_any_online)
        {
            ESP_LOGI("NET", "Internet UP -> init time + start MQTT");

            // SNTP/time sync: chỉ gọi khi vừa có mạng lần đầu
            gw_time_init();

            // MQTT: start khi có mạng
            mqtt_manager_start();
        }

        // ✅ Transition ONLINE -> OFFLINE
        if (!any_online && last_any_online)
        {
            ESP_LOGW("NET", "Internet DOWN -> stop MQTT");

            // Clear bit để các task publish không hiểu nhầm
            xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);

            // Stop MQTT để tránh spam reconnect khi không có mạng
            mqtt_manager_stop();
        }

        // ưu tiên ETH khi vừa có IP
        if (eth_ip && !last_eth_ip)
        {
            esp_netif_t *eth = ethernet_manager_get_netif();
            if (eth) esp_netif_set_default_netif(eth);
            gateway_set_state(GW_STATE_ETH_ONLINE);
        }

        // nếu ETH mất IP mà WiFi còn -> fallback WiFi
        if (!eth_ip && last_eth_ip && wifi_ip && s_wifi_netif)
        {
            esp_netif_set_default_netif(s_wifi_netif);
            gateway_set_state(GW_STATE_WIFI_ONLINE);
        }

        last_eth_ip = eth_ip;
        last_any_online = any_online;

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}



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
        "    <h3>EV Bike Wi-Fi Setup</h3>\n"
        "    <p>Nhập SSID và mật khẩu mạng Wi-Fi bạn muốn Gateway EV kết nối.</p>\n"
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
static bool parse_port_from_topic(const char *topic, uint8_t *out_port)
{
    // topic: tbmq/cs_000001/port01/command
    const char *p = strstr(topic, "/port");
    if (!p)
        return false;
    p += 5; // skip "/port"
    int port = atoi(p);
    if (port <= 0 || port > 99)
        return false;
    *out_port = (uint8_t)port;
    return true;
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
        s_mqtt_auth_failed = false;
        xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
        gateway_set_state(GW_STATE_MQTT_CONNECTED);

        char sub_cmd[128];
        snprintf(sub_cmd, sizeof(sub_cmd), "tbmq/%s/+/command", gateway_config_device_id());
        esp_mqtt_client_subscribe(event->client, sub_cmd, 0);
        ESP_LOGI("MQTT", "Subscribed to %s", sub_cmd);
        // (tuỳ chọn) config topic riêng
        char sub_cfg[128];
        snprintf(sub_cfg, sizeof(sub_cfg), "tbmq/%s/config", gateway_config_device_id());
        esp_mqtt_client_subscribe(event->client, sub_cfg, 0);

        ESP_LOGI("MQTT", "Subscribed to %s", sub_cfg);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "Subscribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI("MQTT", "Before connect");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "Published OK, msg_id=%d", event->msg_id);
        s_last_puback_ms = now_ms();
        break;

    case MQTT_EVENT_DATA:
    {
        char topic_buf[128] = {0};
        snprintf(topic_buf, sizeof(topic_buf), "%.*s", event->topic_len, event->topic);

        char data_buf[512] = {0};
        snprintf(data_buf, sizeof(data_buf), "%.*s", event->data_len, event->data);

        ESP_LOGI("MQTT", "Received topic: %s", topic_buf);
        ESP_LOGI("MQTT", "Received data : %s", data_buf);

        cJSON *json = cJSON_Parse(data_buf);
        if (!json)
        {
            ESP_LOGE("MQTT", "Invalid JSON payload!");
            break;
        }

        /* =======================
           1) GIỮ ACK LOGIC CŨ
           ======================= */
        cJSON *id = cJSON_GetObjectItem(json, "id");
        if (cJSON_IsString(id) && id->valuestring != NULL)
        {
            strncpy(ack_id, id->valuestring, sizeof(ack_id) - 1);
            ack_flag = true;
            ESP_LOGI("MQTT", "Parsed CMD id=%s -> ACK queued", ack_id);
        }

        /* ==========================================================
           2) LẤY cmd/param (hỗ trợ root hoặc nằm trong payload)
           ========================================================== */
        cJSON *payload = cJSON_GetObjectItem(json, "payload");
        cJSON *cmd_obj = NULL;
        cJSON *param = NULL;

        // Ưu tiên format có payload (giống command hiện tại của bạn)
        if (cJSON_IsObject(payload))
        {
            cmd_obj = cJSON_GetObjectItem(payload, "cmd");
            param = cJSON_GetObjectItem(payload, "param");
        }

        // Fallback: format root {cmd, param}
        if (!cJSON_IsString(cmd_obj))
        {
            cmd_obj = cJSON_GetObjectItem(json, "cmd");
            param = cJSON_GetObjectItem(json, "param");
        }

        if (!cJSON_IsString(cmd_obj) || !cmd_obj->valuestring)
        {
            ESP_LOGW("MQTT", "No valid \"cmd\" in command!");
            cJSON_Delete(json);
            break;
        }

        const char *cmd = cmd_obj->valuestring;

        /* ==========================================================
           3) CMD: config_device  -> update NVS -> restart
           ========================================================== */
        if (strcmp(cmd, "config_device") == 0)
        {
            if (!cJSON_IsObject(param))
            {
                ESP_LOGW("MQTT", "config_device: missing param object");
                cJSON_Delete(json);
                break;
            }

            cJSON *did = cJSON_GetObjectItem(param, "device_id");
            cJSON *nd = cJSON_GetObjectItem(param, "number_device");

            bool changed = false;

            if (cJSON_IsString(did) && did->valuestring && did->valuestring[0] != '\0')
            {
                esp_err_t e = gateway_config_set_device_id(did->valuestring);
                if (e == ESP_OK)
                {
                    ESP_LOGI("MQTT", "config_device: device_id -> %s", did->valuestring);
                    changed = true;
                }
                else
                {
                    ESP_LOGW("MQTT", "config_device: set device_id failed: %s", esp_err_to_name(e));
                }
            }

            if (cJSON_IsNumber(nd))
            {
                uint32_t num = (uint32_t)nd->valuedouble;
                if (num > 0 && num <= 99)
                {
                    esp_err_t e = gateway_config_set_number_device(num);
                    if (e == ESP_OK)
                    {
                        ESP_LOGI("MQTT", "config_device: number_device -> %lu", (unsigned long)num);
                        changed = true;
                    }
                    else
                    {
                        ESP_LOGW("MQTT", "config_device: set number_device failed: %s", esp_err_to_name(e));
                    }
                }
                else
                {
                    ESP_LOGW("MQTT", "config_device: invalid number_device=%lu", (unsigned long)num);
                }
            }

            cJSON_Delete(json);

            if (changed)
            {
                ESP_LOGW("MQTT", "Config updated -> restarting...");
                xTaskCreate(delayed_restart_task, "restart_task", 2048, NULL, 10, NULL);
            }
            else
            {
                ESP_LOGW("MQTT", "Config_device received but no change applied");
            }
            break; // kết thúc MQTT_EVENT_DATA
        }

        /* ==========================================================
           4) CMD start/stop_charge chỉ xử lý khi topic là /command
           ========================================================== */
        if (strstr(topic_buf, "/command") != NULL)
        {
            uint8_t port = 0;
            if (!parse_port_from_topic(topic_buf, &port))
            {
                ESP_LOGW("MQTT", "Cannot parse port from topic=%s", topic_buf);
                cJSON_Delete(json);
                break;
            }

            // param optional
            float max_current = 0.0f;
            int timeout_s = 0;

            if (cJSON_IsObject(param))
            {
                cJSON *mc = cJSON_GetObjectItem(param, "max_current");
                cJSON *to = cJSON_GetObjectItem(param, "timeout");
                if (cJSON_IsNumber(mc))
                    max_current = (float)mc->valuedouble;
                if (cJSON_IsNumber(to))
                    timeout_s = to->valueint;
            }

            // map port -> slave/reg (0x0100/0x0101)
            uint8_t slave = 0;
            uint16_t reg = 0;
            if (!modbus_pzem_map_port_to_slave_reg(port, &slave, &reg))
            {
                ESP_LOGW("MQTT", "Port%02u out of range (check stm32_slaves[])", port);
                cJSON_Delete(json);
                break;
            }

            // cmd -> value FC06
            uint16_t value = 0x0000;
            if (strcmp(cmd, "start_charge") == 0)
            {
                value = 0x0003; // relay ON + led ON
            }
            else if (strcmp(cmd, "stop_charge") == 0)
                value = 0x0000; // OFF
            else
            {
                ESP_LOGW("MQTT", "Unknown cmd=%s", cmd);
                cJSON_Delete(json);
                break;
            }

            // enqueue command (không write modbus trong callback)
            if (s_cmd_q)
            {
                gw_cmd_t c = {0};
                c.port = port;
                c.slave = slave;
                c.reg = reg;
                c.value = value;
                c.max_current = max_current;
                c.timeout_s = timeout_s;
                strncpy(c.cmd, cmd, sizeof(c.cmd) - 1);

                if (xQueueSend(s_cmd_q, &c, 0) != pdTRUE)
                    ESP_LOGW("MQTT", "cmd queue full, drop port%02u cmd=%s", port, c.cmd);
                else
                    ESP_LOGI("MQTT",
                             "CMD queued: port%02u -> slave=0x%02X reg=0x%04X val=0x%04X (maxI=%.2f timeout=%ds)",
                             port, slave, reg, value, max_current, timeout_s);
            }
            else
            {
                ESP_LOGW("MQTT", "s_cmd_q not init, drop command");
            }
        }
        cJSON_Delete(json);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW("MQTT", "Disconnected");
        xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
        break;

    case MQTT_EVENT_ERROR:
    {
        ESP_LOGE("MQTT", "Error");
        xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);

        int rc = -1;
        if (event->error_handle)
            rc = event->error_handle->connect_return_code;
        ESP_LOGE("MQTT", "Error: connack_rc=%d", rc);

        // rc==5: Not authorized
        if (rc == 5)
        {
            s_mqtt_auth_failed = true;
            ESP_LOGE("MQTT", "AUTH_FAIL (not authorized) -> stop publish");
            // (khuyến nghị) stop client để khỏi reconnect spam
            if (mqtt_client)
                esp_mqtt_client_stop(mqtt_client);
        }
        break;
    }

    default:
        ESP_LOGW("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

/* ---------------- MQTT Task ---------------- */

void status_led_pulse_green(uint32_t ms);

// Status theo từng PZEM (riêng biệt) + slave_id + pzem_ok
static void pzem_status_str(char *out, size_t out_sz,
                            const pzem_data_t *p,
                            uint8_t slave_id,
                            bool pzem_ok)
{
    if (!out || out_sz == 0)
        return;

    // default
    strncpy(out, "unknown", out_sz - 1);
    out[out_sz - 1] = 0;

    if (!p)
        return;

    // 1) Không phản hồi slave
    if (!pzem_ok)
    {
        snprintf(out, out_sz, "No Response 0x%02X", slave_id);
        return;
    }

    // 2) CRC fail (nếu STM32 đánh dấu)
    if (p->crc_fail)
    {
        strncpy(out, "error", out_sz - 1);
        out[out_sz - 1] = 0;
        return;
    }

    // 3) Điện áp = 0 => Off
    // (dùng <= 0.01f để tránh nhiễu float)
    if (p->voltage_v <= 0.01f)
    {
        strncpy(out, "available", out_sz - 1);
        out[out_sz - 1] = 0;
        return;
    }

    // 4) Charging khi dòng > 0.02A
    if (p->current_a > 0.02f)
    {
        strncpy(out, "charging", out_sz - 1);
        out[out_sz - 1] = 0;
        return;
    }

    // 5) Idle khi V > 180V và I < 0.02A
    if (p->voltage_v > 180.0f && p->current_a <= 0.02f)
    {
        strncpy(out, "charging", out_sz - 1);
        out[out_sz - 1] = 0;
        return;
    }

    // fallback
    strncpy(out, "Idle", out_sz - 1);
    out[out_sz - 1] = 0;
}

static void mqtt_publish_task(void *pvParameters)
{
    while (1)
    {
        // Chờ có ít nhất 1 mạng OK (WiFi hoặc ETH)
        xEventGroupWaitBits(s_net_event_group,
                            NET_WIFI_OK_BIT | NET_ETH_OK_BIT,
                            pdFALSE, pdFALSE,
                            portMAX_DELAY);
        // 2) nếu auth fail thì báo rõ và skip (đỡ hiểu nhầm “vẫn chạy”)
        if (s_mqtt_auth_failed) {
            ESP_LOGE("MQTT", "Skip publish: AUTH_FAIL (check username/password/ACL)");
            // bạn có thể set state riêng: GW_STATE_MQTT_AUTH_FAIL
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Có mạng -> đảm bảo MQTT chạy
        //mqtt_manager_start();

        EventBits_t b = xEventGroupWaitBits(s_mqtt_event_group, MQTT_CONNECTED_BIT,
                                        pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));
        if (!(b & MQTT_CONNECTED_BIT) || !mqtt_client) {
            ESP_LOGW("MQTT", "Not connected -> skip this cycle");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Lấy số STM32 đang poll (theo stm32_slaves[] bạn khai báo)
        uint8_t meter_cnt = modbus_pzem_get_meter_count();

        // Với mỗi STM32: publish 2 lần (PZEM1->portX, PZEM2->portX+1)
        for (uint8_t i = 0; i < meter_cnt; i++)
        {
            stm32_meter_t m;
            if (!modbus_pzem_get_meter(i, &m))
                continue;
            uint8_t port_base = (uint8_t)(i * 2 + 1);

            // publish cho PZEM1 rồi PZEM2
            for (uint8_t k = 0; k < 2; k++)
            {
                const pzem_data_t *p = (k == 0) ? &m.pzem1 : &m.pzem2;
                uint8_t port_num = (uint8_t)(port_base + k);

                char topic[128];
                snprintf(topic, sizeof(topic),
                         "tbmq/%s/port%02u/telemetry", gateway_config_device_id(), (unsigned)port_num);
                float energy_kwh = p->energy_wh;

                char status[32];
                bool ok = (k == 0) ? m.pzem1_ok : m.pzem2_ok;
                pzem_status_str(status, sizeof(status), p, m.slave_id, ok);
                //get timestamp
                uint32_t ts = gw_time_is_synced() ? gw_time_unix() : 0;
                char payload[256];
                int len = snprintf(payload, sizeof(payload),
                                   "{"
                                   "\"ts\":%u,"
                                   "\"data\":{"
                                   "\"voltage\":%.1f,"
                                   "\"current\":%.3f,"
                                   "\"power\":%.1f,"
                                   "\"energy\":%.1f,"
                                   "\"status\":\"%s\""
                                   "}"
                                   "}",
                                   (unsigned)ts,
                                   p->voltage_v,
                                   p->current_a,
                                   p->power_w,
                                   energy_kwh,
                                   status);

                if (len <= 0 || len >= (int)sizeof(payload))
                {
                    ESP_LOGW("MQTT", "Payload too long, skip meter_idx=%u port=%u", i, port_num);
                    continue;
                }

                int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);

                if (msg_id >= 0)
                {
                    ESP_LOGI("MQTT", "Published msg_id=%d topic=%s payload=%s", msg_id, topic, payload);
                    // Publish (giữ đúng style cũ)
                    status_led_pulse_green(500);
                    gateway_set_state(GW_STATE_SENDING_DATA);
                }
                else
                {
                    ESP_LOGW("MQTT", "Publish failed topic=%s", topic);
                }

                // quay lại trạng thái “online” theo interface ưu tiên
                if (ethernet_manager_has_ip())
                    gateway_set_state(GW_STATE_ETH_ONLINE);
                else if (xEventGroupGetBits(s_net_event_group) & NET_WIFI_OK_BIT)
                    gateway_set_state(GW_STATE_WIFI_ONLINE);

                vTaskDelay(pdMS_TO_TICKS(150)); // nhẹ broker
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // chu kỳ gửi
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
        {
            ESP_LOGW(TAG, "Wi-Fi disconnected");
            xEventGroupClearBits(s_net_event_group, NET_WIFI_OK_BIT);

            bool eth_ok = ethernet_manager_has_ip() ||
                          (xEventGroupGetBits(s_net_event_group) & NET_ETH_OK_BIT);

            // ✅ Chỉ stop MQTT nếu mất cả WiFi và Ethernet
            if (!eth_ok)
            {
                if (mqtt_client)
                {
                    esp_mqtt_client_stop(mqtt_client);
                    esp_mqtt_client_destroy(mqtt_client);
                    mqtt_client = NULL;
                    mqtt_task_started = false;
                    ESP_LOGI(TAG, "MQTT client stopped due to internet lost");
                }
            }
            else
            {
                ESP_LOGI(TAG, "ETH online -> keep MQTT, WiFi will retry in background");
            }

            // ✅ WiFi vẫn retry, nhưng KHÔNG restart nếu Ethernet đang online
            if (s_retry_num < 20)
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/5)", s_retry_num);
            }
            else
            {
                if (!eth_ok)
                {
                    ESP_LOGE(TAG, "Wi-Fi cannot connect after retries AND no ETH -> restarting...");
                    esp_restart();
                }
                else
                {
                    // có ETH thì chỉ backoff rồi thử lại, không restart
                    ESP_LOGW(TAG, "Wi-Fi retries exceeded but ETH OK -> backoff 10s then retry");
                    vTaskDelay(pdMS_TO_TICKS(10000));
                    s_retry_num = 0;
                    esp_wifi_connect();
                }
            }
            break;
        }

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
    }
}

/* ---------------- wifi init sta/ap/reset ---------------- */
void wifi_init_sta(const char *ssid, const char *pass)
{
    s_wifi_event_group = xEventGroupCreate();

    // esp_netif_create_default_wifi_sta();
    s_wifi_netif = esp_netif_create_default_wifi_sta(); // ✅ lưu lại handle
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
    gpio_config_t io_conf_out = {
        .pin_bit_mask =
            (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf_out);

    // --- Đặt mức logic 1 cho GPIO10 ---
    gpio_set_level(GPIO_NUM_10, 1);

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

static void mqtt_cmd_task(void *arg)
{
    gw_cmd_t c;
    while (1)
    {
        if (xQueueReceive(s_cmd_q, &c, portMAX_DELAY) != pdTRUE)
            continue;

        ESP_LOGI("CMD", "Exec cmd=%s port%02u -> slave=0x%02X reg=0x%04X val=0x%04X maxI=%.2f timeout=%ds",
                 c.cmd, c.port, c.slave, c.reg, c.value, c.max_current, c.timeout_s);

        esp_err_t e = modbus_pzem_write_single_reg(c.slave, c.reg, c.value);
        if (e == ESP_OK)
        {
            ESP_LOGI("CMD", "OK cmd=%s port%02u", c.cmd, c.port);

            // ✅ QUẢN LÝ TIMEOUT SAU KHI MODBUS OK
            if (strcmp(c.cmd, "start_charge") == 0)
            {
                uint32_t to = (c.timeout_s > 0) ? (uint32_t)c.timeout_s : 0;
                esp_err_t te = modbus_pzem_port_arm_timeout(c.port, to);
                if (te != ESP_OK) {
                    ESP_LOGW("CMD", "arm_timeout fail port%02u err=%s", c.port, esp_err_to_name(te));
                } else {
                    uint32_t remain;
                    if (modbus_pzem_port_get_remaining(c.port, &remain)) {
                        if (remain == 0xFFFFFFFFu)
                            ESP_LOGI("CMD", "port%02u ON (no timeout)", c.port);
                        else
                            ESP_LOGI("CMD", "port%02u timeout remaining=%us", c.port, (unsigned)remain);
                    }
                }
            }
            else if (strcmp(c.cmd, "stop_charge") == 0)
            {
                esp_err_t te = modbus_pzem_port_clear_state(c.port);
                if (te != ESP_OK) {
                    ESP_LOGW("CMD", "clear_state fail port%02u err=%s", c.port, esp_err_to_name(te));
                } else {
                    ESP_LOGI("CMD", "port%02u OFF (timeout cleared)", c.port);
                }
            }
        }
        else
        {
            ESP_LOGW("CMD", "FAIL cmd=%s port%02u err=%s", c.cmd, c.port, esp_err_to_name(e));

            // ❗ Nếu bật thất bại thì không arm timeout.
            // (tuỳ bạn) nếu stop_charge thất bại thì không clear state.
        }

        // publish ack lại server
        if (mqtt_client)
        {
            char topic[96];
            char payload[128];
            snprintf(topic, sizeof(topic), "tbmq/%s/port%02u/ack", gateway_config_device_id(), c.port);
            snprintf(payload, sizeof(payload), "{\"cmd\":\"%s\",\"result\":\"%s\"}",
                     c.cmd, (e == ESP_OK) ? "ok" : "fail");
            esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
        }
    }
}


static esp_err_t mqtt_publish_status_gateway(void)
{
    if (!mqtt_client) return ESP_ERR_INVALID_STATE;

    // 1) Timestamp
    uint32_t ts = gw_time_is_synced() ? gw_time_unix() : 0;

    // 2) Tổng hợp từ các meter/port đang poll
    uint8_t meter_cnt = modbus_pzem_get_meter_count();

    int active_ports = 0;
    double total_power_w = 0.0;
    double grid_v_sum = 0.0;
    int grid_v_cnt = 0;

    for (uint8_t i = 0; i < meter_cnt; i++)
    {
        stm32_meter_t m;
        if (!modbus_pzem_get_meter(i, &m)) continue;

        // PZEM1
        if (m.pzem1_ok && m.pzem1.voltage_v > 1.0f) {
            active_ports++;
            total_power_w += (double)m.pzem1.power_w;
            grid_v_sum += (double)m.pzem1.voltage_v;
            grid_v_cnt++;
        }

        // PZEM2
        if (m.pzem2_ok && m.pzem2.voltage_v > 1.0f) {
            active_ports++;
            total_power_w += (double)m.pzem2.power_w;
            grid_v_sum += (double)m.pzem2.voltage_v;
            grid_v_cnt++;
        }
    }

    float grid_voltage = (grid_v_cnt > 0) ? (float)(grid_v_sum / grid_v_cnt) : 0.0f;

    // 3) Nhiệt độ chip ESP32-S3 (nếu bạn đã thêm gw_temp)
    float temp_c = 0.0f;
    if (gw_temp_read_celsius(&temp_c) != ESP_OK) {
        temp_c = 0.0f;
    }

    // 4) Topic + payload
    char topic[128];
    snprintf(topic, sizeof(topic),
             "tbmq/%s/telemetry",
             gateway_config_device_id());

    char payload[256];
    int len = snprintf(payload, sizeof(payload),
                       "{"
                       "\"ts\":%u,"
                       "\"summary\":{"
                       "\"active_ports\":%d,"
                       "\"total_power\":%.0f,"
                       "\"grid_voltage\":%.1f,"
                       "\"temperature\":%.1f"
                       "}"
                       "}",
                       (unsigned)ts,
                       active_ports,
                       total_power_w,
                       grid_voltage,
                       temp_c);

    if (len <= 0 || len >= (int)sizeof(payload)) {
        ESP_LOGW("MQTT", "GW summary payload too long -> skip");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW("MQTT", "GW summary publish failed topic=%s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI("MQTT", "GW summary published msg_id=%d topic=%s payload=%s", msg_id, topic, payload);
    return ESP_OK;
}

static void mqtt_publish_gateway_status_task(void *pvParameters)
{
    while (1)
    {
        // Chờ có ít nhất 1 mạng OK (WiFi hoặc ETH)
        xEventGroupWaitBits(s_net_event_group,
                            NET_WIFI_OK_BIT | NET_ETH_OK_BIT,
                            pdFALSE, pdFALSE,
                            portMAX_DELAY);

        if (s_mqtt_auth_failed) {
            ESP_LOGE("MQTT", "Skip GW status publish: AUTH_FAIL");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        //mqtt_manager_start();

        EventBits_t b = xEventGroupWaitBits(s_mqtt_event_group, MQTT_CONNECTED_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));
        if (!(b & MQTT_CONNECTED_BIT) || !mqtt_client) {
            ESP_LOGW("MQTT", "Not connected -> skip GW status cycle");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Publish summary/status gateway
        if (mqtt_publish_status_gateway() == ESP_OK) {
            // bạn muốn thì set state riêng
            // gateway_set_state(GW_STATE_SENDING_GW_STATUS);
            status_led_pulse_green(250);
        }

        // chu kỳ status gateway (khuyến nghị chậm hơn port telemetry)
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}


/* ---------------- main ---------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(gateway_config_init());
    ESP_ERROR_CHECK(build_slave_list_from_nvs());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(gw_temp_init());

    s_net_event_group = xEventGroupCreate();
    s_mqtt_event_group = xEventGroupCreate();
    gateway_core_init();
    status_led_cfg_t cfg = {
        .red_gpio = LED_RED_GPIO,
        .blue_gpio = LED_BLUE_GPIO,
        .green_gpio = LED_GREEN_GPIO,
        .active_high = false,
    };
    ESP_ERROR_CHECK(status_led_init(&cfg)); // ✅ tạo task LED trước
    gateway_set_state(GW_STATE_ETH_CONNECTING);
    esp_err_t err = ethernet_manager_start(); // nên bỏ ESP_ERROR_CHECK để khỏi reboot
    xTaskCreate(wifi_reset_button_task, "wifi_reset_button_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_ack_task, "mqtt_ack_task", 4096, NULL, 5, NULL);

    // MQTT publish task tạo 1 lần từ boot (tự chờ internet)
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 6, NULL);
    xTaskCreate(mqtt_publish_gateway_status_task,
            "mqtt_gw_status",
            4096,
            NULL,
            5,
            NULL);


    s_cmd_q = xQueueCreate(10, sizeof(gw_cmd_t));
    xTaskCreate(mqtt_cmd_task, "mqtt_cmd_task", 4096, NULL, 7, NULL);

    // Monitor mạng (ETH/WiFi) + quản lý state + stop mqtt khi mất internet
    xTaskCreate(net_monitor_task, "net_monitor_task", 4096, NULL, 7, NULL);

    modbus_pzem_cfg_t mb = {
        .rs485 = {
            .uart_num = 2,
            .tx_gpio = 10,
            .rx_gpio = 12,
            .de_gpio = 11,
            .baudrate = 9600,
        },
        .slave_ids = s_slave_ids,
        .slave_count = s_slave_count,

        .net_event_group = s_net_event_group,
        .online_bits = NET_WIFI_OK_BIT | NET_ETH_OK_BIT,

        .poll_period_ms = 10000,
        .inter_request_ms = 100,
    };

    ESP_ERROR_CHECK(modbus_pzem_init(&mb));
    ESP_ERROR_CHECK(modbus_pzem_start());

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
