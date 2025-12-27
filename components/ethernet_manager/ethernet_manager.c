#include "ethernet_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"

#include "ethernet_init.h"   // từ example

static const char *TAG = "eth_mgr";

#define ETHMGR_LINK_BIT   BIT0
#define ETHMGR_GOTIP_BIT  BIT1

static EventGroupHandle_t s_eth_evt;
static uint8_t s_port_cnt;
static esp_eth_handle_t *s_eth_handles;

static esp_netif_t *s_netifs[4];
static esp_eth_netif_glue_handle_t s_glues[4];

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    esp_eth_handle_t eth = *(esp_eth_handle_t *)data;

    if (id == ETHERNET_EVENT_CONNECTED) {
        uint8_t mac[6];
        esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "LINK UP %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        xEventGroupSetBits(s_eth_evt, ETHMGR_LINK_BIT);
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "LINK DOWN");
        xEventGroupClearBits(s_eth_evt, ETHMGR_LINK_BIT | ETHMGR_GOTIP_BIT);
    } else if (id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "START");
    } else if (id == ETHERNET_EVENT_STOP) {
        ESP_LOGI(TAG, "STOP");
        xEventGroupClearBits(s_eth_evt, ETHMGR_LINK_BIT | ETHMGR_GOTIP_BIT);
    }
}

static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id;
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "GOT IP " IPSTR, IP2STR(&e->ip_info.ip));
    xEventGroupSetBits(s_eth_evt, ETHMGR_GOTIP_BIT);
}

esp_err_t ethernet_manager_start(void)
{
    if (!s_eth_evt) {
        s_eth_evt = xEventGroupCreate();
        if (!s_eth_evt) return ESP_ERR_NO_MEM;
    }

    // Tạo driver handle(s) từ ethernet_init (cấu hình qua menuconfig)
    ESP_ERROR_CHECK(example_eth_init(&s_eth_handles, &s_port_cnt));
    if (s_port_cnt == 0 || s_port_cnt > 4) return ESP_FAIL;

    // Create netif + attach
    for (int i = 0; i < s_port_cnt; i++) {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        s_netifs[i] = esp_netif_new(&cfg);
        s_glues[i]  = esp_eth_new_netif_glue(s_eth_handles[i]);
        ESP_ERROR_CHECK(esp_netif_attach(s_netifs[i], s_glues[i]));
    }

    // Register events
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, NULL));

    // Start
    for (int i = 0; i < s_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(s_eth_handles[i]));
    }

    ESP_LOGI(TAG, "Ethernet started, ports=%u", s_port_cnt);
    return ESP_OK;
}

bool ethernet_manager_link_up(void)
{
    return s_eth_evt && (xEventGroupGetBits(s_eth_evt) & ETHMGR_LINK_BIT);
}

bool ethernet_manager_has_ip(void)
{
    return s_eth_evt && (xEventGroupGetBits(s_eth_evt) & ETHMGR_GOTIP_BIT);
}

esp_err_t ethernet_manager_get_ip(esp_netif_ip_info_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_netifs[0]) return ESP_ERR_INVALID_STATE;
    return esp_netif_get_ip_info(s_netifs[0], out);
}
