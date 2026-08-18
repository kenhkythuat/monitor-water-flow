#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char device_id[32];      // "cs_000001"
    uint32_t number_device;  // số slave cần poll (hoặc theo bạn định nghĩa)

    // MQTT settings
    char mqtt_uri[96];       // "mqtt://ip:1883"
    char mqtt_user[64];
    char mqtt_pass[64];
    char mqtt_client_id[64];

    uint32_t water_slave_id;
    uint32_t water_baudrate;
    uint32_t water_poll_period_ms;
    uint32_t water_report_interval_s;
} gateway_config_t;

esp_err_t gateway_config_init(void);

// getters
const char *gateway_config_device_id(void);
uint32_t gateway_config_number_device(void);

const char *gateway_config_mqtt_uri(void);
const char *gateway_config_mqtt_user(void);
const char *gateway_config_mqtt_pass(void);
const char *gateway_config_mqtt_client_id(void);
uint32_t gateway_config_water_slave_id(void);
uint32_t gateway_config_water_baudrate(void);
uint32_t gateway_config_water_poll_period_ms(void);
uint32_t gateway_config_water_report_interval_s(void);

// setters (ghi NVS)
esp_err_t gateway_config_set_device_id(const char *id);
esp_err_t gateway_config_set_number_device(uint32_t n);

esp_err_t gateway_config_set_mqtt_uri(const char *uri);
esp_err_t gateway_config_set_mqtt_user(const char *user);
esp_err_t gateway_config_set_mqtt_pass(const char *pass);
esp_err_t gateway_config_set_mqtt_client_id(const char *cid);
esp_err_t gateway_config_set_water_slave_id(uint32_t slave_id);
esp_err_t gateway_config_set_water_baudrate(uint32_t baudrate);
esp_err_t gateway_config_set_water_poll_period_ms(uint32_t poll_period_ms);
esp_err_t gateway_config_set_water_report_interval_s(uint32_t interval_s);

esp_err_t gateway_config_get_all(gateway_config_t *out);

#ifdef __cplusplus
}
#endif
