#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WATER_METER_OK = 0,
    WATER_METER_ERR_TIMEOUT,
    WATER_METER_ERR_CRC,
    WATER_METER_ERR_SLAVE_ID,
    WATER_METER_ERR_FUNCTION,
    WATER_METER_ERR_BYTE_COUNT,
    WATER_METER_ERR_EXCEPTION,
    WATER_METER_ERR_UART,
    WATER_METER_ERR_INVALID_ARG,
} water_meter_status_t;

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int rts_gpio;
    int baudrate;
    uart_parity_t parity;
    uint8_t slave_id;
    uint32_t timeout_ms;
    uint8_t retries;
} water_meter_cfg_t;

typedef struct {
    uint32_t raw_total_volume;
    uint64_t total_volume_liter;
    float total_volume_m3;
    float flow_liter_per_minute;
    bool flow_valid;
    uint32_t ts_ms;
    water_meter_status_t status;
    uint8_t exception_code;
} water_meter_data_t;

typedef struct water_meter_handle_t *water_meter_handle_t;

esp_err_t water_meter_init(const water_meter_cfg_t *cfg, water_meter_handle_t *out);
esp_err_t water_meter_deinit(water_meter_handle_t h);
esp_err_t water_meter_read_total(water_meter_handle_t h, water_meter_data_t *out);

const char *water_meter_status_name(water_meter_status_t status);

uint16_t water_meter_modbus_crc16(const uint8_t *buf, uint16_t len);
uint32_t water_meter_parse_raw_total(const uint8_t data[4]);
uint64_t water_meter_raw_to_liter(uint32_t raw_total_volume);

#ifdef __cplusplus
}
#endif
