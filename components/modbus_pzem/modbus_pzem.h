#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float voltage_v;     // V
    float current_a;     // A
    float power_w;       // W
    float energy_wh;     // Wh
    float freq_hz;       // Hz
    float pf;            // power factor
    uint16_t status;     // raw status reg
    bool updated;        // status bit0
    bool crc_fail;       // status bit1 (theo mô tả của bạn)
} pzem_data_t;

typedef struct {
    uint8_t  slave_id;   // STM32 modbus addr (0x01, 0x02...)
    pzem_data_t pzem1;   // base 0x0000..0x0009
    pzem_data_t pzem2;   // base 0x0010..0x0019
    uint32_t last_ok_ms;
    uint32_t last_err_ms;
    uint32_t ok_count;
    uint32_t err_count;
} stm32_meter_t;

typedef struct {
    int uart_num;        // UART_NUM_1 / UART_NUM_2 ...
    int tx_gpio;         // IO18
    int rx_gpio;         // IO17
    int de_gpio;         // IO16 (mapped to RTS)
    int baudrate;        // 9600
} rs485_modbus_cfg_t;

typedef struct {
    rs485_modbus_cfg_t rs485;

    // danh sách slave STM32 cần poll
    const uint8_t *slave_ids;
    uint8_t slave_count;

    // internet online bits: NET_WIFI_OK_BIT | NET_ETH_OK_BIT
    EventGroupHandle_t net_event_group;
    EventBits_t online_bits;

    // chu kỳ poll
    uint32_t poll_period_ms;      // vd 1000ms
    uint32_t inter_request_ms;    // delay giữa các request (vd 30~80ms)
} modbus_pzem_cfg_t;

esp_err_t modbus_pzem_init(const modbus_pzem_cfg_t *cfg);
esp_err_t modbus_pzem_start(void);  // tạo task poll

// lấy snapshot dữ liệu mới nhất (thread-safe)
uint8_t modbus_pzem_get_meter_count(void);
bool modbus_pzem_get_meter(uint8_t index, stm32_meter_t *out);

#ifdef __cplusplus
}
#endif
