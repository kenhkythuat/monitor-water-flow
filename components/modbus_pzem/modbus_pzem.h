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
    bool pzem1_ok;   // ✅ lần đọc gần nhất PZEM1 OK?
    bool pzem2_ok;   // ✅ lần đọc gần nhất PZEM2 OK?
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

// ghi 1 thanh ghi holding (FC06) tới slave/reg
esp_err_t modbus_pzem_write_single_reg(uint8_t slave_id, uint16_t reg, uint16_t value);

// map portXX -> slave_id + reg(0x0100/0x0101) theo danh sách stm32_slaves
bool modbus_pzem_map_port_to_slave_reg(uint8_t port_num, uint8_t *out_slave, uint16_t *out_reg);

// ===== Port control with timeout =====

// Bật/tắt tải theo port (FC06 ghi 1/0 vào reg 0x0100/0x0101 tương ứng)
esp_err_t modbus_pzem_port_set(uint8_t port_num, bool on);

// Bật tải + set timeout (giây). timeout_s=0 => bật nhưng không auto tắt
esp_err_t modbus_pzem_port_start_timeout(uint8_t port_num, uint32_t timeout_s);

// Lấy thời gian còn lại (giây). return false nếu port invalid hoặc đang OFF.
// remain_s = 0 nghĩa là đã hết hạn (sắp bị task tắt / vừa tắt).
bool modbus_pzem_port_get_remaining(uint8_t port_num, uint32_t *remain_s);

// Sau khi bạn đã bật port bằng FC06 thành công, gọi hàm này để bắt đầu timeout
esp_err_t modbus_pzem_port_arm_timeout(uint8_t port_num, uint32_t timeout_s);

// Khi tắt port thành công, gọi hàm này để clear state
esp_err_t modbus_pzem_port_clear_state(uint8_t port_num);

// ===== STM32 Reset helper =====

// Gửi reset ngay (FC06 reg 0x0110 val 0x0001)
esp_err_t modbus_pzem_stm32_reset_now(uint8_t slave_id);

// Schedule reset sau delay_ms (không block caller)
esp_err_t modbus_pzem_stm32_reset_delay(uint8_t slave_id, uint32_t delay_ms);

// Schedule reset theo port (port -> slave mapping), sau delay_ms
esp_err_t modbus_pzem_port_reset_delay(uint8_t port_num, uint32_t delay_ms);





#ifdef __cplusplus
}
#endif
