#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;     // UART_NUM_1...
    int tx_gpio;
    int rx_gpio;
    int rts_gpio;             // dùng làm DE/RE cho RS485 (enable direction)
    int baudrate;             // 9600
    uart_parity_t parity;     // UART_PARITY_DISABLE / EVEN / ODD
    uint8_t slave_id;         // 1
    uint16_t reg_addr;        // địa chỉ bắt đầu (0x006B theo frame của bạn)
    uint16_t reg_qty;         // số thanh ghi đọc (2 theo frame của bạn)
    uint32_t timeout_ms;      // timeout chờ phản hồi
} ech306l_cfg_t;

typedef struct ech306l_handle_t* ech306l_handle_t;

/**
 * @brief Khởi tạo UART RS485 + cấu hình cảm biến
 */
esp_err_t ech306l_init(const ech306l_cfg_t *cfg, ech306l_handle_t *out);

/**
 * @brief Huỷ handle (giải phóng bộ nhớ). UART driver không auto delete để bạn tái dùng.
 */
esp_err_t ech306l_deinit(ech306l_handle_t h);

/**
 * @brief Đọc khoảng cách (cm).
 *        Mặc định parse 2 register -> uint32 (big-endian register order).
 *        Bạn có thể đổi cách parse nếu datasheet chỉ ra kiểu khác.
 */
esp_err_t ech306l_read_distance_cm(ech306l_handle_t h, float *distance_cm);

/**
 * @brief (Tuỳ chọn) đọc raw 2 thanh ghi (reg_qty=2) để bạn debug.
 */
esp_err_t ech306l_read_raw_u16x2(ech306l_handle_t h, uint16_t *reg0, uint16_t *reg1);

esp_err_t ech306l_read_temperature_c(ech306l_handle_t h, float *temp_c);
esp_err_t ech306l_read_temperature_raw_u16x2(ech306l_handle_t h, uint16_t *reg0, uint16_t *reg1);

#ifdef __cplusplus
}
#endif
