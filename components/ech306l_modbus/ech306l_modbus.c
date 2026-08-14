#include "ech306l_modbus.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"    
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ech306l";

// ====== Mặc định theo frame bạn gửi ======
#ifndef ECH306L_DEFAULT_REG_ADDR
#define ECH306L_DEFAULT_REG_ADDR   (0x006B)   // frame: 00 6B
#endif

#ifndef ECH306L_DEFAULT_REG_QTY
#define ECH306L_DEFAULT_REG_QTY    (2)
#endif

#define ECH306L_TEMP_REG_ADDR   (0x0071)   // frame: 00 71
#define ECH306L_TEMP_REG_QTY    (2)

// ====== Modbus CRC16 (poly 0xA001) ======
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

struct ech306l_handle_t {
    ech306l_cfg_t cfg;
};

static esp_err_t rs485_uart_setup(const ech306l_cfg_t *cfg)
{
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = cfg->parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_num, 1024, 1024, 0, NULL, 0), TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_num, &uart_cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, cfg->rts_gpio, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    // RS485 half duplex: dùng RTS để điều khiển DE/RE
    ESP_RETURN_ON_ERROR(uart_set_mode(cfg->uart_num, UART_MODE_RS485_HALF_DUPLEX), TAG, "uart_set_mode");

    // Tuỳ board: có thể cần set idle time / rx timeout
    uart_set_rx_timeout(cfg->uart_num, 3);

    return ESP_OK;
}

esp_err_t ech306l_init(const ech306l_cfg_t *cfg, ech306l_handle_t *out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;

    ech306l_handle_t h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;

    h->cfg = *cfg;
    if (h->cfg.reg_addr == 0) h->cfg.reg_addr = ECH306L_DEFAULT_REG_ADDR;
    if (h->cfg.reg_qty  == 0) h->cfg.reg_qty  = ECH306L_DEFAULT_REG_QTY;
    if (h->cfg.timeout_ms == 0) h->cfg.timeout_ms = 300;

    ESP_RETURN_ON_ERROR(rs485_uart_setup(&h->cfg), TAG, "rs485_uart_setup");
    *out = h;
    return ESP_OK;
}

esp_err_t ech306l_deinit(ech306l_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    free(h);
    return ESP_OK;
}

/**
 * @brief Gửi request 01 03 [addr_hi addr_lo] [qty_hi qty_lo] [crc_lo crc_hi]
 */
static esp_err_t modbus_send_read_holding(ech306l_handle_t h, uint16_t addr, uint16_t qty)
{
    uint8_t req[8];
    req[0] = h->cfg.slave_id;
    req[1] = 0x03;
    req[2] = (uint8_t)(addr >> 8);
    req[3] = (uint8_t)(addr & 0xFF);
    req[4] = (uint8_t)(qty >> 8);
    req[5] = (uint8_t)(qty & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);       // CRC Lo
    req[7] = (uint8_t)(crc >> 8);         // CRC Hi

    uart_flush_input(h->cfg.uart_num);
    int w = uart_write_bytes(h->cfg.uart_num, (const char*)req, sizeof(req));
    if (w != (int)sizeof(req)) return ESP_FAIL;

    ESP_RETURN_ON_ERROR(uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50)), TAG, "wait_tx_done");
    return ESP_OK;
}

/**
 * @brief Nhận response:
 *  [id][0x03][byte_count][data...][crc_lo][crc_hi]
 *  Với qty=2 => byte_count=4, tổng = 3+4+2=9 bytes
 */
static esp_err_t modbus_recv_response(ech306l_handle_t h, uint16_t qty, uint8_t *data_out, uint16_t data_out_len)
{
    const uint16_t byte_count = qty * 2;
    const uint16_t resp_len = 3 + byte_count + 2;

    if (data_out_len < byte_count) return ESP_ERR_INVALID_SIZE;

    uint8_t resp[64];
    if (resp_len > sizeof(resp)) return ESP_ERR_INVALID_SIZE;

    int r = uart_read_bytes(h->cfg.uart_num, resp, resp_len, pdMS_TO_TICKS(h->cfg.timeout_ms));
    if (r != resp_len) {
        ESP_LOGW(TAG, "RX len mismatch got=%d expect=%u", r, resp_len);
        return ESP_ERR_TIMEOUT;
    }

    // Check CRC
    uint16_t crc_calc = modbus_crc16(resp, resp_len - 2);
    uint16_t crc_rx   = (uint16_t)resp[resp_len - 2] | ((uint16_t)resp[resp_len - 1] << 8);
    if (crc_calc != crc_rx) {
        ESP_LOGW(TAG, "CRC mismatch calc=0x%04X rx=0x%04X", crc_calc, crc_rx);
        return ESP_ERR_INVALID_CRC;
    }

    // Basic checks
    if (resp[0] != h->cfg.slave_id) return ESP_ERR_INVALID_RESPONSE;
    if (resp[1] & 0x80) {
        // exception: [id][func|0x80][excode][crc]
        ESP_LOGW(TAG, "Modbus exception code=0x%02X", resp[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (resp[1] != 0x03) return ESP_ERR_INVALID_RESPONSE;
    if (resp[2] != byte_count) return ESP_ERR_INVALID_RESPONSE;

    memcpy(data_out, &resp[3], byte_count);
    return ESP_OK;
}

esp_err_t ech306l_read_raw_u16x2(ech306l_handle_t h, uint16_t *reg0, uint16_t *reg1)
{
    if (!h || !reg0 || !reg1) return ESP_ERR_INVALID_ARG;
    if (h->cfg.reg_qty != 2) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(modbus_send_read_holding(h, h->cfg.reg_addr, h->cfg.reg_qty), TAG, "send");
    uint8_t data[4] = {0};
    ESP_RETURN_ON_ERROR(modbus_recv_response(h, h->cfg.reg_qty, data, sizeof(data)), TAG, "recv");

    // Modbus register data is big-endian per register: [hi][lo]
    *reg0 = ((uint16_t)data[0] << 8) | data[1];
    *reg1 = ((uint16_t)data[2] << 8) | data[3];
    return ESP_OK;
}

esp_err_t ech306l_read_distance_cm(ech306l_handle_t h, float *distance_cm)
{
    if (!h || !distance_cm) return ESP_ERR_INVALID_ARG;

    if (h->cfg.reg_qty == 1) {
        ESP_RETURN_ON_ERROR(modbus_send_read_holding(h, h->cfg.reg_addr, 1), TAG, "send");
        uint8_t data[2];
        ESP_RETURN_ON_ERROR(modbus_recv_response(h, 1, data, sizeof(data)), TAG, "recv");
        uint16_t v = ((uint16_t)data[0] << 8) | data[1];
        *distance_cm = (float)v;
        return ESP_OK;
    }

    if (h->cfg.reg_qty == 2) {
        uint16_t r0, r1;
        ESP_RETURN_ON_ERROR(ech306l_read_raw_u16x2(h, &r0, &r1), TAG, "raw");

        // Parse 2 regs -> uint32: r0 = high word, r1 = low word
        uint32_t u32 = ((uint32_t)r0 << 16) | (uint32_t)r1;

        // Bạn nói đơn vị cm => coi như integer cm.
        // Nếu thực tế là 0.1cm / 0.01m... thì đổi scale ở đây.
        *distance_cm = (float)u32;

        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ech306l_read_temperature_raw_u16x2(ech306l_handle_t h, uint16_t *reg0, uint16_t *reg1)
{
    if (!h || !reg0 || !reg1) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(modbus_send_read_holding(h, ECH306L_TEMP_REG_ADDR, ECH306L_TEMP_REG_QTY), TAG, "send");
    uint8_t data[4] = {0};
    ESP_RETURN_ON_ERROR(modbus_recv_response(h, ECH306L_TEMP_REG_QTY, data, sizeof(data)), TAG, "recv");

    *reg0 = ((uint16_t)data[0] << 8) | data[1];
    *reg1 = ((uint16_t)data[2] << 8) | data[3];
    return ESP_OK;
}

esp_err_t ech306l_read_temperature_c(ech306l_handle_t h, float *temp_c)
{
    if (!h || !temp_c) return ESP_ERR_INVALID_ARG;

    uint16_t r0, r1;
    ESP_RETURN_ON_ERROR(ech306l_read_temperature_raw_u16x2(h, &r0, &r1), TAG, "raw");

    // Parse mặc định giống distance: 2 regs -> uint32
    uint32_t u32 = ((uint32_t)r0 << 16) | (uint32_t)r1;

    // *** CHỖ NÀY TUỲ DATASHEET ***
    // Nhiều cảm biến trả nhiệt độ dạng:
    //  - integer °C
    //  - hoặc 0.1°C (x10)
    //  - hoặc 0.01°C (x100)
    //
    // Tạm thời mình để integer °C:
    *temp_c = (float)u32/1000;

    return ESP_OK;
}