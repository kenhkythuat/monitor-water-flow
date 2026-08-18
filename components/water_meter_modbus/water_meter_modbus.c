#include "water_meter_modbus.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "water_meter";

#define WATER_TOTAL_REG_ADDR 0x0000
#define WATER_TOTAL_REG_QTY  0x0002
#define WATER_TOTAL_FUNC     0x03
#define WATER_TOTAL_RSP_LEN  9
#define MODBUS_EXCEPTION_BIT 0x80

struct water_meter_handle_t {
    water_meter_cfg_t cfg;
    bool have_previous;
    uint64_t previous_total_liter;
    uint32_t previous_ts_ms;
};

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

uint16_t water_meter_modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint32_t water_meter_parse_raw_total(const uint8_t data[4])
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

uint64_t water_meter_raw_to_liter(uint32_t raw_total_volume)
{
    return (uint64_t)raw_total_volume * 10ULL;
}

const char *water_meter_status_name(water_meter_status_t status)
{
    switch (status) {
    case WATER_METER_OK: return "ok";
    case WATER_METER_ERR_TIMEOUT: return "timeout";
    case WATER_METER_ERR_CRC: return "crc_error";
    case WATER_METER_ERR_SLAVE_ID: return "slave_id_error";
    case WATER_METER_ERR_FUNCTION: return "function_error";
    case WATER_METER_ERR_BYTE_COUNT: return "byte_count_error";
    case WATER_METER_ERR_EXCEPTION: return "modbus_exception";
    case WATER_METER_ERR_UART: return "uart_error";
    case WATER_METER_ERR_INVALID_ARG: return "invalid_arg";
    default: return "unknown";
    }
}

static esp_err_t setup_uart(const water_meter_cfg_t *cfg)
{
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = cfg->parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg->uart_num, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, cfg->rts_gpio, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_set_mode(cfg->uart_num, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) return err;

    uart_set_rx_timeout(cfg->uart_num, 3);
    uart_flush_input(cfg->uart_num);

    ESP_LOGI(TAG, "RS485 UART init ok: uart=%d tx=%d rx=%d de(rts)=%d baud=%d slave=0x%02X",
             cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, cfg->rts_gpio, cfg->baudrate, cfg->slave_id);
    return ESP_OK;
}

esp_err_t water_meter_init(const water_meter_cfg_t *cfg, water_meter_handle_t *out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;
    if (cfg->slave_id == 0 || cfg->slave_id > 247) return ESP_ERR_INVALID_ARG;

    water_meter_handle_t h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;

    h->cfg = *cfg;
    if (h->cfg.baudrate == 0) h->cfg.baudrate = 9600;
    if (h->cfg.timeout_ms == 0) h->cfg.timeout_ms = 300;
    if (h->cfg.retries == 0) h->cfg.retries = 2;

    esp_err_t err = setup_uart(&h->cfg);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    *out = h;
    return ESP_OK;
}

esp_err_t water_meter_deinit(water_meter_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    free(h);
    return ESP_OK;
}

static esp_err_t send_read_total_request(water_meter_handle_t h)
{
    uint8_t req[8] = {
        h->cfg.slave_id,
        WATER_TOTAL_FUNC,
        (uint8_t)(WATER_TOTAL_REG_ADDR >> 8),
        (uint8_t)(WATER_TOTAL_REG_ADDR & 0xFF),
        (uint8_t)(WATER_TOTAL_REG_QTY >> 8),
        (uint8_t)(WATER_TOTAL_REG_QTY & 0xFF),
        0,
        0,
    };

    uint16_t crc = water_meter_modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    uart_flush_input(h->cfg.uart_num);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, req, sizeof(req), ESP_LOG_DEBUG);

    int written = uart_write_bytes(h->cfg.uart_num, (const char *)req, sizeof(req));
    if (written != (int)sizeof(req)) {
        ESP_LOGW(TAG, "UART write failed: wrote=%d expected=%u", written, (unsigned)sizeof(req));
        return ESP_FAIL;
    }

    esp_err_t err = uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "UART wait tx done failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static water_meter_status_t receive_total_response(water_meter_handle_t h, water_meter_data_t *out)
{
    uint8_t resp[WATER_TOTAL_RSP_LEN] = {0};
    int read_len = uart_read_bytes(h->cfg.uart_num, resp, sizeof(resp), pdMS_TO_TICKS(h->cfg.timeout_ms));

    if (read_len == 5 && resp[0] == h->cfg.slave_id && resp[1] == (WATER_TOTAL_FUNC | MODBUS_EXCEPTION_BIT)) {
        uint16_t crc_calc = water_meter_modbus_crc16(resp, 3);
        uint16_t crc_rx = (uint16_t)resp[3] | ((uint16_t)resp[4] << 8);
        if (crc_calc != crc_rx) return WATER_METER_ERR_CRC;
        out->exception_code = resp[2];
        ESP_LOGW(TAG, "Modbus exception: slave=0x%02X code=0x%02X", resp[0], resp[2]);
        return WATER_METER_ERR_EXCEPTION;
    }

    if (read_len != WATER_TOTAL_RSP_LEN) {
        ESP_LOGW(TAG, "RX timeout/length: got=%d expected=%u", read_len, WATER_TOTAL_RSP_LEN);
        if (read_len > 0) ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp, read_len, ESP_LOG_WARN);
        return WATER_METER_ERR_TIMEOUT;
    }

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp, sizeof(resp), ESP_LOG_DEBUG);

    uint16_t crc_calc = water_meter_modbus_crc16(resp, WATER_TOTAL_RSP_LEN - 2);
    uint16_t crc_rx = (uint16_t)resp[7] | ((uint16_t)resp[8] << 8);
    if (crc_calc != crc_rx) {
        ESP_LOGW(TAG, "CRC mismatch calc=0x%04X rx=0x%04X", crc_calc, crc_rx);
        return WATER_METER_ERR_CRC;
    }

    if (resp[0] != h->cfg.slave_id) return WATER_METER_ERR_SLAVE_ID;
    if (resp[1] != WATER_TOTAL_FUNC) return WATER_METER_ERR_FUNCTION;
    if (resp[2] != 0x04) return WATER_METER_ERR_BYTE_COUNT;

    uint32_t raw = water_meter_parse_raw_total(&resp[3]);
    uint64_t liter = water_meter_raw_to_liter(raw);
    uint32_t ts_ms = now_ms();

    out->raw_total_volume = raw;
    out->total_volume_liter = liter;
    out->total_volume_m3 = (float)raw * 0.01f;
    out->flow_valid = false;
    out->flow_liter_per_minute = 0.0f;
    out->ts_ms = ts_ms;
    out->status = WATER_METER_OK;
    out->exception_code = 0;

    if (h->have_previous) {
        uint32_t elapsed_ms = ts_ms - h->previous_ts_ms;
        if (liter >= h->previous_total_liter && elapsed_ms > 0) {
            uint64_t delta_liter = liter - h->previous_total_liter;
            out->flow_liter_per_minute = ((float)delta_liter * 60000.0f) / (float)elapsed_ms;
            out->flow_valid = true;
        } else if (liter < h->previous_total_liter) {
            ESP_LOGW(TAG, "Total volume decreased, reset flow baseline: old=%llu new=%llu",
                     (unsigned long long)h->previous_total_liter,
                     (unsigned long long)liter);
        }
    }

    h->previous_total_liter = liter;
    h->previous_ts_ms = ts_ms;
    h->have_previous = true;

    return WATER_METER_OK;
}

esp_err_t water_meter_read_total(water_meter_handle_t h, water_meter_data_t *out)
{
    if (!h || !out) return ESP_ERR_INVALID_ARG;

    water_meter_data_t candidate = {
        .status = WATER_METER_ERR_TIMEOUT,
    };

    for (uint8_t attempt = 0; attempt <= h->cfg.retries; attempt++) {
        esp_err_t err = send_read_total_request(h);
        if (err != ESP_OK) {
            candidate.status = WATER_METER_ERR_UART;
            ESP_LOGW(TAG, "Read attempt %u UART error: %s", (unsigned)attempt + 1, esp_err_to_name(err));
        } else {
            candidate.status = receive_total_response(h, &candidate);
        }

        if (candidate.status == WATER_METER_OK) {
            *out = candidate;
            ESP_LOGI(TAG, "Total water raw=%lu liter=%llu m3=%.2f flow=%.2f L/min valid=%d",
                     (unsigned long)out->raw_total_volume,
                     (unsigned long long)out->total_volume_liter,
                     out->total_volume_m3,
                     out->flow_liter_per_minute,
                     out->flow_valid ? 1 : 0);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Read attempt %u failed: %s", (unsigned)attempt + 1,
                 water_meter_status_name(candidate.status));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    out->status = candidate.status;
    out->exception_code = candidate.exception_code;
    return ESP_FAIL;
}
