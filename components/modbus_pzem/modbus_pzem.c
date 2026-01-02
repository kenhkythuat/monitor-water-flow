#include "modbus_pzem.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


static const char *TAG = "modbus_pzem";

// ===================== CRC16 MODBUS =====================
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ===================== CONFIG / STATE =====================
static modbus_pzem_cfg_t s_cfg;
static bool s_inited = false;

#define MAX_SLAVES 16
static stm32_meter_t s_meters[MAX_SLAVES];
static uint8_t s_meter_count = 0;

static SemaphoreHandle_t s_lock;

// ===================== UART RS485 INIT =====================
static esp_err_t rs485_uart_init(const rs485_modbus_cfg_t *c)
{
    uart_config_t uc = {
        .baud_rate = c->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(c->uart_num, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(c->uart_num, &uc));

    // Map DE pin vào RTS để dùng RS485 half duplex mode (ESP-IDF tự toggle RTS)
    ESP_ERROR_CHECK(uart_set_pin(c->uart_num, c->tx_gpio, c->rx_gpio, c->de_gpio, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(c->uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Clean
    uart_flush_input(c->uart_num);
    ESP_LOGI(TAG, "RS485 UART init ok: uart=%d tx=%d rx=%d de(rts)=%d baud=%d",
             c->uart_num, c->tx_gpio, c->rx_gpio, c->de_gpio, c->baudrate);
    return ESP_OK;
}

// ===================== MODBUS READ INPUT REGS =====================
// Read Input Registers (0x04): response: [id][fc][bc][data..][crc_lo][crc_hi]
static esp_err_t modbus_read_input_regs(uint8_t slave_id, uint16_t start, uint16_t qty,
                                       uint16_t *out_regs, uint32_t timeout_ms)
{
    if (!out_regs || qty == 0) return ESP_ERR_INVALID_ARG;

    uint8_t req[8];
    req[0] = slave_id;
    req[1] = 0x04;
    req[2] = (start >> 8) & 0xFF;
    req[3] = (start >> 0) & 0xFF;
    req[4] = (qty   >> 8) & 0xFF;
    req[5] = (qty   >> 0) & 0xFF;

    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;         // CRC Lo
    req[7] = (crc >> 8) & 0xFF;  // CRC Hi

    const int uartn = s_cfg.rs485.uart_num;

    uart_flush_input(uartn);

    // ===== DEBUG TX =====
    ESP_LOGI(TAG, "MB TX slave=0x%02X fc=0x%02X start=0x%04X qty=%u",
             slave_id, req[1], start, qty);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, req, sizeof(req), ESP_LOG_INFO);

    int w = uart_write_bytes(uartn, (const char*)req, sizeof(req));
    ESP_ERROR_CHECK(uart_wait_tx_done(uartn, pdMS_TO_TICKS(200)));

    if (w != (int)sizeof(req)) return ESP_FAIL;

    // Đọc header 3 bytes
    uint8_t hdr[3] = {0};
    int r = uart_read_bytes(uartn, hdr, 3, pdMS_TO_TICKS(timeout_ms));
    if (r != 3) {
        ESP_LOGW(TAG, "MB RX timeout hdr: got=%d/3 slave=0x%02X start=0x%04X", r, slave_id, start);
        if (r > 0) ESP_LOG_BUFFER_HEX_LEVEL(TAG, hdr, r, ESP_LOG_WARN);
        return ESP_ERR_TIMEOUT;
    }

    // ===== DEBUG RX HDR =====
    ESP_LOGI(TAG, "MB RX hdr:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, hdr, 3, ESP_LOG_INFO);

    if (hdr[0] != slave_id) return ESP_FAIL;

    if (hdr[1] & 0x80) {
        uint8_t ex_rest[3] = {0};
        uart_read_bytes(uartn, ex_rest, 3, pdMS_TO_TICKS(50));
        ESP_LOGW(TAG, "MB exception slave=0x%02X fc=0x%02X ex=0x%02X",
                 slave_id, hdr[1], hdr[2]);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, ex_rest, 3, ESP_LOG_WARN);
        return ESP_FAIL;
    }
    if (hdr[1] != 0x04) return ESP_FAIL;

    uint8_t byte_count = hdr[2];
    if (byte_count != (uint8_t)(qty * 2)) {
        ESP_LOGW(TAG, "ByteCount mismatch: got=%u expected=%u", byte_count, (unsigned)(qty*2));
        return ESP_FAIL;
    }

    // data + crc
    uint8_t buf[256] = {0};
    if (byte_count + 2 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;

    r = uart_read_bytes(uartn, buf, byte_count + 2, pdMS_TO_TICKS(timeout_ms));
    if (r != (int)(byte_count + 2)) {
        ESP_LOGW(TAG, "MB RX timeout body: got=%d/%d", r, (int)(byte_count+2));
        if (r > 0) ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, r, ESP_LOG_WARN);
        return ESP_ERR_TIMEOUT;
    }

    // ===== DEBUG RX FULL FRAME =====
    uint8_t frame[300];
    memcpy(frame, hdr, 3);
    memcpy(frame + 3, buf, byte_count + 2);
    ESP_LOGI(TAG, "MB RX frame:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, 3 + byte_count + 2, ESP_LOG_INFO);

    // CRC check
    uint16_t crc_calc = modbus_crc16(frame, 3 + byte_count);
    uint16_t crc_rx   = (uint16_t)buf[byte_count] | ((uint16_t)buf[byte_count + 1] << 8);
    if (crc_calc != crc_rx) {
        ESP_LOGW(TAG, "CRC fail: calc=0x%04X rx=0x%04X", crc_calc, crc_rx);
        return ESP_ERR_INVALID_CRC;
    }

    // parse regs
    for (int i = 0; i < qty; i++) {
        uint8_t hi = buf[i*2 + 0];
        uint8_t lo = buf[i*2 + 1];
        out_regs[i] = ((uint16_t)hi << 8) | lo;
    }

    return ESP_OK;
}


// ===================== PZEM PARSE =====================
static void parse_pzem_regs(const uint16_t *r, pzem_data_t *o)
{
    // r[0]=V*10
    uint16_t v10 = r[0];

    uint32_t i_raw = ((uint32_t)r[2] << 16) | r[1]; // low then high
    uint32_t p_raw = ((uint32_t)r[4] << 16) | r[3];
    uint32_t e_wh  = ((uint32_t)r[6] << 16) | r[5];

    uint16_t f10 = r[7];
    uint16_t pf100 = r[8];
    uint16_t st = r[9];

    o->voltage_v = (float)v10 / 10.0f;
    o->current_a = (float)i_raw / 1000.0f;
    o->power_w   = (float)p_raw / 10.0f;
    o->energy_wh = (float)e_wh;
    o->freq_hz   = (float)f10 / 10.0f;
    o->pf        = (float)pf100 / 100.0f;

    o->status = st;
    o->updated = (st & (1u << 0)) != 0;
    o->crc_fail = (st & (1u << 1)) != 0;
}

static void dump_pzem_regs(const char *tag, uint8_t slave_id, const char *name,
                           uint16_t base, const uint16_t *r)
{
    // In raw regs (10 regs)
    ESP_LOGI(tag, "slave=0x%02X %s base=0x%04X raw:", slave_id, name, base);
    ESP_LOGI(tag,
             "  r0=%04X r1=%04X r2=%04X r3=%04X r4=%04X r5=%04X r6=%04X r7=%04X r8=%04X r9=%04X",
             r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9]);

    // Parse theo đúng mapping bạn đưa
    uint16_t v10    = r[0];
    uint32_t i_raw  = ((uint32_t)r[2] << 16) | r[1];
    uint32_t p_raw  = ((uint32_t)r[4] << 16) | r[3];
    uint32_t e_wh   = ((uint32_t)r[6] << 16) | r[5];
    uint16_t f10    = r[7];
    uint16_t pf100  = r[8];
    uint16_t st     = r[9];

    float V  = (float)v10 / 10.0f;
    float I  = (float)i_raw / 1000.0f;
    float P  = (float)p_raw / 10.0f;
    float F  = (float)f10 / 10.0f;
    float PF = (float)pf100 / 100.0f;

    ESP_LOGI(tag,
             "slave=0x%02X %s parsed: V=%.1fV I=%.3fA P=%.1fW E=%luWh F=%.1fHz PF=%.2f status=0x%04X (upd=%d crc_fail=%d)",
             slave_id, name, V, I, P, (unsigned long)e_wh, F, PF, st,
             (st & 0x0001) ? 1 : 0,
             (st & 0x0002) ? 1 : 0);
}


// ===================== POLL ONE SLAVE =====================
static void poll_one_slave(uint8_t slave_id)
{
    uint16_t regs1[10] = {0};
    uint16_t regs2[10] = {0};

    esp_err_t e1 = modbus_read_input_regs(slave_id, 0x0000, 0x000A, regs1, 500);
    vTaskDelay(pdMS_TO_TICKS(s_cfg.inter_request_ms));
    esp_err_t e2 = modbus_read_input_regs(slave_id, 0x0010, 0x000A, regs2, 500);

        // ✅ DEBUG DUMP NGAY SAU KHI ĐỌC
    if (e1 == ESP_OK) dump_pzem_regs(TAG, slave_id, "PZEM1", 0x0000, regs1);
    else ESP_LOGW(TAG, "slave=0x%02X PZEM1 read err: %s", slave_id, esp_err_to_name(e1));

    if (e2 == ESP_OK) dump_pzem_regs(TAG, slave_id, "PZEM2", 0x0010, regs2);
    else ESP_LOGW(TAG, "slave=0x%02X PZEM2 read err: %s", slave_id, esp_err_to_name(e2));

    xSemaphoreTake(s_lock, portMAX_DELAY);

    // find meter slot
    int idx = -1;
    for (int i = 0; i < s_meter_count; i++) {
        if (s_meters[i].slave_id == slave_id) { idx = i; break; }
    }

    if (idx < 0) {
        // should not happen, but safe
        xSemaphoreGive(s_lock);
        return;
    }

    stm32_meter_t *m = &s_meters[idx];

    if (e1 == ESP_OK && e2 == ESP_OK) {
        parse_pzem_regs(regs1, &m->pzem1);
        parse_pzem_regs(regs2, &m->pzem2);
        m->last_ok_ms = now_ms();
        m->ok_count++;
    } else {
        m->last_err_ms = now_ms();
        m->err_count++;
        if (e1 != ESP_OK) ESP_LOGW(TAG, "slave=0x%02X PZEM1 read err: %s", slave_id, esp_err_to_name(e1));
        if (e2 != ESP_OK) ESP_LOGW(TAG, "slave=0x%02X PZEM2 read err: %s", slave_id, esp_err_to_name(e2));
    }

    xSemaphoreGive(s_lock);
}

// ===================== TASK =====================
static void modbus_pzem_task(void *arg)
{
    (void)arg;

    while (1) {
        // Chờ có internet (WiFi hoặc ETH)
        xEventGroupWaitBits(s_cfg.net_event_group,
                            s_cfg.online_bits,
                            pdFALSE, pdFALSE,
                            portMAX_DELAY);

        // Poll lần lượt các slave
        for (int i = 0; i < s_cfg.slave_count; i++) {
            // Nếu trong lúc poll mất internet thì break để chờ lại
            EventBits_t b = xEventGroupGetBits(s_cfg.net_event_group);
            if ((b & s_cfg.online_bits) == 0) break;

            poll_one_slave(s_cfg.slave_ids[i]);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.poll_period_ms));
    }
}

// ===================== PUBLIC API =====================
esp_err_t modbus_pzem_init(const modbus_pzem_cfg_t *cfg)
{
    if (!cfg || !cfg->slave_ids || cfg->slave_count == 0) return ESP_ERR_INVALID_ARG;
    if (cfg->slave_count > MAX_SLAVES) return ESP_ERR_INVALID_ARG;
    if (!cfg->net_event_group || cfg->online_bits == 0) return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;

    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(rs485_uart_init(&s_cfg.rs485));

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_meter_count = cfg->slave_count;
    for (int i = 0; i < s_meter_count; i++) {
        memset(&s_meters[i], 0, sizeof(s_meters[i]));
        s_meters[i].slave_id = cfg->slave_ids[i];
    }
    xSemaphoreGive(s_lock);

    s_inited = true;
    ESP_LOGI(TAG, "modbus_pzem init ok, slaves=%u", (unsigned)s_meter_count);
    return ESP_OK;
}

esp_err_t modbus_pzem_start(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreate(modbus_pzem_task, "modbus_pzem_task", 4096, NULL, 6, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

uint8_t modbus_pzem_get_meter_count(void)
{
    return s_meter_count;
}

bool modbus_pzem_get_meter(uint8_t index, stm32_meter_t *out)
{
    if (!out || index >= s_meter_count) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_meters[index];
    xSemaphoreGive(s_lock);
    return true;
}
