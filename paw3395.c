/*
  PAW3395 driver (datasheet-aligned) with weak submit_mouse_report stub.

  - Provides a weak default implementation of submit_mouse_report to avoid
    linker errors when main does not define it.
  - If main.c provides submit_mouse_report (strong symbol), that one will be used.
*/

#include "paw3395.h"
#include "pins.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdint.h>

static const char *TAG = "paw3395_drv";
static spi_device_handle_t s_spi = NULL;
static paw3395_config_t s_cfg;

/* Registers */
#define REG_POWER_UP_RESET 0x3A
#define REG_MOTION         0x02
#define REG_DELTA_X        0x03
#define REG_DELTA_Y        0x04

/* Weak stub for submit_mouse_report.
   If the application (main.c) defines submit_mouse_report, that strong symbol
   will override this weak stub. This prevents link errors when the app does not
   provide a report submission implementation (useful during debugging).
*/
void submit_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel) __attribute__((weak));
void submit_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    (void)buttons;
    (void)dx;
    (void)dy;
    (void)wheel;
    // weak stub: do nothing
}

/* Low-level register write: write one or more bytes */
static esp_err_t paw_write_regs(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_spi) return ESP_ERR_INVALID_STATE;

    size_t tx_len = 1 + len;
    uint8_t tx[16];
    if (tx_len > sizeof(tx)) return ESP_ERR_INVALID_ARG;

    tx[0] = reg | 0x80; // write command
    for (size_t i = 0; i < len; i++) tx[1 + i] = data[i];

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = tx_len * 8;
    t.tx_buffer = tx;
    return spi_device_transmit(s_spi, &t);
}

/* Low-level register read: read len bytes starting at reg */
static esp_err_t paw_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    if (!s_spi) return ESP_ERR_INVALID_STATE;
    if (len + 1 > 16) return ESP_ERR_INVALID_ARG;

    uint8_t tx[16];
    uint8_t rx[16];
    memset(tx, 0x00, sizeof(tx));
    memset(rx, 0x00, sizeof(rx));

    tx[0] = reg; // read command
    for (size_t i = 1; i <= len; i++) tx[i] = 0x00;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret == ESP_OK) {
        for (size_t i = 0; i < len; i++) out[i] = rx[1 + i];
    }
    return ret;
}

static esp_err_t paw_read_reg(uint8_t reg, uint8_t *val) { return paw_read_regs(reg, val, 1); }
static esp_err_t paw_write_reg(uint8_t reg, uint8_t val) { return paw_write_regs(reg, &val, 1); }

esp_err_t paw3395_read_motion(int8_t *dx, int8_t *dy, uint8_t *buttons)
{
    if (!s_spi) return ESP_ERR_INVALID_STATE;

    uint8_t motion = 0;
    esp_err_t r = paw_read_reg(REG_MOTION, &motion);
    if (r != ESP_OK) return r;
    if ((motion & 0x80) == 0 && motion == 0) return ESP_ERR_NOT_FOUND;

    uint8_t x = 0, y = 0;
    r = paw_read_reg(REG_DELTA_X, &x);
    if (r != ESP_OK) return r;
    r = paw_read_reg(REG_DELTA_Y, &y);
    if (r != ESP_OK) return r;

    *dx = (int8_t)x;
    *dy = (int8_t)y;
    *buttons = 0;
    return ESP_OK;
}

static void poll_task(void *arg)
{
    const TickType_t normal_delay = pdMS_TO_TICKS(1000 / MOUSE_REPORT_RATE_DEFAULT);
    const TickType_t no_device_delay = pdMS_TO_TICKS(500);

    while (1) {
        if (!s_spi) {
            vTaskDelay(no_device_delay);
            continue;
        }

        int8_t dx = 0, dy = 0;
        uint8_t buttons = 0;
        esp_err_t r = paw3395_read_motion(&dx, &dy, &buttons);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "motion dx=%d dy=%d", dx, dy);
            submit_mouse_report(buttons, dx, dy, 0);
            vTaskDelay(normal_delay);
        } else if (r == ESP_ERR_NOT_FOUND) {
            vTaskDelay(no_device_delay);
        } else {
            ESP_LOGW(TAG, "paw3395 read error: %s", esp_err_to_name(r));
            vTaskDelay(no_device_delay);
        }
    }
}

esp_err_t paw3395_init(const paw3395_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;

    spi_bus_config_t buscfg = {
        .mosi_io_num = s_cfg.pin_mosi,
        .miso_io_num = s_cfg.pin_miso,
        .sclk_io_num = s_cfg.pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    esp_err_t ret = spi_bus_initialize(s_cfg.spi_host, &buscfg, 1);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000,
        .mode = 3,
        .spics_io_num = s_cfg.pin_cs,
        .queue_size = 3,
    };

    ret = spi_bus_add_device(s_cfg.spi_host, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "spi_bus_add_device failed: %s (device disabled)", esp_err_to_name(ret));
        s_spi = NULL;
        return ret;
    }

    gpio_reset_pin(s_cfg.pin_reset);
    gpio_set_direction(s_cfg.pin_reset, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cfg.pin_reset, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(s_cfg.pin_reset, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ret = paw_write_reg(REG_POWER_UP_RESET, 0x5A);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "POWER_UP_RESET write failed: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t probe;
    bool ok = true;
    for (uint8_t r = REG_MOTION; r <= (REG_MOTION + 4); r++) {
        ret = paw_read_reg(r, &probe);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "probe reg 0x%02X failed: %s", r, esp_err_to_name(ret));
            ok = false;
            break;
        } else {
            ESP_LOGD(TAG, "probe reg 0x%02X = 0x%02X", r, probe);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    gpio_reset_pin(s_cfg.pin_int);
    gpio_set_direction(s_cfg.pin_int, GPIO_MODE_INPUT);

    BaseType_t ok_task = xTaskCreate(poll_task, "paw3395_poll", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (ok_task != pdPASS) {
        ESP_LOGE(TAG, "create paw3395 poll task failed");
        if (s_spi) {
            spi_bus_remove_device(s_spi);
            s_spi = NULL;
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "paw3395 initialized (probe %s)", ok ? "OK" : "FAILED");
    return ESP_OK;
}

void paw3395_deinit(void)
{
    if (s_spi) {
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
    }
}