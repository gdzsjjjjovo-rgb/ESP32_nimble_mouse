/* main.c
 * ESP BLE HID mouse — mouse-only, manual demo via monitor stdin.
 * Controls:
 *   's' start periodic demo (move right every 2s)
 *   'x' stop demo
 *   'e' emit one single right-move event
 *
 * Build and flash with ESP-IDF.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_event.h"

#include "driver/uart.h" /* only for UART definitions if needed */

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "esp_nimble_hci.h"

/* Forward declaration */
void ble_store_config_init(void);

static const char *TAG = "esp_ble_hid_mouse";

/* Mouse report map (no report ID) */
static const unsigned char mouseReportMap[] = {
    0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,0xA1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,0x95,0x01,0x75,0x05,0x81,0x03,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x03,
    0x81,0x06,0xC0,0xC0
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    { .data = mouseReportMap, .len = sizeof(mouseReportMap) }
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "ESP Mouse",
    .manufacturer_name  = "Espressif",
    .serial_number      = "0001",
    .report_maps        = ble_report_maps,
    .report_maps_len    = 1
};

static esp_hidd_dev_t *s_hid_dev = NULL;
static TaskHandle_t s_demo_task = NULL;

/* send mouse report helper: [buttons, dx, dy, wheel] */
static void send_mouse_report(int8_t dx, int8_t dy, uint8_t buttons)
{
    uint8_t buf[4];
    buf[0] = buttons;
    buf[1] = (uint8_t)dx;
    buf[2] = (uint8_t)dy;
    buf[3] = 0x00;
    if (s_hid_dev) {
        int rc = esp_hidd_dev_input_set(s_hid_dev, 0, 0, buf, sizeof(buf));
        ESP_LOGI(TAG, "sent rc=%d data=%02X %02X %02X %02X", rc, buf[0], buf[1], buf[2], buf[3]);
    } else {
        ESP_LOGW(TAG, "hid_dev NULL, cannot send");
    }
}

/* demo task: periodic right move */
static void hid_mouse_demo_task(void *pv)
{
    ESP_LOGI(TAG, "mouse demo started");
    for (;;) {
        send_mouse_report(10, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        send_mouse_report(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* safe start / stop demo functions (exported symbol expected by esp_hid_gap) */
void ble_hid_task_start_up(void)
{
    if (s_demo_task) return;
    xTaskCreate(hid_mouse_demo_task, "hid_mouse_demo", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &s_demo_task);
    ESP_LOGI(TAG, "demo created");
}

void ble_hid_task_shut_down(void)
{
    if (!s_demo_task) return;
    vTaskDelete(s_demo_task);
    s_demo_task = NULL;
    ESP_LOGI(TAG, "demo stopped");
}

/* stdin control task — read from idf.py monitor stdin */
static void stdin_control_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG, "stdin control ready: 's' start, 'x' stop, 'e' emit once");
    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        char ch = (char)c;
        if (ch == 's') {
            ble_hid_task_start_up();
        } else if (ch == 'x') {
            ble_hid_task_shut_down();
        } else if (ch == 'e') {
            send_mouse_report(10, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            send_mouse_report(0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* esp_hidd event callback */
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "START - advertising");
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "CONNECT");
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "DISCONNECT");
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "PROTOCOL MODE map_index=%u mode=%s",
                 param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "CONTROL control=%d (auto-start disabled)", param->control.control);
        if (!param->control.control) ble_hid_task_shut_down();
        break;
    default:
        ESP_LOGI(TAG, "HIDD event id=%d", (int)event);
        break;
    }
}

/* NimBLE host task */
void ble_hid_device_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* app_main */
void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Optional debug logs */
    esp_log_level_set("ble_hs", ESP_LOG_INFO);
    esp_log_level_set("ble_hci", ESP_LOG_INFO);
    esp_log_level_set("ble_sm", ESP_LOG_INFO);
    esp_log_level_set("NimBLE", ESP_LOG_INFO);

    ESP_LOGI(TAG, "init HID GAP");
    ret = esp_hid_gap_init(HID_DEV_MODE);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", ret); return; }

    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", ret); return; }

    ret = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_hid_dev);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", ret); return; }

    /* NimBLE store for bonding */
    ble_store_config_init();

    /* SM config for Just Works */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret); return; }

    /* Create stdin control task (use idf.py monitor keyboard) */
    xTaskCreate(stdin_control_task, "stdin_ctrl", 3 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

    ESP_LOGI(TAG, "app_main done");
}