/*
  main.c -- tailored for gdzsjjjjovo-rgb/ESP32_nimble_mouse

  - Defines ble_report_maps and ble_hid_config for print_report_map.c / esp_hidd
  - Implements submit_mouse_report (overrides weak stub in paw3395.c)
  - Provides ble_hid_task_start_up / ble_hid_task_shut_down for esp_hid_gap.c
  - Initializes NVS, esp_hid_gap, esp_hidd and registers callbacks
  - Compatible whether nimble_reconnect.c is present or not (weak stub provided)
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_hidd.h"
#include "esp_hid_common.h"
#include "esp_hid_gap.h"

/* If nimble_reconnect.c was removed, provide a weak no-op so linking still succeeds.
   If nimble_reconnect.c exists, its strong definition will override this. */
void nimble_reconnect_init(void) __attribute__((weak));
void nimble_reconnect_init(void) { /* no-op if absent */ }

/* ble_store_config_init may be provided by NimBLE store util; weak stub if absent */
void ble_store_config_init(void) __attribute__((weak));
void ble_store_config_init(void) { /* no-op if absent */ }

/* Optional helper in print_report_map.c (weak) */
void print_report_map_info(const esp_hid_raw_report_map_t *rm) __attribute__((weak));

#if CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#endif

static const char *TAG = "main_nimble_mouse";

/* Device name as used in this repo (keeps prior behavior) */
static const char device_name_str[] = "ESP HID Mouse";

/* HID report map (mouse). REPORT_ID placed after Collection(Application) */
static const uint8_t mouse_report_map[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x02,       /* Usage (Mouse) */
    0xA1, 0x01,       /* Collection (Application) */

    0x85, 0x01,       /* REPORT_ID (1) - placed here so parser accepts it */

      0x09, 0x01,     /*   Usage (Pointer) */
      0xA1, 0x00,     /*   Collection (Physical) */
        0x05, 0x09,   /*     Usage Page (Buttons) */
        0x19, 0x01,   /*     Usage Minimum (01) */
        0x29, 0x03,   /*     Usage Maximum (03) */
        0x15, 0x00,   /*     Logical Minimum (0) */
        0x25, 0x01,   /*     Logical Maximum (1) */
        0x95, 0x03,   /*     Report Count (3) */
        0x75, 0x01,   /*     Report Size (1) */
        0x81, 0x02,   /*     Input (Data,Var,Abs) */
        0x95, 0x01,   /*     Report Count (1) */
        0x75, 0x05,   /*     Report Size (5) */
        0x81, 0x03,   /*     Input (Cnst,Var,Abs) */

        0x05, 0x01,   /*     Usage Page (Generic Desktop) */
        0x09, 0x30,   /*     Usage (X) */
        0x09, 0x31,   /*     Usage (Y) */
        0x09, 0x38,   /*     Usage (Wheel) */
        0x15, 0x81,   /*     Logical Minimum (-127) */
        0x25, 0x7F,   /*     Logical Maximum (127) */
        0x75, 0x08,   /*     Report Size (8) */
        0x95, 0x03,   /*     Report Count (3) */
        0x81, 0x06,   /*     Input (Data,Var,Rel) */
      0xC0,
    0xC0
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    { .data = (uint8_t *)mouse_report_map, .len = sizeof(mouse_report_map) }
};

static const esp_hid_device_config_t ble_hid_config = {
    .vendor_id         = 0x16C0,
    .product_id        = 0x05DF,
    .version           = 0x0100,
    .device_name       = device_name_str,
    .manufacturer_name = "Espressif",
    .serial_number     = "0001",
    .report_maps       = ble_report_maps,
    .report_maps_len   = sizeof(ble_report_maps) / sizeof(ble_report_maps[0])
};

/* Runtime state shared with other modules */
static esp_hidd_dev_t *s_hid_dev = NULL;
static volatile bool s_hid_connected = false;
static TaskHandle_t s_demo_task = NULL;

/* HID demo task (sends reports with report_id = 1) */
static void hid_mouse_demo_task(void *pv)
{
    (void)pv;
    uint8_t report[4];
    while (1) {
        if (s_hid_connected && s_hid_dev) {
            report[0] = 0;    /* buttons */
            report[1] = 10;   /* dx */
            report[2] = 0;    /* dy */
            report[3] = 0;    /* wheel */
            esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 1, 0, report, sizeof(report)); /* report_id=1 */
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_hidd_dev_input_set failed: %d", err);
            } else {
                ESP_LOGI(TAG, "Sent mouse report id=1 dx=%d", report[1]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Start/stop helpers expected by esp_hid_gap.c (non-static so other TU can call) */
void ble_hid_task_start_up(void)
{
    if (s_demo_task) return;
    if (xTaskCreate(hid_mouse_demo_task, "hid_demo", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &s_demo_task) != pdPASS) {
        ESP_LOGW(TAG, "Failed to create hid demo task");
        s_demo_task = NULL;
    } else {
        ESP_LOGI(TAG, "HID demo started");
    }
}

void ble_hid_task_shut_down(void)
{
    if (!s_demo_task) return;
    vTaskDelete(s_demo_task);
    s_demo_task = NULL;
    ESP_LOGI(TAG, "HID demo stopped");
}

/* Strong submit_mouse_report used by paw3395.c to deliver sensor motion */
void submit_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    if (!s_hid_dev || !s_hid_connected) return;

    uint8_t report[4];
    report[0] = buttons;
    report[1] = (uint8_t)dx;
    report[2] = (uint8_t)dy;
    report[3] = (uint8_t)wheel;

    esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 1, 0, report, sizeof(report)); /* report_id = 1 */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "submit_mouse_report: esp_hidd_dev_input_set failed: %d", err);
    }
}

/* esp_hidd event callback */
static void hidd_event_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args; (void)base;
    esp_hidd_event_t ev = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (ev) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_START_EVENT");
        ESP_LOGI(TAG, "HID device_name='%s'", ble_hid_config.device_name);
        ESP_LOGI(TAG, "report_maps_len=%d", (int)ble_hid_config.report_maps_len);
        if (print_report_map_info) {
            print_report_map_info(&ble_report_maps[0]);
        } else {
            ESP_LOG_BUFFER_HEX(TAG, ble_report_maps[0].data, (ble_report_maps[0].len > 128) ? 128 : ble_report_maps[0].len);
        }
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONNECT_EVENT");
        s_hid_connected = true;
        s_hid_dev = param->connect.dev;
        /* Do not start demo immediately; let pairing/encryption/subscribe complete (nimble_reconnect or gap handler will call ble_hid_task_start_up) */
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_DISCONNECT_EVENT reason=%d", param->disconnect.reason);
        s_hid_connected = false;
        ble_hid_task_shut_down();
        /* Re-advertise using esp_hid helper */
        esp_hid_ble_gap_adv_start();
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_OUTPUT_EVENT len=%d", param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;

    default:
        ESP_LOGI(TAG, "HIDD event id=%d", (int)ev);
        break;
    }
}

#if CONFIG_BT_NIMBLE_ENABLED
/* NimBLE host callbacks (lightweight) */
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "ble_app_on_sync: host synced");
    /* esp_hid_gap / esp_nimble_enable in repo will handle advertising start */
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "ble_app_on_reset: reason=%d", reason);
}
#endif

void app_main(void)
{
    esp_err_t err;

    /* Initialize NVS */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Helpful log levels */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("ble_sm", ESP_LOG_DEBUG);
    esp_log_level_set("ble_store", ESP_LOG_DEBUG);
    esp_log_level_set("ble_hs", ESP_LOG_INFO);
    esp_log_level_set("NimBLE", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Starting HID application");

    /* Optional init helpers (weak no-op if absent) */
    nimble_reconnect_init();
    ble_store_config_init();

#if CONFIG_BT_NIMBLE_ENABLED
    /* register NimBLE callbacks if host is started by repo's shim */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
#endif

    /* Initialize controller + GAP via esp_hid_gap helper (repo provides esp_hid_gap.c) */
    err = esp_hid_gap_init(HID_DEV_MODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", err);
        return;
    }

    /* Prepare advertisement data (device name) */
    err = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", err);
        return;
    }

    /* Initialize HID device and register callback */
    err = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, hidd_event_cb, &s_hid_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", err);
        return;
    }

    ESP_LOGI(TAG, "app_main finished");
}