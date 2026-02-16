/*
  main.c

  NimBLE HID mouse example (complete main)
  - Uses NimBLE (CONFIG_BT_NIMBLE_ENABLED = y).
  - Tries to include nimble_reconnect.h if present; otherwise provides a fallback prototype
    so compilation won't fail with "implicit declaration".
  - Make sure nimble_reconnect.c is compiled and linked (add to main/CMakeLists.txt or place it
    where the build system picks it up).
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_hidd.h"
#include "esp_hid_common.h"
#include "esp_hid_gap.h"

/* Prefer to include nimble_reconnect.h; if not available, declare prototype to avoid
   "implicit declaration" compile errors. Ensure nimble_reconnect.c is linked. */
#if defined(__has_include)
#  if __has_include("nimble_reconnect.h")
#    include "nimble_reconnect.h"
#  else
/* Fallback prototype: ensure your nimble_reconnect.c actually defines this function */
extern void nimble_reconnect_init(void);
#  endif
#else
/* Older compilers: try include, otherwise fall back */
#include "nimble_reconnect.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#endif

static const char *TAG = "nimble_hid_main";

/* ---- Minimal HID mouse report map ---- */
static const uint8_t mouse_report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
      0x09, 0x01,     //   Usage (Pointer)
      0xA1, 0x00,     //   Collection (Physical)
        0x05, 0x09,   //     Usage Page (Buttons)
        0x19, 0x01,   //     Usage Minimum (01)
        0x29, 0x03,   //     Usage Maximum (03)
        0x15, 0x00,   //     Logical Minimum (0)
        0x25, 0x01,   //     Logical Maximum (1)
        0x95, 0x03,   //     Report Count (3)
        0x75, 0x01,   //     Report Size (1)
        0x81, 0x02,   //     Input (Data,Var,Abs)
        0x95, 0x01,   //     Report Count (1)
        0x75, 0x05,   //     Report Size (5)
        0x81, 0x03,   //     Input (Cnst,Var,Abs)
        0x05, 0x01,   //     Usage Page (Generic Desktop)
        0x09, 0x30,   //     Usage (X)
        0x09, 0x31,   //     Usage (Y)
        0x09, 0x38,   //     Usage (Wheel)
        0x15, 0x81,   //     Logical Minimum (-127)
        0x25, 0x7F,   //     Logical Maximum (127)
        0x75, 0x08,   //     Report Size (8)
        0x95, 0x03,   //     Report Count (3)
        0x81, 0x06,   //     Input (Data,Var,Rel)
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
    .device_name       = "ESP HID Mouse",
    .manufacturer_name = "Espressif",
    .serial_number     = "0001",
    .report_maps       = ble_report_maps,
    .report_maps_len   = sizeof(ble_report_maps) / sizeof(ble_report_maps[0])
};

/* HID runtime state */
static esp_hidd_dev_t *s_hid_dev = NULL;
static volatile bool s_hid_connected = false;
static TaskHandle_t s_demo_task = NULL;

/* HID demo task: periodic small mouse moves */
static void hid_mouse_demo_task(void *pv)
{
    (void)pv;
    uint8_t report[4] = {0};
    while (1) {
        if (s_hid_connected && s_hid_dev) {
            report[0] = 0;      // buttons
            report[1] = 10;     // dx
            report[2] = 0;      // dy
            report[3] = 0;      // wheel
            esp_hidd_dev_input_set(s_hid_dev, 0, 0, report, sizeof(report));
            ESP_LOGI(TAG, "Sent mouse report: dx=%d", report[1]);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void ble_hid_task_start_up(void)
{
    if (s_demo_task) return;
    xTaskCreate(hid_mouse_demo_task, "hid_demo", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &s_demo_task);
    ESP_LOGI(TAG, "HID demo started");
}

void ble_hid_task_shut_down(void)
{
    if (!s_demo_task) return;
    vTaskDelete(s_demo_task);
    s_demo_task = NULL;
    ESP_LOGI(TAG, "HID demo stopped");
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
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONNECT_EVENT");
        s_hid_connected = true;
        s_hid_dev = param->connect.dev;
        /* HID demo will be started when encryption is active (handled by nimble_reconnect) */
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_DISCONNECT_EVENT reason=%d", param->disconnect.reason);
        s_hid_connected = false;
        ble_hid_task_shut_down();
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
/* NimBLE host task entry */
void ble_hid_device_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* host callbacks */
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "ble_app_on_sync: host synced; start advertising");
    esp_hid_ble_gap_adv_start();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "ble_app_on_reset: reason=%d", reason);
}
#endif /* CONFIG_BT_NIMBLE_ENABLED */

/* main */
void app_main(void)
{
    esp_err_t err;

    /* 1) Initialize NVS (required for BLE bonding store) */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2) Set helpful log levels for SMP/reconnect debugging */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("ble_sm", ESP_LOG_DEBUG);
    esp_log_level_set("ble_store", ESP_LOG_DEBUG);
    esp_log_level_set("ble_hs", ESP_LOG_INFO);
    esp_log_level_set("NimBLE", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Initialize nimble_reconnect (if available) and HID GAP/device");

    /* 3) Initialize nimble_reconnect (must be called after NVS init).
       If nimble_reconnect.c is not linked, this will cause a linker error.
       Ensure nimble_reconnect.c is added to your build (main/CMakeLists.txt SRCS or component). */
    nimble_reconnect_init();

    /* 4) Initialize controller + stack via esp_hid_gap helper */
    err = esp_hid_gap_init(HID_DEV_MODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", err);
        return;
    }

    /* 5) Prepare advertising data */
    err = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", err);
        return;
    }

    /* 6) Initialize HID device and register callback */
    err = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, hidd_event_cb, &s_hid_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", err);
        return;
    }

#if CONFIG_BT_NIMBLE_ENABLED
    /* 7) Register host callbacks */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* 8) Start NimBLE host task using nimble_port_freertos_init */
    nimble_port_freertos_init(ble_hid_device_host_task);
#else
    /* Non-NimBLE: start advertising directly */
    esp_hid_ble_gap_adv_start();
#endif

    ESP_LOGI(TAG, "app_main finished");
}