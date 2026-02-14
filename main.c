/*
 * main.c
 * ESP BLE HID mouse — mouse-only, manual demo via monitor stdin.
 *
 * This file:
 *  - mouse-only HID report map
 *  - stdin control ('s' start demo, 'x' stop, 'e' emit once)
 *  - delayed demo start after connect
 *  - exponential backoff reconnect timer after disconnect
 *  - NimBLE sync and reset callbacks (start/stop advertising correctly)
 *  - BLE manager task to perform BLE operations from task context (avoids timer/task-context crashes)
 *  - optional weak hooks for optional modules (hid_input_init, paw_integration_init, gpio_button_init, print_report_map_info)
 *
 * Build and flash with ESP-IDF:
 *   . "$IDF_PATH/export.sh"
 *   idf.py fullclean
 *   idf.py build
 *   idf.py -p <PORT> flash monitor
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_event.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "esp_nimble_hci.h"

/* Forward (from NimBLE example) */
void ble_store_config_init(void);

/* Optional external modules (weak) — if linked, these will be used */
void hid_input_init(void) __attribute__((weak));
void gpio_button_init(void) __attribute__((weak));
esp_err_t paw_integration_init(void) __attribute__((weak));
void print_report_map_info(const esp_hid_raw_report_map_t *rm) __attribute__((weak));

/* HID device globals */
esp_hidd_dev_t *s_hid_dev = NULL;
volatile bool g_hid_connected = false;

/* Demo task handle */
static TaskHandle_t s_demo_task = NULL;

/* Timers & reconnect state */
static TimerHandle_t s_reconnect_timer = NULL;
static TimerHandle_t s_start_demo_timer = NULL;
static int s_reconnect_attempts = 0;

/* BLE manager task handle */
static TaskHandle_t s_ble_mgr_task = NULL;

/* Logging tags */
static const char *TAG = "esp_ble_hid_mouse";
static const char *TAG_RECON = "hid_recon";

/* BLE manager command bits (used with xTaskNotify) */
#define BLE_MGR_CMD_START_ADV   (1U << 0)
#define BLE_MGR_CMD_STOP_ADV    (1U << 1)
#define BLE_MGR_CMD_START_DEMO  (1U << 2)
#define BLE_MGR_CMD_STOP_DEMO   (1U << 3)

/* Forward prototypes for local helpers used out-of-order */
static void schedule_reconnect(void);
static void cancel_reconnect(void);
static void schedule_delayed_demo_start(void);
static void cancel_delayed_demo_start(void);
void ble_hid_task_start_up(void);
void ble_hid_task_shut_down(void);

/* -------------------- Report Map: mouse only (no Report ID) -------------------- */
/* Standard mouse: 3 buttons + X/Y + Wheel (relative) */
static const unsigned char mouseReportMap[] = {
    0x05,0x01,       /* Usage Page (Generic Desktop) */
    0x09,0x02,       /* Usage (Mouse) */
    0xA1,0x01,       /* Collection (Application) */
      0x09,0x01,     /*   Usage (Pointer) */
      0xA1,0x00,     /*   Collection (Physical) */
        0x05,0x09,   /*     Usage Page (Buttons) */
        0x19,0x01,   /*     Usage Minimum (01) */
        0x29,0x03,   /*     Usage Maximum (03) */
        0x15,0x00,   /*     Logical Minimum (0) */
        0x25,0x01,   /*     Logical Maximum (1) */
        0x95,0x03,   /*     Report Count (3) */
        0x75,0x01,   /*     Report Size (1) */
        0x81,0x02,   /*     Input (Data,Var,Abs) */
        0x95,0x01,   /*     Report Count (1) */
        0x75,0x05,   /*     Report Size (5) - padding */
        0x81,0x03,   /*     Input (Const,Arr,Abs) */
        0x05,0x01,   /*     Usage Page (Generic Desktop) */
        0x09,0x30,   /*     Usage (X) */
        0x09,0x31,   /*     Usage (Y) */
        0x09,0x38,   /*     Usage (Wheel) */
        0x15,0x81,   /*     Logical Minimum (-127) */
        0x25,0x7F,   /*     Logical Maximum (127) */
        0x75,0x08,   /*     Report Size (8) */
        0x95,0x03,   /*     Report Count (3) */
        0x81,0x06,   /*     Input (Data,Var,Rel) */
      0xC0,         /*   End Collection */
    0xC0            /* End Collection */
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

/* -------------------- helper: send mouse report -------------------- */
/* buffer format: [buttons, dx (int8), dy (int8), wheel (int8)] */
static void send_mouse_report(int8_t dx, int8_t dy, uint8_t buttons)
{
    uint8_t buf[4];
    buf[0] = buttons;
    buf[1] = (uint8_t)dx;
    buf[2] = (uint8_t)dy;
    buf[3] = 0x00;
    if (s_hid_dev && g_hid_connected) {
        int rc = esp_hidd_dev_input_set(s_hid_dev, 0, 0, buf, sizeof(buf));
        ESP_LOGI(TAG, "esp_hidd_dev_input_set rc=%d data=%02X %02X %02X %02X",
                 rc, buf[0], buf[1], buf[2], buf[3]);
    } else {
        ESP_LOGD(TAG, "send_mouse_report dropped (not connected or hid_dev NULL)");
    }
}

/* -------------------- demo task (periodic move) -------------------- */
static void hid_mouse_demo_task(void *pv)
{
    ESP_LOGI(TAG, "mouse demo task started");
    for (;;) {
        send_mouse_report(10, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        send_mouse_report(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Provide implementations for ble_hid_task_start_up / shut_down */
void ble_hid_task_start_up(void)
{
    if (s_demo_task) {
        ESP_LOGI(TAG, "ble_hid_task_start_up: already running");
        return;
    }
    xTaskCreate(hid_mouse_demo_task, "hid_mouse_demo", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &s_demo_task);
    ESP_LOGI(TAG, "ble_hid_task_start_up: demo created");
}

void ble_hid_task_shut_down(void)
{
    if (!s_demo_task) {
        ESP_LOGI(TAG, "ble_hid_task_shut_down: nothing to stop");
        return;
    }
    vTaskDelete(s_demo_task);
    s_demo_task = NULL;
    ESP_LOGI(TAG, "ble_hid_task_shut_down: demo stopped");
}

/* -------------------- stdin control task -------------------- */
/* Uses idf.py monitor's stdin forwarding (fgetc) */
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

/* ===== Reconnect & delayed-start logic ===== */
/* Configurable backoff */
#define RECONNECT_BACKOFF_INITIAL_MS 1000
#define RECONNECT_BACKOFF_MAX_MS     30000
#define RECONNECT_MAX_ATTEMPTS       10

static uint32_t reconnect_backoff_ms(int attempts)
{
    uint32_t backoff = RECONNECT_BACKOFF_INITIAL_MS;
    if (attempts > 1) {
        int shift = attempts - 1;
        if (shift > 31) shift = 31;
        uint64_t tmp = ((uint64_t)RECONNECT_BACKOFF_INITIAL_MS) << shift;
        if (tmp > RECONNECT_BACKOFF_MAX_MS) backoff = RECONNECT_BACKOFF_MAX_MS;
        else backoff = (uint32_t)tmp;
    }
    if (backoff > RECONNECT_BACKOFF_MAX_MS) backoff = RECONNECT_BACKOFF_MAX_MS;
    return backoff;
}

/* reconnect timer callback: runs in timer task context -> notify BLE manager */
static void reconnect_timer_cb(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG_RECON, "Reconnect timer fired, attempts=%d -> notify BLE manager to start adv", s_reconnect_attempts);
    if (s_ble_mgr_task) {
        xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_ADV, eSetBits);
    } else {
        ESP_LOGW(TAG_RECON, "ble manager task not ready; scheduling reconnect again");
        schedule_reconnect();
    }
}

/* schedule reconnect with exponential backoff */
static void schedule_reconnect(void)
{
    if (!s_reconnect_timer) {
        s_reconnect_timer = xTimerCreate("reconnect_t", pdMS_TO_TICKS(RECONNECT_BACKOFF_INITIAL_MS), pdFALSE, NULL, reconnect_timer_cb);
        if (!s_reconnect_timer) {
            ESP_LOGE(TAG_RECON, "create reconnect timer failed");
            return;
        }
    }

    if (s_reconnect_attempts < RECONNECT_MAX_ATTEMPTS) s_reconnect_attempts++;
    uint32_t delay_ms = reconnect_backoff_ms(s_reconnect_attempts);
    ESP_LOGI(TAG_RECON, "scheduling reconnect in %u ms (attempt %d)", delay_ms, s_reconnect_attempts);

    xTimerStop(s_reconnect_timer, 0);
    xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(delay_ms), 0);
    xTimerStart(s_reconnect_timer, 0);
}

/* cancel reconnect (on successful connect) */
static void cancel_reconnect(void)
{
    if (s_reconnect_timer) {
        xTimerStop(s_reconnect_timer, 0);
    }
    s_reconnect_attempts = 0;
}

/* delayed demo start callback: notify BLE manager to start demo (safe in timer context) */
static void start_demo_timer_cb(TimerHandle_t t)
{
    if (s_ble_mgr_task) {
        xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_DEMO, eSetBits);
    }
}

/* schedule delayed demo start after connect */
static void schedule_delayed_demo_start(void)
{
    if (!s_start_demo_timer) {
        s_start_demo_timer = xTimerCreate("start_demo", pdMS_TO_TICKS(2000), pdFALSE, NULL, start_demo_timer_cb);
        if (!s_start_demo_timer) {
            ESP_LOGW(TAG_RECON, "create start_demo timer failed");
            return;
        }
    }
    xTimerStop(s_start_demo_timer, 0);
    xTimerStart(s_start_demo_timer, 0);
}

/* cancel delayed demo (on disconnect or manual stop) */
static void cancel_delayed_demo_start(void)
{
    if (s_start_demo_timer) xTimerStop(s_start_demo_timer, 0);
}

/* -------------------- BLE manager task -------------------- */
/* Performs BLE operations (advertise start/stop) and demo control from task context */
static void ble_manager_task(void *pv)
{
    uint32_t notified_value = 0;

    ESP_LOGI(TAG_RECON, "BLE manager task started");
    for (;;) {
        /* Wait for notification bits (cleared on exit) */
        if (xTaskNotifyWait(0, UINT32_MAX, &notified_value, portMAX_DELAY) == pdTRUE) {
            if (notified_value & BLE_MGR_CMD_START_ADV) {
                ESP_LOGI(TAG_RECON, "BLE manager: start advertising (task context)");
                esp_err_t err = esp_hid_ble_gap_adv_start();
                if (err != ESP_OK) {
                    ESP_LOGW(TAG_RECON, "esp_hid_ble_gap_adv_start failed in ble_mgr: %s", esp_err_to_name(err));
                    /* schedule_reconnect will arrange backoff */
                    schedule_reconnect();
                } else {
                    /* successful start: reset reconnect attempts */
                    s_reconnect_attempts = 0;
                }
            }
            if (notified_value & BLE_MGR_CMD_STOP_ADV) {
                ESP_LOGI(TAG_RECON, "BLE manager: stop advertising (task context)");
                /* If you have a stop API, call it here. Most stacks auto-stop on connect */
                /* esp_hid_ble_gap_adv_stop(); */
            }
            if (notified_value & BLE_MGR_CMD_START_DEMO) {
                ESP_LOGI(TAG_RECON, "BLE manager: start demo (task context)");
                ble_hid_task_start_up();
            }
            if (notified_value & BLE_MGR_CMD_STOP_DEMO) {
                ESP_LOGI(TAG_RECON, "BLE manager: stop demo (task context)");
                ble_hid_task_shut_down();
            }
        }
    }
}

/* -------------------- BLE host sync & reset callbacks -------------------- */
/* In many IDF versions ble_hs_cfg.sync_cb is void (*)(void) */
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "ble_app_on_sync: Host synced -> notify BLE manager to start advertising");
    if (s_ble_mgr_task) {
        xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_ADV, eSetBits);
    } else {
        ESP_LOGW(TAG_RECON, "ble manager not ready on sync; scheduling reconnect");
        schedule_reconnect();
    }
}

/* Reset callback: called when controller resets or host loses sync.
   Reason is NimBLE return code. */
static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "ble_app_on_reset: controller reset or host lost sync, reason=%d", reason);

    /* Stop activity and clear state; advertising/starting will be handled on next sync. */
    /* Notify BLE manager to stop demo if necessary */
    if (s_ble_mgr_task) {
        xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
    }
    g_hid_connected = false;
    cancel_delayed_demo_start();
    cancel_reconnect();
}

/* -------------------- esp_hidd event callback -------------------- */
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_START_EVENT - advertising (started by BLE manager)");
        /* do not call esp_hid_ble_gap_adv_start() here; BLE manager handles advertising */
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONNECT_EVENT");
        g_hid_connected = true;
        cancel_reconnect();
        /* schedule delayed demo start to avoid immediate traffic (handled by timer->ble_mgr) */
        schedule_delayed_demo_start();
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_DISCONNECT_EVENT");
        /* stop demo and reset state */
        if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
        g_hid_connected = false;
        cancel_delayed_demo_start();
        /* schedule reconnect with backoff (timer will notify ble_mgr to start adv) */
        schedule_reconnect();
        break;
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_PROTOCOL_MODE_EVENT map_index=%u mode=%s",
                 param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONTROL_EVENT: control=%d (auto-start disabled)", param->control.control);
        if (!param->control.control) {
            if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
            ble_hid_task_shut_down();
        }
        break;
    default:
        ESP_LOGI(TAG, "HIDD event id=%d", (int)event);
        break;
    }
}

/* -------------------- NimBLE host task -------------------- */
void ble_hid_device_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* -------------------- app_main -------------------- */
void app_main(void)
{
    esp_err_t ret;

    /* NVS init */
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

    ESP_LOGI(TAG, "Initialize HID GAP and device");

    /* HID GAP init */
    ret = esp_hid_gap_init(HID_DEV_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", ret);
        return;
    }

    /* Adv init with mouse appearance */
    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", ret);
        return;
    }

    /* Initialize HID device */
    ret = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_hid_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", ret);
        return;
    }

    /* Print report map if helper exists (weak) */
    if (print_report_map_info) {
        print_report_map_info(&ble_report_maps[0]);
    }

    /* NimBLE store (bonding persistence) */
    ble_store_config_init();

    /* SM config for Just Works */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* Create BLE manager task before enabling NimBLE host */
    if (s_ble_mgr_task == NULL) {
        BaseType_t ok = xTaskCreate(ble_manager_task, "ble_mgr", 4096, NULL, tskIDLE_PRIORITY + 3, &s_ble_mgr_task);
        if (ok != pdPASS) {
            ESP_LOGW(TAG_RECON, "create ble_manager_task failed");
            s_ble_mgr_task = NULL;
        }
    }

    /* Set host sync and reset callbacks so we start adv only after host is ready,
       and handle controller resets gracefully. */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* Start NimBLE host */
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
        return;
    }

    /* Initialize optional modules if linked (weak symbols) */
    if (hid_input_init) hid_input_init();
    if (gpio_button_init) gpio_button_init();
    if (paw_integration_init) {
        esp_err_t r = paw_integration_init();
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "paw_integration_init returned %s", esp_err_to_name(r));
        }
    }

    /* Create stdin control task for monitor input */
    xTaskCreate(stdin_control_task, "stdin_ctrl", 3 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

    /* NOTE: DO NOT call esp_hid_ble_gap_adv_start() here — advertising is
       started in ble_app_on_sync() (which notifies ble_manager_task) once NimBLE host is synchronized. */

    ESP_LOGI(TAG, "app_main finished");
}