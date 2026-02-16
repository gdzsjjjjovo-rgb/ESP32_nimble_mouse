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
#include "host/ble_store.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "esp_nimble_hci.h"
#include "esp_nimble_enable.h" /* local shim that calls nimble_port_freertos_init() */

void ble_store_config_init(void); /* provided by NimBLE/NVS persistence component */

/* Optional weak hooks */
void print_report_map_info(const esp_hid_raw_report_map_t *rm) __attribute__((weak));

/* Globals */
static esp_hidd_dev_t *s_hid_dev = NULL;
static volatile bool g_hid_connected = false;
static TaskHandle_t s_demo_task = NULL;

/* Reconnect/backoff timers */
static TimerHandle_t s_reconnect_timer = NULL;
static TimerHandle_t s_start_demo_timer = NULL;
static int s_reconnect_attempts = 0;

/* BLE manager task */
static TaskHandle_t s_ble_mgr_task = NULL;

static const char *TAG = "esp_ble_hid_mouse";
static const char *TAG_RECON = "hid_recon";

/* BLE manager notification bits */
#define BLE_MGR_CMD_START_ADV   (1U << 0)
#define BLE_MGR_CMD_STOP_ADV    (1U << 1)
#define BLE_MGR_CMD_START_DEMO  (1U << 2)
#define BLE_MGR_CMD_STOP_DEMO   (1U << 3)

/* Forward declarations to avoid implicit-declaration errors */
static void schedule_reconnect(void);
static void cancel_reconnect(void);
static void schedule_delayed_demo_start(void);
static void cancel_delayed_demo_start(void);

/* -------------------- Mouse Report Map -------------------- */
static const unsigned char mouseReportMap[] = {
    0x05,0x01,0x09,0x02,0xA1,0x01,
      0x09,0x01,0xA1,0x00,
        0x05,0x09,0x19,0x01,0x29,0x03,0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,
        0x95,0x01,0x75,0x05,0x81,0x03,
        0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x03,0x81,0x06,
      0xC0,
    0xC0
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    { .data = (unsigned char *)mouseReportMap, .len = sizeof(mouseReportMap) }
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

/* -------------------- HID send helper & demo -------------------- */
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

/* Provide start/stop for demo (used by events) */
void ble_hid_task_start_up(void)
{
    if (s_demo_task) return;
    xTaskCreate(hid_mouse_demo_task, "hid_mouse_demo", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, &s_demo_task);
    ESP_LOGI(TAG, "hid demo started");
}

void ble_hid_task_shut_down(void)
{
    if (!s_demo_task) return;
    vTaskDelete(s_demo_task);
    s_demo_task = NULL;
    ESP_LOGI(TAG, "hid demo stopped");
}

/* -------------------- stdin control (monitor) -------------------- */
static void stdin_control_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG, "stdin control: 's' start demo, 'x' stop demo, 'e' emit once");
    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        char ch = (char)c;
        if (ch == 's') ble_hid_task_start_up();
        else if (ch == 'x') ble_hid_task_shut_down();
        else if (ch == 'e') {
            send_mouse_report(20, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ===== Reconnect & delayed-start logic ===== */
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

static void reconnect_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG_RECON, "Reconnect timer fired, attempts=%d -> notify BLE manager to start adv", s_reconnect_attempts);
    if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_ADV, eSetBits);
    else schedule_reconnect();
}

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

static void cancel_reconnect(void)
{
    if (s_reconnect_timer) xTimerStop(s_reconnect_timer, 0);
    s_reconnect_attempts = 0;
}

static void start_demo_timer_cb(TimerHandle_t t)
{
    (void)t;
    if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_DEMO, eSetBits);
}

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

static void cancel_delayed_demo_start(void)
{
    if (s_start_demo_timer) xTimerStop(s_start_demo_timer, 0);
}

/* -------------------- BLE manager task -------------------- */
static void ble_manager_task(void *pv)
{
    uint32_t notified_value = 0;
    ESP_LOGI(TAG_RECON, "BLE manager task started");
    for (;;) {
        if (xTaskNotifyWait(0, UINT32_MAX, &notified_value, portMAX_DELAY) == pdTRUE) {
            if (notified_value & BLE_MGR_CMD_START_ADV) {
                ESP_LOGI(TAG_RECON, "BLE manager: start advertising (task context)");
                esp_err_t err = esp_hid_ble_gap_adv_start();
                if (err != ESP_OK) {
                    ESP_LOGW(TAG_RECON, "esp_hid_ble_gap_adv_start failed: %s", esp_err_to_name(err));
                    schedule_reconnect();
                } else {
                    s_reconnect_attempts = 0;
                }
            }
            if (notified_value & BLE_MGR_CMD_STOP_ADV) {
                ESP_LOGI(TAG_RECON, "BLE manager: stop advertising (task context)");
                /* esp_hid_ble_gap_adv_stop(); // if provided */
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

/* -------------------- BLE host callbacks -------------------- */
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

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "ble_app_on_reset: controller reset or host lost sync, reason=%d", reason);
    if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
    g_hid_connected = false;
    cancel_delayed_demo_start();
    cancel_reconnect();
}

/* -------------------- esp_hidd event callback (esp_event-style) -------------------- */
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args; (void)base;
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_START_EVENT");
        if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_ADV, eSetBits);
        else schedule_reconnect();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONNECT_EVENT");
        g_hid_connected = true;
        cancel_reconnect();
        schedule_delayed_demo_start();
        break;

    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_PROTOCOL_MODE_EVENT map_index=%u mode=%s",
                 param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;

    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_CONTROL_EVENT: control=%d", param->control.control);
        if (param->control.control) {
            if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_DEMO, eSetBits);
            else ble_hid_task_start_up();
        } else {
            if (s_ble_mgr_task) xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
            else ble_hid_task_shut_down();
        }
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_OUTPUT_EVENT len=%d", param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;

    case ESP_HIDD_FEATURE_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_FEATURE_EVENT len=%d", param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_DISCONNECT_EVENT reason=%d", param->disconnect.reason);
        if (s_ble_mgr_task) {
            xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_STOP_DEMO, eSetBits);
            xTaskNotify(s_ble_mgr_task, BLE_MGR_CMD_START_ADV, eSetBits);
        } else {
            ble_hid_task_shut_down();
            schedule_reconnect();
        }
        break;

    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "ESP_HIDD_STOP_EVENT");
        break;

    default:
        ESP_LOGI(TAG, "HIDD event id=%d", (int)event);
        break;
    }
}

/* -------------------- NimBLE host task (entry passed to esp_nimble_enable) -------------------- */
void ble_hid_device_host_task(void *param)
{
    (void)param;
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

    /* Logging levels for debug */
    esp_log_level_set("ble_hs", ESP_LOG_INFO);
    esp_log_level_set("ble_hci", ESP_LOG_INFO);
    esp_log_level_set("ble_sm", ESP_LOG_INFO);
    esp_log_level_set("ble_store", ESP_LOG_DEBUG);
    esp_log_level_set("NimBLE", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Initialize HID GAP and device (mouse only)");

    /* HID GAP init */
    ret = esp_hid_gap_init(HID_DEV_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", ret);
        return;
    }

    /* Prepare advertising fields (do NOT start advertising here) */
    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", ret);
        return;
    }

    /* Initialize HID device and register esp_event-style callback */
    ret = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_hid_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", ret);
        return;
    }

    if (print_report_map_info) print_report_map_info(&ble_report_maps[0]);

    /* NimBLE store (bonding persistence) */
    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* SM config: Just Works + key distribution */
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

    /* Set host callbacks so advertising is started only after host is ready */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* Start NimBLE host (shim in repo calls nimble_port_freertos_init) */
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
        return;
    }

    /* Create stdin control task for monitor input */
    xTaskCreate(stdin_control_task, "stdin_ctrl", 3 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

    ESP_LOGI(TAG, "app_main finished");
}