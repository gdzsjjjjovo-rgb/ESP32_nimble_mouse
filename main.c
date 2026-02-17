#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "nimble.h"   /* your BLE wrapper: wake_ble(), ble_mounted(), ble_hid_mouse_report() */
#include "paw3395.h"  /* sensor driver: wake_paw3395(), read_move(), (optional set_dpi) */
#include "pins.h"     /* board pin definitions (provide pin macros used below) */

static const char *TAG = "main";

/* -------------------------------------------------------------------------
   Weak stub for set_dpi to allow building before real implementation is added.
   Remove or replace when you have the real set_dpi() implementation.
   ------------------------------------------------------------------------- */
void set_dpi(uint16_t dpi) __attribute__((weak));
void set_dpi(uint16_t dpi)
{
    (void)dpi; /* no-op */
}

/* -------------------------------------------------------------------------
   Fallbacks: map CONFIG_* to pin macros from pins.h if CONFIG macros aren't set
   (this avoids compile errors when sdkconfig doesn't define these).
   Adjust pins.h to match your board names.
   ------------------------------------------------------------------------- */
#ifndef CONFIG_MICRO_PIN_L
#define CONFIG_MICRO_PIN_L LEFT_BUTTON_GPIO
#endif
#ifndef CONFIG_MICRO_PIN_R
#define CONFIG_MICRO_PIN_R RIGHT_BUTTON_GPIO
#endif
#ifndef CONFIG_MICRO_PIN_M
#define CONFIG_MICRO_PIN_M WHEEL_BUTTON_GPIO
#endif

#ifndef CONFIG_ENCODER_A_NUM
#define CONFIG_ENCODER_A_NUM WHEEL_ENC_A_GPIO
#endif
#ifndef CONFIG_ENCODER_B_NUM
#define CONFIG_ENCODER_B_NUM WHEEL_ENC_B_GPIO
#endif

#ifndef CONFIG_PAW3395D_MOTION_NUM
#ifdef PAW3395_MOTION_INT
#define CONFIG_PAW3395D_MOTION_NUM PAW3395_MOTION_INT
#else
#error "Motion pin macro not defined — set CONFIG_PAW3395D_MOTION_NUM or PAW3395_MOTION_INT in pins.h"
#endif
#endif

#ifndef CONFIG_MICRO_DEBOUNCE
#define CONFIG_MICRO_DEBOUNCE (50000)    /* 50 ms in microseconds */
#endif
#ifndef CONFIG_ENCODER_DEBOUNCE
#define CONFIG_ENCODER_DEBOUNCE (20000)  /* 20 ms in microseconds */
#endif
#ifndef CONFIG_STOP_INTERVAL_BLE
#define CONFIG_STOP_INTERVAL_BLE 8       /* ms between BLE HID reports */
#endif
#ifndef CONFIG_PAW3395_READ_INTERVAL
#define CONFIG_PAW3395_READ_INTERVAL 5   /* ms */
#endif

/* -------------------------------------------------------------------------
   Forward declarations expected from other modules (nimble.h / paw3395.h)
   nimble.h should provide:
     esp_err_t wake_ble(void);
     bool ble_mounted(void);
     void ble_hid_mouse_report(uint8_t buttons, char x, char y, char vertical);
   paw3395.h should provide sensor init/read functions used below:
     void wake_paw3395(void);
     esp_err_t read_move(int16_t *dx, int16_t *dy);
   ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
   Helper utilities
   ------------------------------------------------------------------------- */
static inline int8_t clamp_int8(int16_t v)
{
    if (v > 127) return 127;
    if (v < -128) return -128;
    return (int8_t)v;
}

static inline uint8_t get_encoder_state(void)
{
    return (gpio_get_level(CONFIG_ENCODER_A_NUM) << 1) | gpio_get_level(CONFIG_ENCODER_B_NUM);
}

/* -------------------------------------------------------------------------
   Types and state
   ------------------------------------------------------------------------- */
typedef struct {
    uint8_t bit;
    gpio_num_t gpio;
    uint64_t last_isr_tick;
} button_info_t;

typedef struct {
    int16_t x;
    int16_t y;
    int8_t vertical;
} accum_item_t;

/* Button mapping (bits) */
static button_info_t btn_left    = { .bit = 0, .gpio = CONFIG_MICRO_PIN_L };
static button_info_t btn_mid     = { .bit = 2, .gpio = CONFIG_MICRO_PIN_M };
static button_info_t btn_right   = { .bit = 1, .gpio = CONFIG_MICRO_PIN_R };

/* Runtime accumulators and sync objects */
static int16_t accum_x = 0;
static int16_t accum_y = 0;
static int8_t accum_vertical = 0;
static SemaphoreHandle_t accum_mutex = NULL;
static QueueHandle_t accum_queue = NULL;

static uint8_t motion_level = 0;
static TaskHandle_t move_task_handle = NULL;
static uint8_t encoder_state = 0;
static uint64_t last_slide_tick = 0;

static uint8_t buttons_temp = 0;
static uint8_t buttons = 0;
static TaskHandle_t report_task_handle = NULL;

/* -------------------------------------------------------------------------
   ISR handlers
   ------------------------------------------------------------------------- */
static void IRAM_ATTR on_move(void *args)
{
    (void)args;
    motion_level = gpio_get_level(CONFIG_PAW3395D_MOTION_NUM);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(move_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void IRAM_ATTR on_click(void *args)
{
    uint64_t now = esp_timer_get_time();
    button_info_t *btn = (button_info_t *)args;

    int level = gpio_get_level(btn->gpio);

    if (!level) buttons_temp |= (1 << btn->bit);
    else buttons_temp &= ~(1 << btn->bit);

    if (buttons_temp == buttons) return;

    if ((now - btn->last_isr_tick) >= CONFIG_MICRO_DEBOUNCE) {
        btn->last_isr_tick = now;
        buttons = buttons_temp;
        accum_item_t item = {0};
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(accum_queue, &item, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void IRAM_ATTR on_scroll(void *args)
{
    (void)args;
    uint64_t now = esp_timer_get_time();

    uint8_t encoder_state_temp = get_encoder_state();
    if (encoder_state_temp == encoder_state) return;

    int8_t state_transition = (encoder_state << 2) | encoder_state_temp;
    int8_t vertical;
    switch (state_transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000:
        vertical = -1; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100:
        vertical = 1; break;
    default:
        return;
    }

    if ((now - last_slide_tick) >= CONFIG_ENCODER_DEBOUNCE) {
        last_slide_tick = now;
        accum_item_t item = { .x = 0, .y = 0, .vertical = vertical };
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(accum_queue, &item, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    encoder_state = encoder_state_temp;
}

/* -------------------------------------------------------------------------
   Register ISRs
   ------------------------------------------------------------------------- */
static void reg_isr_handler(void)
{
    gpio_config_t motion_conf = {
        .pin_bit_mask = BIT64(CONFIG_PAW3395D_MOTION_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&motion_conf);
    gpio_isr_handler_add(CONFIG_PAW3395D_MOTION_NUM, on_move, NULL);

    gpio_config_t switch_conf = {
        .pin_bit_mask = BIT64(CONFIG_MICRO_PIN_L) | BIT64(CONFIG_MICRO_PIN_R) |
                        BIT64(CONFIG_MICRO_PIN_M) ,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&switch_conf);
    gpio_isr_handler_add(CONFIG_MICRO_PIN_L, on_click, &btn_left);
    gpio_isr_handler_add(CONFIG_MICRO_PIN_R, on_click, &btn_right);
    gpio_isr_handler_add(CONFIG_MICRO_PIN_M, on_click, &btn_mid);

    gpio_config_t enc_conf = {
        .pin_bit_mask = BIT64(CONFIG_ENCODER_B_NUM) | BIT64(CONFIG_ENCODER_A_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&enc_conf);
    gpio_isr_handler_add(CONFIG_ENCODER_B_NUM, on_scroll, NULL);
    gpio_isr_handler_add(CONFIG_ENCODER_A_NUM, on_scroll, NULL);

    encoder_state = get_encoder_state();
}

/* -------------------------------------------------------------------------
   Reporting
   ------------------------------------------------------------------------- */
static void report_send(uint8_t accum_buttons_temp, int16_t accum_x_temp, int16_t accum_y_temp, int8_t accum_vertical_temp)
{
    do {
        int8_t x_send = clamp_int8(accum_x_temp);
        int8_t y_send = clamp_int8(accum_y_temp);

        if (ble_mounted()) {
            /* nimble.h uses char for x/y/vertical — cast safely */
            ble_hid_mouse_report(accum_buttons_temp, (char)x_send, (char)y_send, (char)accum_vertical_temp);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_STOP_INTERVAL_BLE));
        } else {
            /* Not connected: short delay (alternatively buffer) */
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        accum_x_temp -= x_send;
        accum_y_temp -= y_send;
        accum_vertical_temp = 0;
    } while (accum_x_temp != 0 || accum_y_temp != 0 || accum_vertical_temp != 0);
}

/* report loop task */
static void report_loop_task(void *pv)
{
    (void)pv;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(accum_mutex, portMAX_DELAY) == pdTRUE) {
            uint8_t accum_buttons_temp = buttons;
            int16_t accum_x_temp = accum_x;
            int16_t accum_y_temp = accum_y;
            int8_t accum_vertical_temp = accum_vertical;

            accum_x = 0;
            accum_y = 0;
            accum_vertical = 0;

            xSemaphoreGive(accum_mutex);

            report_send(accum_buttons_temp, accum_x_temp, accum_y_temp, accum_vertical_temp);
        }
    }
}

/* accum loop task */
static void accum_loop_task(void *pv)
{
    (void)pv;
    accum_item_t item;
    for (;;) {
        if (xQueueReceive(accum_queue, &item, portMAX_DELAY) == pdPASS) {
            if (xSemaphoreTake(accum_mutex, portMAX_DELAY) == pdTRUE) {
                accum_x += item.x;
                accum_y += item.y;
                accum_vertical += item.vertical;
                xSemaphoreGive(accum_mutex);

                xTaskNotifyGive(report_task_handle);
            }
        }
    }
}

/* move loop task: poll sensor while motion pin indicates motion */
static void move_loop_task(void *pv)
{
    (void)pv;
    int16_t x = 0, y = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (motion_level == 0) {
            if (read_move(&x, &y) == ESP_OK) {
                if (x != 0 || y != 0) {
                    accum_item_t it = { .x = x, .y = y, .vertical = 0 };
                    xQueueSend(accum_queue, &it, 0);
                    x = y = 0;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(CONFIG_PAW3395_READ_INTERVAL));
        }

        /* drain */
        if (read_move(&x, &y) == ESP_OK) {
            if (x != 0 || y != 0) {
                accum_item_t it = { .x = x, .y = y, .vertical = 0 };
                xQueueSend(accum_queue, &it, 0);
            }
        }

        x = y = 0;
    }
}

/* API helpers */
void api_set_dpi(uint16_t dpi) { set_dpi(dpi); }

void api_macro(int16_t x, int16_t y, uint8_t btns)
{
    accum_item_t item = { .x = x, .y = y, .vertical = 0 };
    buttons = btns; /* preserve original (thread-unsafe) behavior */
    xQueueSend(accum_queue, &item, 0);
}

/* -------------------------------------------------------------------------
   app_main
   ------------------------------------------------------------------------- */
void app_main(void)
{
    esp_err_t ret;

    /* NVS for BLE bonding storage */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Start BLE */
    ret = wake_ble();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wake_ble failed: %s", esp_err_to_name(ret));
        /* continue: input tasks still useful but BLE won't send */
    }

    /* Initialize sensor */
    wake_paw3395();

    /* Install GPIO ISR service */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "GPIO ISR service installed");
    }

    reg_isr_handler();
    ESP_LOGI(TAG, "ISR handlers ready");

    /* Create queue and mutex */
    accum_queue = xQueueCreate(32, sizeof(accum_item_t));
    if (!accum_queue) {
        ESP_LOGE(TAG, "xQueueCreate failed");
        return;
    }
    accum_mutex = xSemaphoreCreateMutex();
    if (!accum_mutex) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
        return;
    }

    /* Create tasks */
    if (xTaskCreate(report_loop_task, "report_loop_task", 4096, NULL, 1, &report_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate report_loop_task failed");
        return;
    }
    if (xTaskCreate(accum_loop_task, "accum_loop_task", 4096, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate accum_loop_task failed");
        return;
    }
    if (xTaskCreate(move_loop_task, "move_loop_task", 4096, NULL, 1, &move_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate move_loop_task failed");
        return;
    }

    ESP_LOGI(TAG, "app_main finished, tasks running");
}
