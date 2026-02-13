/*
 * Minimal shim implementation of esp_nimble_enable / esp_nimble_disable
 * Matches the esp_err_t esp_nimble_enable(void *host_task) signature provided
 * by the NimBLE port header in esp-idf.
 *
 * NOTE: This is a lightweight shim. The real esp-nimble component from ESP-IDF
 * may do extra initializations. Prefer the official component when available.
 */

#include "esp_nimble_enable.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/nimble_port.h"
#include "esp_log.h"

static const char *TAG = "esp_nimble_enable_shim";

esp_err_t esp_nimble_enable(void *host_task)
{
    if (host_task == NULL) {
        ESP_LOGE(TAG, "host_task is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting NimBLE host via nimble_port_freertos_init()");
    /*
     * nimble_port_freertos_init expects a function pointer with signature
     * void (*host_task)(void *). We forward the provided pointer as-is.
     */
    nimble_port_freertos_init((void (*)(void *))host_task);

    /* Success: host task created and NimBLE will run there */
    return ESP_OK;
}

esp_err_t esp_nimble_disable(void)
{
    ESP_LOGI(TAG, "Stopping NimBLE host");
    /* Request NimBLE stack shutdown */
    nimble_port_stop();

    /* Deinitialize FreeRTOS integration */
    nimble_port_freertos_deinit();

    return ESP_OK;
}