#ifndef ESP_NIMBLE_ENABLE_H
#define ESP_NIMBLE_ENABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/*
 * Start the NimBLE host.
 * The parameter is a pointer to the host task entry (type depends on nimble port).
 * Returns ESP_OK on success or an esp_err_t error code.
 */
esp_err_t esp_nimble_enable(void *host_task);

/*
 * Stop the NimBLE host.
 * Returns ESP_OK on success or an esp_err_t error code.
 */
esp_err_t esp_nimble_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_NIMBLE_ENABLE_H */