#ifndef NIMBLE_RECONNECT_H
#define NIMBLE_RECONNECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

#ifdef CONFIG_BT_NIMBLE_ENABLED
/* Make ble_gap_event visible to callers */
#include "host/ble_gap.h"
#endif

/* Initialize NimBLE SMP/store defaults for robust reconnect handling.
   Call once after nvs_flash_init() and before starting the NimBLE host. */
void nimble_reconnect_init(void);

/* Handle common gap events related to reconnect/pairing/encryption.
   - If CONFIG_BT_NIMBLE_ENABLED is set, this takes struct ble_gap_event* and may
     return BLE_GAP_REPEAT_PAIRING_RETRY to request stack retrying pairing.
   - If NimBLE is not enabled, a no-op stub is provided.
*/
#ifdef CONFIG_BT_NIMBLE_ENABLED
int nimble_reconnect_handle_gap_event(struct ble_gap_event *event);
#else
/* stub for non-NimBLE builds */
static inline int nimble_reconnect_handle_gap_event(void *event) { (void)event; return 0; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* NIMBLE_RECONNECT_H */