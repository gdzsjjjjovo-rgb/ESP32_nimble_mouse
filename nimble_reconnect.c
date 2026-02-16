#include "nimble_reconnect.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"
#include "host/ble_sm.h"
#endif

static const char *TAG = "nimble_reconnect";

/* Optional application hooks (if present in your application)
   These symbols should be defined in your app (e.g. main.c) if you want
   the reconnect module to start/stop HID demo when encryption becomes active.
*/
extern void ble_hid_task_start_up(void);
extern void ble_hid_task_shut_down(void);

void nimble_reconnect_init(void)
{
#ifdef CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGI(TAG, "nimble_reconnect_init: initializing ble_store and SM defaults");

    /* Ensure the NimBLE store is configured (uses NVS backend provided by IDF) */
    extern void ble_store_config_init(void);
    ble_store_config_init();

    /* Use the default store status callback helper if available */
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Configure security manager suitable for a headless HID device */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_SIGN;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_SIGN;

    ESP_LOGI(TAG, "SM cfg: bonding=%d io=%d mitm=%d sc=%d our_kdist=0x%02X their_kdist=0x%02X",
             ble_hs_cfg.sm_bonding, ble_hs_cfg.sm_io_cap, ble_hs_cfg.sm_mitm, ble_hs_cfg.sm_sc,
             ble_hs_cfg.sm_our_key_dist, ble_hs_cfg.sm_their_key_dist);
#else
    ESP_LOGW(TAG, "nimble_reconnect_init: NimBLE not enabled in configuration");
    (void)TAG;
#endif
}

#ifdef CONFIG_BT_NIMBLE_ENABLED
int nimble_reconnect_handle_gap_event(struct ble_gap_event *event)
{
    if (event == NULL) return 0;

    int rc;
    struct ble_gap_conn_desc desc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "reconnect: CONNECT status=%d", event->connect.status);
        if (event->connect.status == 0) {
            int conn_handle = event->connect.conn_handle;
            rc = ble_gap_security_initiate(conn_handle);
            ESP_LOGI(TAG, "reconnect: ble_gap_security_initiate rc=%d conn=%d", rc, conn_handle);
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "reconnect: ENC_CHANGE status=%d conn=%d", event->enc_change.status, event->enc_change.conn_handle);
        if (event->enc_change.status == 0) {
            rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGI(TAG, "reconnect: encryption active for peer %02X:%02X:%02X:%02X:%02X:%02X",
                         desc.peer_id_addr.val[5], desc.peer_id_addr.val[4], desc.peer_id_addr.val[3],
                         desc.peer_id_addr.val[2], desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);
                if (&ble_hid_task_start_up) ble_hid_task_start_up();
            }
        } else {
            rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGW(TAG, "reconnect: enc failed; deleting peer and awaiting re-pair");
                ble_store_util_delete_peer(&desc.peer_id_addr);
            } else {
                ESP_LOGW(TAG, "reconnect: enc failed; conn_find rc=%d", rc);
            }
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        ESP_LOGW(TAG, "reconnect: REPEAT_PAIRING detected (conn=%d)", event->repeat_pairing.conn_handle);
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
            ESP_LOGI(TAG, "reconnect: deleted old peer entry; returning RETRY");
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        } else {
            ESP_LOGW(TAG, "reconnect: repeat_pairing - conn_find rc=%d", rc);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "reconnect: DISCONNECT reason=0x%04X (%d)", event->disconnect.reason, event->disconnect.reason);
        if (&ble_hid_task_shut_down) ble_hid_task_shut_down();
        break;

    default:
        break;
    }

    return 0;
}
#endif /* CONFIG_BT_NIMBLE_ENABLED */