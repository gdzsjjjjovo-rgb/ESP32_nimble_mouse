#include "esp_log.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* 必须包含定义 esp_hid_raw_report_map_t 的头 */
#include "esp_hidd.h"
#include "esp_hid_gap.h"

/* extern 声明：确保名字与 main.c 中定义的 ble_report_maps 相同 */
extern esp_hid_raw_report_map_t ble_report_maps[]; /* defined in main.c */

static const char *TAG_RP = "HID_RM";

/* 打印 raw hex 并做简单的 Report ID / Input/Output/Feature 扫描 */
void print_report_map_info(const esp_hid_raw_report_map_t *rm)
{
    if (!rm || rm->len == 0 || !rm->data) {
        ESP_LOGW(TAG_RP, "empty report map");
        return;
    }
    const uint8_t *data = rm->data;
    size_t len = rm->len;
    ESP_LOGI(TAG_RP, "Report Map len=%d bytes", (int)len);

    /* 打印 raw hex 每行 16 字节 */
    char line[128];
    int pos = 0;
    for (size_t i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
        if ((i % 16) == 15 || i == len - 1) {
            ESP_LOGI(TAG_RP, "%04x: %s", (int)(i - (i % 16)), line);
            pos = 0;
            line[0] = '\0';
        }
    }

    /* 简易解析：查找 REPORT_ID (0x85) 和 Input/Output/Feature (0x81/0x91/0xB1) */
    int last_report_id = 0;
    typedef struct { int id; int type_mask; } rep_t;
    rep_t reps[16];
    int rc = 0;
    memset(reps, 0, sizeof(reps));

    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        if (b == 0x85 && i + 1 < len) {
            last_report_id = data[++i];
            int found = 0;
            for (int k = 0; k < rc; k++) if (reps[k].id == last_report_id) { found = 1; break; }
            if (!found && rc < (int)(sizeof(reps)/sizeof(reps[0]))) {
                reps[rc].id = last_report_id; reps[rc].type_mask = 0; rc++;
            }
            continue;
        }
        if (b == 0x81 || b == 0x91 || b == 0xB1) {
            int tbit = (b == 0x81) ? 1 : (b == 0x91) ? 2 : 4;
            int assign = last_report_id;
            int found = 0;
            for (int k = 0; k < rc; k++) {
                if (reps[k].id == assign) { reps[k].type_mask |= tbit; found = 1; break; }
            }
            if (!found && rc < (int)(sizeof(reps)/sizeof(reps[0]))) {
                reps[rc].id = assign; reps[rc].type_mask = tbit; rc++;
            }
        }
    }

    if (rc == 0) {
        ESP_LOGI(TAG_RP, "No explicit Report ID found in descriptor (report_id = 0 assumed)");
    } else {
        for (int k = 0; k < rc; k++) {
            char types[64] = {0};
            int p = 0;
            if (reps[k].type_mask & 1) p += snprintf(types+p, sizeof(types)-p, "Input ");
            if (reps[k].type_mask & 2) p += snprintf(types+p, sizeof(types)-p, "Output ");
            if (reps[k].type_mask & 4) p += snprintf(types+p, sizeof(types)-p, "Feature ");
            ESP_LOGI(TAG_RP, "Found Report ID=%d  Types: %s", reps[k].id, types[0] ? types : "Unknown");
        }
    }
}