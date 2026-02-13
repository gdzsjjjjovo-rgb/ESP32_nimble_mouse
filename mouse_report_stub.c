#include "mouse_report.h"
#include "esp_log.h"

static const char *TAG = "mouse_report_stub";

/* Weak stub: 如果你没有 BLE HID 的 send_mouse_report 实现，
   这个文件会提供 send_mouse_report 的弱实现，打印日志。
   如果你 later 在其它模块实现同名函数，链接器会替换此实现。 */
__attribute__((weak))
void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    ESP_LOGI(TAG, "mouse report btn=0x%02x dx=%d dy=%d wheel=%d", buttons, dx, dy, wheel);
}