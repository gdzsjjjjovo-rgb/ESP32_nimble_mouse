#ifndef MOUSE_REPORT_H
#define MOUSE_REPORT_H

#include <stdint.h>

/*
 * 发送鼠标报告的接口。
 * 参数：
 *  - buttons : bit0 = 左键, bit1 = 右键, bit2 = 中键
 *  - dx, dy  : 相对移动 (signed 8-bit)
 *  - wheel   : 滚轮 (signed 8-bit)
 *
 * 该函数在本组件中由弱实现（mouse_report_stub.c），你可以在 BLE HID 模块中实现同名函数
 * 来替换/重写，直接把数据通过 GATT 通知发送出去。
 */
void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel);

#endif // MOUSE_REPORT_H