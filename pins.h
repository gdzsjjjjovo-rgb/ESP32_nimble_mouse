#ifndef PINS_H
#define PINS_H

#include "driver/spi_master.h" // 确保 SPI_HOST/VSPI_HOST/HSPI_HOST 宏可用

// 已避开不可用引脚： 0, 2, 4, 5, 12, 15

// SPI（使用 VSPI_HOST，安全且不与 flash 总线冲突）
#define PAW3395_SPI_MOSI   23   // VSPI MOSI
#define PAW3395_SPI_MISO   19   // VSPI MISO
#define PAW3395_SPI_SCLK   18   // VSPI SCLK
#define PAW3395_SPI_CS     13   // CS，选用 GPIO13，非启动脚

// PAW3395 控制引脚（避免影响启动）
#define PAW3395_nRESET     17   // RESET = GPIO17（安全）
#define PAW3395_MOTION_INT 27   // MOTION/IRQ = GPIO27 (输入)

// 鼠标按键与控制（若你已接线不同可按需修改）
#define LEFT_BUTTON_GPIO   26
#define RIGHT_BUTTON_GPIO  33
#define WHEEL_BUTTON_GPIO  25
#define DPI_SWITCH_GPIO    32

// 滚轮编码器（GPIO34/35 为输入专用）
#define WHEEL_ENC_A_GPIO   34
#define WHEEL_ENC_B_GPIO   35

// 使用的 SPI host：强制使用 VSPI_HOST（不要使用 SPI_HOST/flash）
#ifndef PAW3395_SPI_HOST
#define PAW3395_SPI_HOST   VSPI_HOST
#endif

// 报告率（Hz）
#define MOUSE_REPORT_RATE_DEFAULT 150
#define MOUSE_REPORT_RATE_MIN     100
#define MOUSE_REPORT_RATE_MAX     150

#endif // PINS_H