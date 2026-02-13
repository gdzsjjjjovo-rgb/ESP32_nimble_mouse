#ifndef PAW3395_H
#define PAW3395_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h" /* for spi_host_device_t */

/*
 * paw3395.h
 * 这个头与你当前项目里的 paw3395.c 实现匹配：
 * - paw3395_init 接受 const paw3395_config_t *
 * - paw3395_read_motion 使用 (int8_t *dx, int8_t *dy, uint8_t *buttons)
 *
 * 请用此头替换工程中所有旧的 paw3395.h（避免重复定义冲突）。
 */

/* 配置结构：包含 SPI/引脚等配置项（与 paw3395.c 中 s_cfg 字段一致） */
typedef struct {
    int pin_mosi;
    int pin_miso;
    int pin_sclk;
    int pin_cs;
    int pin_reset;
    int pin_int;
    spi_host_device_t spi_host; /* HSPI_HOST / VSPI_HOST */
    uint32_t spi_clock_hz;      /* SPI 时钟，单位 Hz */
} paw3395_config_t;

/* 运动数据结构（如果实现没有用此结构，仍保留以防扩展） */
typedef struct {
    int16_t dx;
    int16_t dy;
    int8_t  squal;
    bool    motion;
} paw3395_motion_t;

/* API（与 paw3395.c 中实现匹配） */
esp_err_t paw3395_init(const paw3395_config_t *cfg);
esp_err_t paw3395_reset(void);
esp_err_t paw3395_set_report_rate_hz(uint16_t hz);
esp_err_t paw3395_set_cpi(uint16_t cpi);
/* 读取运动：返回 dx, dy（signed 8-bit 或实现定义的宽度）与 buttons（如果驱动提供）*/
esp_err_t paw3395_read_motion(int8_t *dx, int8_t *dy, uint8_t *buttons);
void paw3395_deinit(void);

#endif /* PAW3395_H */