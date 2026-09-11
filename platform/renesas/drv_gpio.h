/*
 * drv_gpio.h
 *
 *  GPIO 输出引脚封装(阶段5 补:把 dev_wheel 的方向引脚从直接 R_IOPORT_PinWrite
 *  收进 drv 层,使 dev_wheel 不再认识 HAL 枚举,换平台只改本文件)。
 *  当前仅"单个引脚、写高低电平"——dev_wheel 方向脚够用,不做更多。
 *
 *  Created for: RA6M5 Robot Vacuum
 */

#ifndef DRV_DRV_GPIO_H_
#define DRV_DRV_GPIO_H_

#include <stdint.h>
#include <stdbool.h>
#include "hal_data.h"          /* bsp_io_port_pin_t / R_IOPORT_PinWrite / g_ioport_ctrl */

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO 输出引脚对象。pin 用 RA 的 bsp_io_port_pin_t;对外电平用 0/1 而非 BSP 枚举,
 *  这样上层(dev_wheel)不依赖 BSP_IO_LEVEL_*,换平台只改本驱动。 */
typedef struct {
    bsp_io_port_pin_t pin;
    bool              ready;        /* init 后置 true,防误用 */
} drv_gpio_t;

/* 初始化:记下引脚号。RA 的 IOPORT 由板子启动就绪,无需 Open,故这里只登记。 */
void drv_gpio_init(drv_gpio_t *g, bsp_io_port_pin_t pin);

/* 写电平:level 非 0 写高、0 写低。封装 R_IOPORT_PinWrite。 */
void drv_gpio_write(drv_gpio_t *g, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* DRV_DRV_GPIO_H_ */