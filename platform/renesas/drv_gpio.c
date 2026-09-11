/*
 * drv_gpio.c
 *
 *  GPIO 输出引脚封装实现(RA6M5/FSP)。
 *  换平台(如 STM32)只需把本文件改成 HAL_GPIO_WritePin,drv_gpio.h 与上层不动。
 */

#include "drv_gpio.h"

void drv_gpio_init(drv_gpio_t *g, bsp_io_port_pin_t pin)
{
    g->pin   = pin;
    g->ready = true;
}

void drv_gpio_write(drv_gpio_t *g, uint8_t level)
{
    if (!g || !g->ready) return;     /* 防误用:未 init 不动作 */
    bsp_io_level_t lvl = (level != 0u) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
    R_IOPORT_PinWrite(&g_ioport_ctrl, g->pin, lvl);
}