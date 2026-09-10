/*
 * drv_gpio.h
 *
 *  GPIO 驱动封装：输出与输入两组接口，屏蔽 HAL 调用细节。
 *
 *  特性：
 *    - 零额外资源，结构体由调用方持有。
 *    - 输出接口与输入接口各自独立的类型与函数。
 *    - 所有接口对未初始化对象安全返回（写：忽略；读：返回 0）。
 *    - 仅依赖 <stdint.h> <stdbool.h> 与 stm32f1xx_hal.h（按芯片型号调整）。
 *
 *  使用前提：
 *    1. 调用方需先调用对应 init，再使用 write / read。
 *    2. HAL 层的 GPIO 时钟与引脚模式应由调用方在 init 前完成配置。
 *
 *  示例：
 *    // 输出：初始化并置高
 *    drv_gpio_t led;
 *    drv_gpio_init(&led, LED_GPIO_Port, LED_Pin);
 *    drv_gpio_write(&led, 1);
 *
 *    // 输入：初始化并读取
 *    drv_gpio_in_t key;
 *    drv_gpio_in_init(&key, KEY_GPIO_Port, KEY_Pin);
 *    uint8_t pressed = drv_gpio_in_read(&key);
 */

#ifndef DRV_GPIO_H
#define DRV_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"   /* 根据实际芯片型号调整 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 输出封装 ========== */

typedef struct {
    GPIO_TypeDef *port;      /* 端口 */
    uint16_t      pin;       /* 引脚号 */
    bool          ready;     /* 是否已初始化 */
} drv_gpio_t;

/**
 * @brief 初始化 GPIO 输出对象。
 * @param g    GPIO 输出对象指针
 * @param port GPIO 端口（如 GPIOA）
 * @param pin  GPIO 引脚（如 GPIO_PIN_5）
 * @note  仅记录 port/pin 并置 ready；不影响引脚电平。
 * @note  g 为 NULL 时直接返回。
 */
void drv_gpio_init (drv_gpio_t *g, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief 写入 GPIO 输出电平。
 * @param g     GPIO 输出对象指针
 * @param level 0=低电平，非 0=高电平
 * @note  未初始化（g 为 NULL 或 ready=false）时直接返回。
 */
void drv_gpio_write(drv_gpio_t *g, uint8_t level);

/* ========== 2. 输入封装 ========== */

typedef struct {
    GPIO_TypeDef *port;      /* 端口 */
    uint16_t      pin;       /* 引脚号 */
    bool          ready;     /* 是否已初始化 */
} drv_gpio_in_t;

/**
 * @brief 初始化 GPIO 输入对象。
 * @param g    GPIO 输入对象指针
 * @param port GPIO 端口（如 GPIOB）
 * @param pin  GPIO 引脚（如 GPIO_PIN_3）
 * @note  仅记录 port/pin 并置 ready；不改变引脚方向与上下拉。
 * @note  g 为 NULL 时直接返回。
 */
void drv_gpio_in_init (drv_gpio_in_t *g, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief 读取 GPIO 输入电平。
 * @param g GPIO 输入对象指针
 * @return 1=高电平，0=低电平
 * @note  未初始化（g 为 NULL 或 ready=false）时返回 0，即按低电平处理。
 */
uint8_t drv_gpio_in_read(drv_gpio_in_t *g);

#ifdef __cplusplus
}
#endif

#endif /* DRV_GPIO_H */