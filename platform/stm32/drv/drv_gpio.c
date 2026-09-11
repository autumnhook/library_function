/*
 * drv_gpio.c
 *
 *  GPIO 驱动封装：输出与输入两组接口，屏蔽 HAL 调用细节。
 *
 *  特性：
 *    - 输出接口 drv_gpio_t / 输入接口 drv_gpio_in_t 各自独立。
 *    - init 时保存 port/pin 并置 ready=true；未 init 时接口安全返回。
 *    - 零额外资源，结构体由调用方持有。
 *    - 提供中性双引脚互补写 drv_gpio_write_pair()，不涉及方向/电机语义。
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
 *
 *    // 双引脚互补写：A=高 B=低
 *    drv_gpio_write_pair(GPIOA, GPIO_PIN_0, GPIO_PIN_1, true);
 */

#include "drv_gpio.h"

/* ========== 输出实现 ========== */

/**
 * @brief 初始化 GPIO 输出对象。
 * @param g    GPIO 输出对象指针
 * @param port GPIO 端口（如 GPIOA）
 * @param pin  GPIO 引脚（如 GPIO_PIN_5）
 * @note  仅记录 port/pin 并置 ready；不影响引脚电平。
 * @note  g 为 NULL 时直接返回。
 */
void drv_gpio_init(drv_gpio_t *g, GPIO_TypeDef *port, uint16_t pin)
{
    if (!g) return;
    g->port  = port;
    g->pin   = pin;
    g->ready = true;
}

/**
 * @brief 写入 GPIO 输出电平。
 * @param g     GPIO 输出对象指针
 * @param level 0=低电平，非 0=高电平
 * @note  未初始化（g 为 NULL 或 ready=false）时直接返回。
 */
void drv_gpio_write(drv_gpio_t *g, uint8_t level)
{
    if (!g || !g->ready) return;
    HAL_GPIO_WritePin(g->port, g->pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 同一端口写两个引脚，a_high 决定 a/b 的高低。
 * @param port   GPIO 端口（如 GPIOA）
 * @param pin_a  引脚 A
 * @param pin_b  引脚 B
 * @param a_high true: A=高 B=低；false: A=低 B=高
 * @note  纯 GPIO 语义，不涉及“方向 / 电机”含义。
 * @note  port 为 NULL 时直接返回。
 */
void drv_gpio_write_pair(GPIO_TypeDef *port, uint16_t pin_a,
                         uint16_t pin_b, bool a_high)
{
    if (!port) return;
    HAL_GPIO_WritePin(port, pin_a, a_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, pin_b, a_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ========== 输入实现 ========== */

/**
 * @brief 初始化 GPIO 输入对象。
 * @param g    GPIO 输入对象指针
 * @param port GPIO 端口（如 GPIOB）
 * @param pin  GPIO 引脚（如 GPIO_PIN_3）
 * @note  仅记录 port/pin 并置 ready；不改变引脚方向与上下拉。
 * @note  g 为 NULL 时直接返回。
 */
void drv_gpio_in_init(drv_gpio_in_t *g, GPIO_TypeDef *port, uint16_t pin)
{
    if (!g) return;
    g->port  = port;
    g->pin   = pin;
    g->ready = true;
}

/**
 * @brief 读取 GPIO 输入电平。
 * @param g GPIO 输入对象指针
 * @return 1=高电平，0=低电平
 * @note  未初始化（g 为 NULL 或 ready=false）时返回 0，即按低电平处理。
 */
uint8_t drv_gpio_in_read(drv_gpio_in_t *g)
{
    if (!g || !g->ready) return 0;   /* 未初始化时返回低电平 */
    return (HAL_GPIO_ReadPin(g->port, g->pin) == GPIO_PIN_SET) ? 1u : 0u;
}