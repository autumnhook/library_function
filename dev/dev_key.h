/*
 * dev_key.h
 *
 *  设备层：独立按键（按下接地，需上拉）驱动，支持多实例。
 *  引脚初始化由 CubeMX 生成的 gpio.c（MX_GPIO_Init）完成，
 *  本模块只负责 扫描 + 消抖 + 按下事件，响应逻辑归应用层。
 *
 *  特性：
 *    - 支持多实例，每个对象绑定一个 GPIO 输入引脚
 *    - 固定节拍扫描消抖，连续 KEY_DEBOUNCE_TICKS 拍稳定新电平才更新状态
 *    - 按下沿产生一次性事件，dev_key_get_press 读后清除
 *    - 底层 GPIO 读取由 drv_gpio 输入对象封装
 *
 *  使用前提：
 *    1. 按键输入：按下 = 低电平（按键另一端接地），GPIO 需配置为上拉输入。
 *    2. 引脚初始化已在 CubeMX 生成的 gpio.c（MX_GPIO_Init）中完成。
 *    3. dev_key_scan 以固定节拍调用（主循环每拍一次，实际约 20ms，
 *       全系统唯一调用点在 main.c），连续 KEY_DEBOUNCE_TICKS 拍稳定的
 *       新电平才更新状态（消抖），抖动期间计数清零重来。
 *
 *  本工程实例（引脚初始化见 gpio.c）：
 *    g_tKey   PE3：操作键（菜单下移光标 / 循迹启动）
 *    g_tKeyOk PE4：确认进入功能 / 返回菜单
 *
 *  示例：
 *    // 全局实例（已在 dev_key.c 中定义）
 *    extern dev_key_t g_tKey;
 *    extern dev_key_t g_tKeyOk;
 *
 *    // 初始化（绑定具体引脚）
 *    dev_key_init(&g_tKey, GPIOE, GPIO_PIN_3, "KEY");
 *    dev_key_init(&g_tKeyOk, GPIOE, GPIO_PIN_4, "OK");
 *
 *    // 主循环或定时节拍中扫描
 *    dev_key_scan(&g_tKey);
 *    dev_key_scan(&g_tKeyOk);
 *
 *    // 读取按下事件
 *    if (dev_key_get_press(&g_tKey)) { ... }
 */

#ifndef DEV_DEV_KEY_H_
#define DEV_DEV_KEY_H_

#include "main.h"
#include "drv_gpio.h"          /* 引入 GPIO 输入封装 */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 硬件/消抖参数 ========== */
#define KEY_DEBOUNCE_TICKS  2              /* 消抖: 连续 N 拍(每拍约20ms)确认 */

/* ========== 对象定义 ========== */
typedef struct {
    drv_gpio_in_t gpio;      /* 输入引脚对象（封装底层 GPIO 读取） */
    const char   *name;      /* 按键名（保留接口兼容） */
    bool          stable;    /* 消抖后稳态: true = 按下 */
    uint8_t       filter_cnt;/* 消抖计数(电平翻转候选计数) */
    bool          press_evt; /* 按下事件(读后清除) */
} dev_key_t;

/* ========== 全局实例 ========== */
extern dev_key_t g_tKey;      /* PE3: 操作键(下移/步进/启动) */
extern dev_key_t g_tKeyOk;    /* PE4: 确认 / 返回 */

/* ========== 函数接口 ========== */

/**
 * @brief 初始化按键驱动对象。
 * @param self 按键对象指针（调用方持有全局实例）
 * @param port GPIO 端口，例如 GPIOE
 * @param pin  GPIO 引脚，例如 GPIO_PIN_3
 * @param name 按键名（保留接口兼容）
 * @note  该函数只绑定 GPIO 输入对象和名称，不负责 GPIO 硬件初始化；
 *        GPIO 初始化由 CubeMX 生成的 gpio.c（MX_GPIO_Init）完成。
 */
void dev_key_init(dev_key_t *self, GPIO_TypeDef *port, uint16_t pin, const char *name);

/**
 * @brief 按键扫描：消抖并产生按下事件。
 * @param self 按键对象指针
 * @note  需以固定节拍调用（主循环每拍一次，实际约 20ms）。
 *        连续 KEY_DEBOUNCE_TICKS 拍稳定的新电平才更新状态，
 *        抖动期间计数清零重来。
 */
void dev_key_scan(dev_key_t *self);

/**
 * @brief 读取一次“按下”事件。
 * @param self 按键对象指针
 * @return true 表示自上次读取后发生过按下事件；false 表示无事件。
 * @note  读后清除事件标志。
 */
bool dev_key_get_press(dev_key_t *self);

#ifdef __cplusplus
}
#endif

#endif /* DEV_DEV_KEY_H_ */