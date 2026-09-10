/*
 * dev_key.c
 *
 *  设备层实现：独立按键（按下接地）驱动 + 消抖 + 按下事件检测。
 *
 *  设计要点：
 *    - 每个按键一个 dev_key_t 实例，dev_key_init 绑定端口/引脚/名字。
 *    - 引脚初始化由 gpio.c（CubeMX 生成）完成，本模块只做扫描逻辑。
 *    - 固定节拍扫描，连续 KEY_DEBOUNCE_TICKS 拍一致才确认（消抖）。
 *    - 释放 -> 按下 沿产生一次性事件，dev_key_get_press 读后清除。
 *
 *  使用前提：
 *    - 按键输入：按下 = 低电平（按键另一端接地），GPIO 需配置为上拉输入。
 *    - dev_key_scan 由主循环每拍调用一次（实际节拍约 20ms，
 *      全系统唯一调用点在 main.c）。
 *    - 事件由应用层通过 dev_key_get_press 消费。
 *
 *  本工程实例：
 *    g_tKey   PE3：操作键（菜单下移光标 / 循迹启动）
 *    g_tKeyOk PE4：确认进入功能 / 返回菜单
 */

#include "dev_key.h"

/* ========== 全局实例 ========== */
dev_key_t g_tKey;      /* PE3: 操作键 */
dev_key_t g_tKeyOk;    /* PE4: 确认 / 返回 */

/* ========== 1. 初始化按键驱动对象 ========== */
void dev_key_init(dev_key_t *self, GPIO_TypeDef *port, uint16_t pin, const char *name)
{
    if (!self) return;

    // 初始化输入引脚对象
    drv_gpio_in_init(&self->gpio, port, pin);

    self->name       = name;
    self->stable     = false;    // 上电默认释放
    self->filter_cnt = 0;
    self->press_evt  = false;
}

/* ========== 2. 按键扫描：消抖 + 按下事件 ========== */
void dev_key_scan(dev_key_t *self)
{
    if (!self) return;

    // 读取引脚电平：按下接地，低电平表示按下
    bool now = (drv_gpio_in_read(&self->gpio) == 0u);

    if (now != self->stable)
    {
        // 电平翻转候选：连续 N 拍一致才确认（消抖）
        if (++self->filter_cnt >= KEY_DEBOUNCE_TICKS)
        {
            self->stable     = now;
            self->filter_cnt = 0;

            if (now)
            {
                self->press_evt = true;   // 释放 -> 按下 沿
            }
        }
    }
    else
    {
        self->filter_cnt = 0;
    }
}

/* ========== 3. 读取一次“按下”事件（读后清除） ========== */
bool dev_key_get_press(dev_key_t *self)
{
    if (!self) return false;

    bool evt = self->press_evt;
    self->press_evt = false;
    return evt;
}