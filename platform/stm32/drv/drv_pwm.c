/*
 * drv_pwm.c
 *
 *  PWM 驱动层实现（STM32 HAL 版本）
 *
 *  设计要点：
 *    - 通过结构体对象封装 PWM 的定时器句柄、通道和状态。
 *    - 启动/停止使用 started 标志防止重复调用 HAL 函数。
 *    - 百分比占空比计算使用 64 位中间值，避免 16/32 位乘法溢出。
 *    - 初始化时读取 ARR 寄存器作为周期，供后续限幅和百分比换算。
 *    - 启动/停止返回 bool 表示操作是否成功。
 *
 *  使用前提：
 *    - 定时器已由 CubeMX 配置为 PWM 模式，并生成初始化代码。
 *    - 本驱动不修改定时器基础配置（分频、周期、极性），只控制占空比与启停。
 */

#include "drv_pwm.h"

/* ========== 1. 初始化 PWM 驱动对象 ========== */
void drv_pwm_init(drv_pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel)
{
    // 保存定时器句柄和输出通道
    pwm->htim    = htim;
    pwm->channel = channel;

    // 记录初始状态：未启动
    pwm->started = false;

    // 从硬件 ARR 寄存器读回周期计数值（即 PWM 分辨率最大值）
    pwm->period  = __HAL_TIM_GET_AUTORELOAD(htim);
}

/* ========== 2. 设置占空比（原始计数值方式） ========== */
void drv_pwm_set_duty(drv_pwm_t *pwm, uint32_t duty)
{
    // 限幅：计数值不能超过周期最大值
    if (duty > pwm->period)
    {
        duty = pwm->period;
    }

    // 写入比较寄存器，立即更新占空比
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, duty);
}

/* ========== 3. 设置占空比（百分比方式，0~100） ========== */
void drv_pwm_set_duty_percent(drv_pwm_t *pwm, uint8_t percent)
{
    uint32_t duty;

    // 限制百分比范围，防止越界
    if (percent > 100)
    {
        percent = 100;
    }

    // 将百分比转换为计数值：
    // 使用 uint64_t 中间计算，避免 period * percent 溢出 32 位范围。
    duty = (uint32_t)((uint64_t)pwm->period * percent / 100);

    // 调用原始计数值接口完成设置
    drv_pwm_set_duty(pwm, duty);
}

/* ========== 4. 启动 PWM 输出 ========== */
bool drv_pwm_start(drv_pwm_t *pwm)
{
    // 如果已经启动，直接返回成功，避免重复启动
    if (pwm->started)
    {
        return true;
    }

    // 调用 HAL 启动函数，并检查返回值
    if (HAL_TIM_PWM_Start(pwm->htim, pwm->channel) == HAL_OK)
    {
        pwm->started = true;   // 启动成功，更新状态
        return true;
    }

    // 启动失败，保持未启动状态
    return false;
}

/* ========== 5. 停止 PWM 输出 ========== */
bool drv_pwm_stop(drv_pwm_t *pwm)
{
    // 如果已经停止，直接返回成功
    if (!pwm->started)
    {
        return true;
    }

    // 调用 HAL 停止函数，并检查返回值
    if (HAL_TIM_PWM_Stop(pwm->htim, pwm->channel) == HAL_OK)
    {
        pwm->started = false;  // 停止成功，更新状态
        return true;
    }

    // 停止失败，保持原状态
    return false;
}