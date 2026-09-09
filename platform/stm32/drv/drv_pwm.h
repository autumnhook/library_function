/*
 * drv_pwm.h
 *
 *  PWM 驱动层：把 STM32 HAL 的 TIM PWM 封装成一个对象。
 *
 *  特性：
 *    - 多实例支持：每个通道独立创建对象
 *    - 启动/停止带保护，避免重复操作
 *    - 提供计数值和百分比两种占空比设置
 *    - 初始化时自动读取周期，无需手动配置
 *
 *  使用前提：
 *    - 定时器已由 CubeMX 配置为 PWM 模式，并生成初始化代码
 *    - 若多个实例共享同一定时器，建议只由一个实例负责启动/停止
 *
 *  ========================== 使用示例 ==========================
 *
 *  // 1. 定义 PWM 对象（通常为全局变量）
 *  drv_pwm_t g_pwm_motor;
 *
 *  // 2. 初始化（假设使用 TIM4 的通道 1，定时器已在 CubeMX 中配置好）
 *  drv_pwm_init(&g_pwm_motor, &htim4, TIM_CHANNEL_1);
 *
 *  // 3. 设置占空比（二选一）
 *  drv_pwm_set_duty(&g_pwm_motor, 500);              // 计数值方式（0~period）
 *  drv_pwm_set_duty_percent(&g_pwm_motor, 50);       // 百分比方式（0~100）
 *
 *  // 4. 启动 PWM 输出
 *  if (!drv_pwm_start(&g_pwm_motor)) {
 *      // 启动失败处理
 *  }
 *
 *  // 5. 停止 PWM 输出
 *  drv_pwm_stop(&g_pwm_motor);
 *
 *  // 6. 多个通道示例（假设 TIM4_CH2 也用于另一个电机）
 *  drv_pwm_t g_pwm_motor2;
 *  drv_pwm_init(&g_pwm_motor2, &htim4, TIM_CHANNEL_2);
 *  drv_pwm_set_duty_percent(&g_pwm_motor2, 30);
 *  drv_pwm_start(&g_pwm_motor2);   // 注意：如果两个实例共用 htim4，建议只调用一次 start
 *  ==============================================================
 */

#ifndef DRV_DRV_PWM_H_
#define DRV_DRV_PWM_H_

#include "stm32f1xx_hal.h"   /* 根据实际芯片修改 */
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    TIM_HandleTypeDef *htim;      /* 定时器句柄 */
    uint32_t          channel;    /* 通道：TIM_CHANNEL_1 ~ TIM_CHANNEL_4 */
    uint32_t          period;     /* 周期计数值（= ARR） */
    bool              started;    /* 启动标志 */
} drv_pwm_t;

/* 初始化 PWM 驱动对象 */
void drv_pwm_init(drv_pwm_t *pwm, TIM_HandleTypeDef *htim, uint32_t channel);

/* 设置占空比（计数值） */
void drv_pwm_set_duty(drv_pwm_t *pwm, uint32_t duty);

/* 设置占空比（百分比 0~100） */
void drv_pwm_set_duty_percent(drv_pwm_t *pwm, uint8_t percent);

/* 启动 PWM 输出，成功返回 true */
bool drv_pwm_start(drv_pwm_t *pwm);

/* 停止 PWM 输出，成功返回 true */
bool drv_pwm_stop(drv_pwm_t *pwm);

#endif /* DRV_DRV_PWM_H_ */