/*
 * dev_servo.h
 *
 *  设备层：SG90/MG996R 类舵机（180° 位置舵机）驱动。
 *
 *  特性：
 *    - 角度双 API：整度接口（int16_t，-0 ~ 180）与毫度接口（int32_t，
 *      0 ~ 180000），内部统一用毫度存储，整度只做四舍五入换算。
 *    - 硬件按整度步进可靠响应，毫度接口内部会先把输入四舍五入到整度，
 *      拒绝亚度噪声（避免 0.001° 级抖动引起 PWM 持续微调）。
 *    - 越界输入自动夹到 [0, SERVO_ANGLE_MAX_MDEG]，不报错、不崩。
 *    - 脉宽由硬件 50Hz 定时器直接给出：0° -> 500µs，180° -> 2500µs，
 *      线性映射（每度约 11.11µs）。
 *
 *  分层原则：
 *    - 本模块只依赖 drv_pwm，不直接碰 HAL / TIM。
 *    - 定时器配置（预分频 / ARR）由 CubeMX 完成，本层只把"角度 -> 脉宽"
 *      换算出来交给 drv_pwm_set_duty。
 *
 *  使用前提：
 *    1. 调用方需先 init 好一个 drv_pwm_t（TIM 应配为 1MHz 计数、50Hz 周期，
 *       即 1 count = 1µs，ARR = 19999 -> 20ms 周期）。
 *    2. drv_pwm_set_duty 的参数语义须与 TIM 计数一致（本层传的是 µs 计数）。
 *
 *  示例：
 *    // 初始化（CubeMX 已配好 TIM5）
 *    dev_servo_init(&pwm_servo);
 *
 *    // 设置角度
 *    dev_servo_set_angle(90);           // 90°
 *    dev_servo_set_mdeg(90500);         // 90.5°（会被四舍五入到 91°）
 *
 *    // 读取
 *    int16_t deg = dev_servo_get_angle();
 *    int32_t mdeg = dev_servo_get_mdeg();
 */

#ifndef DEV_DEV_SERVO_H_
#define DEV_DEV_SERVO_H_

#include "drv/drv_pwm.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 编译期参数 ========== */

#define SERVO_ANGLE_MAX_MDEG  180000   /* 180° */
#define SERVO_PULSE_MIN_US    500u     /* 0°   -> 500 µs */
#define SERVO_PULSE_MAX_US    2500u    /* 180° -> 2500 µs */

/* ========== 2. API ========== */

/**
 * @brief 初始化舵机。
 * @param pwm 已 init 的 PWM 驱动对象指针（TIM 配为 1MHz 计数 / 50Hz 周期）
 * @note  内部会把 current_angle_mdeg 复位到默认 80°（上电安全位），并立即
 *        输出该角度、启动 PWM。
 */
void    dev_servo_init(drv_pwm_t *pwm);

/**
 * @brief 按整度设置角度。
 * @param angle 目标角度（单位：度，负值/超 180 会被夹到 [0,180]）
 * @note  内部转成毫度后走 dev_servo_set_mdeg，等效四舍五入到整度。
 */
void    dev_servo_set_angle(int16_t angle);

/**
 * @brief 按毫度设置角度。
 * @param angle_mdeg 目标角度（单位：毫度，0 ~ 180000）
 * @note  输入先四舍五入到整度（硬件按整度步进可靠响应，拒绝亚度噪声），
 *        再夹到 [0, SERVO_ANGLE_MAX_MDEG]，然后换算脉宽并写 PWM。
 */
void    dev_servo_set_mdeg(int32_t angle_mdeg);

/**
 * @brief 读取当前角度（整度）。
 * @return 当前角度（单位：度，毫度值四舍五入）
 */
int16_t dev_servo_get_angle(void);

/**
 * @brief 读取当前角度（毫度）。
 * @return 当前角度（单位：毫度，内部即整度值 * 1000）
 */
int32_t dev_servo_get_mdeg(void);

#ifdef __cplusplus
}
#endif

#endif /* DEV_DEV_SERVO_H_ */