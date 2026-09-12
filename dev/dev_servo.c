/*
 * dev_servo.c
 *
 *  设备层实现：把"角度 -> 脉宽 -> PWM 输出"收进一个舵机驱动。
 *
 *  特性：
 *    - 角度存储用毫度 int32_t（current_angle_mdeg），避免整度类型在
 *      连续小步进时量化丢失；对外整度接口只做四舍五入换算。
 *    - 输入先四舍五入到整度：舵机按整度步进可靠响应，亚度级噪声只会让
 *      PWM 持续微调、舵机嗡嗡响，故直接丢弃。
 *    - 越界自动夹到 [0, SERVO_ANGLE_MAX_MDEG]。
 *    - 脉宽线性映射：pulse = 500 + angle_mdeg * 2000 / 180000（μs），
 *      即 0° -> 500μs，180° -> 2500μs，每度约 11.11μs。
 *
 *  分层原则：
 *    - 本文件不直接碰 HAL / TIM，只用 drv_pwm_t 的 set_duty / start。
 *    - 定时器周期与预分频由 CubeMX 配好（TIM5 @ 1MHz 计数、50Hz 周期），
 *      本层只管角度->脉宽换算，1 count = 1μs 是换算前提。
 *
 *  使用前提：
 *    1. 调用方需先 init 好 drv_pwm_t，且 TIM 已配为 1MHz 计数 / 50Hz 周期
 *       （ARR = 19999 -> 20ms）。
 *    2. drv_pwm_set_duty 的参数单位须为"计数"（= μs），与上一条匹配。
 */

#include "dev/dev_servo.h"

/* TIM5 @ 1MHz, 50Hz: 1 count = 1μs, ARR = 19999 (20ms).
 * 调用方在 CubeMX 配好 TIM5 后传入已 init 的 drv_pwm_t。 */
static drv_pwm_t *s_pwm;
static int32_t    current_angle_mdeg = 80000;

/* ============ 1. 初始化 ============ */

/**
 * @brief 初始化舵机。
 * @param pwm 已 init 的 PWM 驱动对象指针
 * @note  先把 PWM 输出设到默认 80°（上电安全位），再启动 PWM；顺序不能反，
 *        否则启动瞬间可能输出上一次残留的比较值。
 */
void dev_servo_init(drv_pwm_t *pwm)
{
    s_pwm = pwm;
    dev_servo_set_mdeg(current_angle_mdeg);
    drv_pwm_start(s_pwm);                  /* 替代 HAL_TIM_PWM_Start(&htim5, ...) */
}

/* ============ 2. 角度设置 ============ */

/**
 * @brief 按整度设置角度。
 * @param angle 目标角度（单位：度）
 * @note  转成毫度后走 dev_servo_set_mdeg，等效四舍五入到整度 + 越界夹紧。
 */
void dev_servo_set_angle(int16_t angle)
{
    dev_servo_set_mdeg((int32_t)angle * 1000);
}

/**
 * @brief 按毫度设置角度。
 * @param angle_mdeg 目标角度（单位：毫度）
 * @note  输入先四舍五入到整度（+500 再整除 1000）：硬件按整度步进可靠响应，
 *        拒绝亚度噪声，避免 PWM 持续微调引起舵机嗡嗡响。
 * @note  再夹到 [0, SERVO_ANGLE_MAX_MDEG]。
 * @note  脉宽换算 pulse = 500 + angle_mdeg * 2000 / 180000：
 *        angle_mdeg=0 -> 500μs；angle_mdeg=180000 -> 2500μs。
 *        分子先 *2000 再加 90000（=180000/2）是为了在整除前四舍五入，
 *        避免整数除法丢低位导致脉宽台阶抖动。
 */
void dev_servo_set_mdeg(int32_t angle_mdeg)
{
    uint32_t pulse;

    /* Hardware reads reliably in whole-degree steps; reject sub-degree noise. */
    angle_mdeg = ((angle_mdeg + 500) / 1000) * 1000;

    if (angle_mdeg < 0)
    {
        angle_mdeg = 0;
    }
    if (angle_mdeg > SERVO_ANGLE_MAX_MDEG)
    {
        angle_mdeg = SERVO_ANGLE_MAX_MDEG;
    }

    current_angle_mdeg = angle_mdeg;
    pulse = 500u + (((uint32_t)angle_mdeg * 2000u) + 90000u) / 180000u;

    drv_pwm_set_duty(s_pwm, pulse);        /* 替代 __HAL_TIM_SET_COMPARE(...) */
}

/* ============ 3. 角度读取 ============ */

/**
 * @brief 读取当前角度（整度）。
 * @return 当前角度（单位：度，毫度值四舍五入）
 */
int16_t dev_servo_get_angle(void)
{
    return (int16_t)((current_angle_mdeg + 500) / 1000);
}

/**
 * @brief 读取当前角度（毫度）。
 * @return 当前角度（单位：毫度，内部即整度值 * 1000）
 * @note  因 set_mdeg 已四舍五入到整度，返回值的末三位恒为 000。
 */
int32_t dev_servo_get_mdeg(void)
{
    return current_angle_mdeg;
}