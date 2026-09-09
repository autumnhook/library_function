/*
 * drv_encoder.c
 *
 *  编码器驱动层实现（STM32 HAL 版本）
 *
 *  设计要点：
 *    - 利用无符号整数减法的自动回绕特性，简化增量计算，无需显式溢出判断。
 *    - 启动保护：通过 started 标志避免重复调用 HAL_TIM_Encoder_Start。
 *    - 初始化时捕获当前计数值，防止首次读取产生虚假大增量。
 *
 *  使用前提：
 *    - 定时器已由 CubeMX 配置为编码器模式（对应 TIMx 初始化代码已生成）。
 *    - 调用 drv_encoder_get_delta() 的频率必须保证两次调用之间计数器增量
 *      不超过 enc_count_t 的最大值（16位：65535，32位：2^32-1）。
 */

#include "drv_encoder.h"

/* ========== 1. 初始化编码器驱动对象 ========== */
void drv_encoder_open(drv_encoder_t *enc, TIM_HandleTypeDef *htim)
{
    // 保存定时器句柄
    enc->htim = htim;

    // 读取当前计数值作为初始基准，避免第一次调用 get_delta 时产生虚假增量
    enc->last_count = (enc_count_t)__HAL_TIM_GET_COUNTER(htim);

    // 标记为未启动，等待 enable 调用
    enc->started = false;
}

/* ========== 2. 启动编码器计数 ========== */
void drv_encoder_enable(drv_encoder_t *enc)
{
    // 如果尚未启动，则启动编码器模式（两相同时使能）
    if (!enc->started)
    {
        HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
        enc->started = true;  // 记录启动状态，防止重复启动
    }
}

/* ========== 3. 读取当前绝对值（快照） ========== */
enc_count_t drv_encoder_get_count(drv_encoder_t *enc)
{
    // 直接从硬件计数器寄存器读取当前值，并转换为 enc_count_t 类型
    return (enc_count_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

/* ========== 4. 计算增量（核心逻辑，自动回绕） ========== */
uint32_t drv_encoder_get_delta(drv_encoder_t *enc)
{
    // 读取当前计数值
    enc_count_t current = drv_encoder_get_count(enc);

    // 无符号减法自动处理计数器回绕：
    // 若 current < last_count，结果会被解释为一个很大的无符号数，
    // 但实际值 = 2^n + current - last_count，正是我们需要的增量。
    enc_count_t delta = current - enc->last_count;

    // 更新基准值供下次使用
    enc->last_count = current;

    // 返回增量，转换为 uint32_t 方便上层使用
    return (uint32_t)delta;
}