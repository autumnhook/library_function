/*
 * drv_encoder.h
 *
 *  编码器驱动层：将 STM32 HAL 的 TIM 编码器模式封装为一个轻量对象。
 *
 *  特性：
 *    - 支持 16 位或 32 位定时器（通过修改 enc_count_t 类型切换）
 *    - 增量计算使用无符号整数自动回绕，无需显式溢出判断
 *    - 内部记录启动状态，防止重复启动
 *    - 初始化时自动捕获当前计数值，避免首次读取产生虚假增量
 *
 *  使用前提：
 *    1. 定时器已在 CubeMX 中配置为编码器模式，并生成初始化代码。
 *    2. 调用 drv_encoder_get_delta() 的频率必须保证两次调用之间
 *       计数器增量不超过 enc_count_t 的最大值（16位：65535，32位：2^32-1）。
 *       通常 1~10ms 的轮询周期完全满足。
 *
 *  示例：
 *    // 全局定义
 *    drv_encoder_t g_enc;
 *
 *    // 初始化（假设编码器接在 TIM2）
 *    drv_encoder_open(&g_enc, &htim2);
 *
 *    // 启动计数
 *    drv_encoder_enable(&g_enc);
 *
 *    // 在主循环或定时中断中读取增量
 *    uint32_t delta = drv_encoder_get_delta(&g_enc);
 *    if (delta > 0) { ... }
 */

#ifndef DRV_DRV_ENCODER_H_
#define DRV_DRV_ENCODER_H_

#include "stm32f1xx_hal.h"   /* 根据具体芯片型号修改，例如 stm32f4xx_hal.h */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 计数器位宽配置 ==========
 * 根据所用定时器的位数选择对应的类型。
 *  - 16 位定时器（TIM3、TIM4 等）: typedef uint16_t enc_count_t;
 *  - 32 位定时器（TIM2、TIM5 等）: typedef uint32_t enc_count_t;
 */
typedef uint16_t enc_count_t;      /* 默认 16 位，如使用 32 位请改为 uint32_t */

/* ========== 编码器对象 ========== */
typedef struct {
    TIM_HandleTypeDef *htim;        /* 定时器句柄（编码器模式已由 CubeMX 配置） */
    enc_count_t        last_count;  /* 上一次读取的计数值（用于计算增量） */
    bool               started;     /* 标记是否已启动，防止重复调用 HAL_TIM_Encoder_Start */
} drv_encoder_t;

/* ========== 函数接口 ========== */

/**
 * @brief 初始化编码器驱动对象。
 * @param enc  驱动对象指针（调用方持有全局实例）
 * @param htim CubeMX 生成的定时器句柄（例如 &htim2）
 * @note  该函数不会启动计数器，只保存句柄并捕获当前计数值作为基准。
 */
void drv_encoder_open(drv_encoder_t *enc, TIM_HandleTypeDef *htim);

/**
 * @brief 启动编码器计数。
 * @param enc 驱动对象指针
 * @note  重复调用会被忽略。
 */
void drv_encoder_enable(drv_encoder_t *enc);

/**
 * @brief 读取当前计数值（绝对值）。
 * @param enc 驱动对象指针
 * @return 当前计数器值（0 ~ enc_count_t 最大值）
 */
enc_count_t drv_encoder_get_count(drv_encoder_t *enc);

/**
 * @brief 计算自上次调用以来的脉冲增量，并更新基准。
 * @param enc 驱动对象指针
 * @return 增量脉冲数（自动处理计数器回绕）
 * @note  调用间隔必须保证计数器增量不超过 enc_count_t 最大值。
 */
uint32_t drv_encoder_get_delta(drv_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* DRV_DRV_ENCODER_H_ */