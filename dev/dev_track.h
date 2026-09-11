/*
 * dev_track.h
 *
 * 设备层：8 路灰度/巡线传感器接口（序号 1~8 = PF0~PF7）。
 *
 * 设计要点：
 *   - 传感器直接接到 PF0~PF7，一次读 GPIOF->IDR 低 8 位即可，无移位时序开销。
 *   - 极性/方向通过宏配置：TRACK_LINE_ACTIVE 控制有效电平，TRACK_BIT_MIRROR 控制位序镜像。
 *   - 误差输出为 int8_t，范围 -7 ~ +7，与电机差速控制直接对接。
 *   - 十字/横线判定需连续 TRACK_LINE_ALL_CONFIRM 拍确认，避免急弯斜切误判。
 *   - 状态量均在 ISR 中更新，字段加 volatile，主循环可直接读。
 *
 * 使用前提：
 *   1. 传感器端口与引脚掩码通过 TRACK_SENSOR_PORT / TRACK_SENSOR_MASK 配置。
 *   2. 有效电平和位序需与实际硬件匹配，否则会读反。
 *   3. dev_track_update_isr() 必须放在固定周期中断中调用（推荐 TIM1 10ms）。
 *   4. 全局实例 g_tTrack 由本模块提供，main.c 可直接引用。
 *
 * 位序约定：
 *   bit0 = PF0 = 传感器 1 ... bit7 = PF7 = 传感器 8
 *   归一化后：bit = 1 表示该路压线/检测到有效信号
 *
 * 误差约定：
 *   error < 0 : 线偏左（对应 -1/-3/-5/-7）
 *   error = 0 : 4+5 同时压线，居中
 *   error > 0 : 线偏右（对应 +1/+3/+5/+7）
 *
 * 示例：
 *   dev_track_init(&g_tTrack);
 *
 *   // TIM1 10ms 中断中：
 *   dev_track_update_isr(&g_tTrack);
 *
 *   // 主循环中：
 *   int8_t  err   = g_tTrack.error;
 *   uint8_t cross = g_tTrack.line_all;
 */

#ifndef DEV_DEV_TRACK_H_
#define DEV_DEV_TRACK_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 硬件/配置宏 ========== */
#define TRACK_SENSOR_PORT       GPIOF      /* 8 路传感器所在端口               */
#define TRACK_SENSOR_MASK       0x00FFu    /* PF0~PF7                         */
#define TRACK_LINE_ACTIVE       0          /* 有效电平：0=低电平有效，1=高有效 */
#define TRACK_BIT_MIRROR        0          /* 1=镜像 bit0<->bit7，适配反装    */

/* line_all 确认拍数：cnt>=4 连续 N 拍后置 line_all=1。
 * 急弯斜切回线时会出现 1~2 拍假横线，单拍就确认会导致提前停车。 */
#define TRACK_LINE_ALL_CONFIRM  3

/* ========== 巡线状态 ========== */
typedef struct {
    volatile uint8_t sensors;        /* 归一化位图：1=压线，bit0=传感器1..bit7=传感器8 */
    volatile int8_t  error;          /* 误差 -7..+7，0=4+5居中；丢线时保持上次值      */
    volatile uint8_t line_all;       /* 1=十字/终点线（连续确认后置位）              */
    volatile uint8_t line_all_ticks; /* 连续 cnt>=4 的拍数计数（ISR 内累加）          */
} dev_track_t;

/* 全局实例 */
extern dev_track_t g_tTrack;

/* ========== API ========== */

/**
 * @brief 初始化巡线状态。
 * @param self 状态对象指针
 * @note  所有字段清零，error 初始为 0。
 */
void dev_track_init(dev_track_t *self);

/**
 * @brief 读取传感器并计算误差（中断中调用）。
 * @param self 状态对象指针
 * @note  推荐在固定周期中断中调用（如 TIM1 10ms），内部不做阻塞延时。
 * @note  内部完成：读 IDR -> 归一化 -> 位序镜像 -> 十字判定 -> 误差计算。
 */
void dev_track_update_isr(dev_track_t *self);

#ifdef __cplusplus
}
#endif

#endif /* DEV_DEV_TRACK_H_ */