/*
 * dev_track.c
 *
 * 设备层：8 路灰度/巡线传感器读取与误差计算。
 *
 * 设计要点：
 *   - 读取：一次读 GPIOF->IDR 低 8 位，无需移位时序。
 *   - 归一化：根据 TRACK_LINE_ACTIVE 统一成 “1=压线”，屏蔽有效电平差异。
 *   - 位序镜像：TRACK_BIT_MIRROR 打开时做 bit0<->bit7 反转，适配反装。
 *   - 边缘优先：1/2/7/8 触发时直接取最大误差级，不做平均，便于快速回正。
 *   - 中间平均：3~6 触发时按位平均，输出平滑误差（如 {3,4} -> -2）。
 *   - 十字/横线：cnt>=4 连续 TRACK_LINE_ALL_CONFIRM 拍才确认，避免误判。
 *   - 丢线：line==0 时保持上次误差，避免瞬间跳变。
 *
 * 使用前提：
 *   1. dev_track_update_isr() 必须在固定周期中断中调用（推荐 TIM1 10ms）。
 *   2. TRACK_SENSOR_PORT / TRACK_SENSOR_MASK / TRACK_LINE_ACTIVE 必须与硬件一致。
 *   3. 反装或位序相反时，通过 TRACK_BIT_MIRROR 修正，无需改接线。
 *
 * 误差映射：
 *   第 1 路 -> -7   第 8 路 -> +7
 *   第 2 路 -> -5   第 7 路 -> +5
 *   第 3 路 -> -3   第 6 路 -> +3
 *   第 4 路 -> -1   第 5 路 -> +1
 *   4+5 同时压线 -> 0（居中）
 */

#include "dev_track.h"

dev_track_t g_tTrack;

/* 8 路传感器对应的误差等级，索引 0..7 对应 bit0..bit7。
 * 中间 4、5 同时压线时平均为 0，表示居中。 */
static const int8_t s_grade[8] = {-7, -5, -3, -1, +1, +3, +5, +7};

/* ========== 内部辅助 ========== */

/**
 * @brief 8 位按位镜像：bit0<->bit7, bit1<->bit6, ...
 * @param v 输入位图
 * @return 镜像后的位图
 * @note  仅当 TRACK_BIT_MIRROR 打开时调用。
 */
static uint8_t mirror_bits(uint8_t v)
{
    uint8_t r = 0;
    uint8_t i;
    for (i = 0; i < 8; i++)
        if (v & (uint8_t)(1u << i))
            r |= (uint8_t)(1u << (7 - i));
    return r;
}

/* ========== API 实现 ========== */

void dev_track_init(dev_track_t *self)
{
    // 状态清零，error 初始为 0
    self->sensors        = 0;
    self->error          = 0;
    self->line_all       = 0;
    self->line_all_ticks = 0;
}

/**
 * @brief 读取传感器并计算误差。
 * @param self 状态对象指针
 *
 * 处理流程：
 *   1. 读 GPIOF->IDR 低 8 位，按 TRACK_LINE_ACTIVE 归一化为 “1=压线”。
 *   2. 若 TRACK_BIT_MIRROR 打开，做位序镜像。
 *   3. line==0：丢线，保持上次 error，返回。
 *   4. cnt>=4：疑似十字/横线，连续确认 TRACK_LINE_ALL_CONFIRM 拍后置 line_all=1，error=0。
 *   5. 边缘路（1/2/7/8）触发：直接取对应等级，不做平均。
 *   6. 中间路（3~6）触发：按位平均输出平滑误差。
 */
void dev_track_update_isr(dev_track_t *self)
{
    uint8_t raw  = (uint8_t)(TRACK_SENSOR_PORT->IDR & TRACK_SENSOR_MASK);
    uint8_t line;
    uint8_t cnt;
    uint8_t i;
    int8_t  err;

    // 归一化：统一成 “1=压线”
#if TRACK_LINE_ACTIVE
    line = raw;
#else
    line = (uint8_t)(~raw) & TRACK_SENSOR_MASK;
#endif

    // 位序镜像（反装时启用）
#if TRACK_BIT_MIRROR
    line = mirror_bits(line);
#endif

    self->sensors = line;

    // 丢线：保持上次 error，避免跳变
    if (line == 0)
    {
        self->line_all       = 0;
        self->line_all_ticks = 0;
        return;
    }

    // 统计触发路数
    cnt = 0;
    for (i = 0; i < 8; i++)
        if (line & (uint8_t)(1u << i))
            cnt++;

    // 十字/横线：cnt>=4 需连续确认，防止急弯斜切误判
    if (cnt >= 4)
    {
        if (self->line_all_ticks < TRACK_LINE_ALL_CONFIRM)
            self->line_all_ticks++;
        if (self->line_all_ticks >= TRACK_LINE_ALL_CONFIRM)
        {
            self->line_all = 1;
            self->error    = 0;
            return;
        }
        self->line_all = 0;         // 确认窗口内：不置位，error 保持
        return;
    }
    self->line_all       = 0;
    self->line_all_ticks = 0;

    // 误差计算：
    //   边缘路（1/2/7/8）触发 -> 直接取边缘等级，便于快速回正
    //   中间路（3~6）触发    -> 按位平均，输出平滑误差
    if (line & 0xC3u)
    {
        if      (line & 0x01u) err = s_grade[0];   // 第 1 路 -> -7
        else if (line & 0x80u) err = s_grade[7];   // 第 8 路 -> +7
        else if (line & 0x02u) err = s_grade[1];   // 第 2 路 -> -5
        else                   err = s_grade[6];   // 第 7 路 -> +5
    }
    else
    {
        int16_t sum = 0;
        for (i = 0; i < 8; i++)
            if (line & (uint8_t)(1u << i))
                sum += s_grade[i];
        err = (int8_t)(sum / cnt);                 // cnt 前面已统计，非 0
    }

    self->error = err;
}