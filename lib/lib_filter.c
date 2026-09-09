/*
 * lib_filter.c
 *
 *
 *  设计要点：
 *    - 中值滤波：3 点窗口，循环覆盖，直接比较取中位，无排序开销。
 *    - EWMA：一阶低通，右移实现整数除法，避免浮点；shift 限制 0~15 防位移 UB。
 *    - 滑动平均：环形窗口 + 运行和 O(1) 更新；填窗期返回当前值，避免从 0 爬升。
 *    - 所有输入输出均为 int16_t，适用于传感器、ADC 等整数数据。
 *
 *  使用前提：
 *    - 滑动平均需由调用方提供缓冲区内存（mem 数组），大小任意正整数。
 *    - EWMA 的 shift 参数应在 0~15 之间（超过自动截断为 15）。
 */

#include "lib_filter.h"
#include <stddef.h>    /* NULL */

/* ========== 1. 3 点中值滤波 ========== */

void filter_median_init(filter_median_t *f)
{
    // 初始填充 0，primed 置 false
    f->buf[0] = f->buf[1] = f->buf[2] = 0;
    f->idx    = 0;
    f->primed = false;
}

int16_t filter_median_push(filter_median_t *f, int16_t v)
{
    // 首次输入时用当前值填满窗口，避免输出初始零
    if (!f->primed)
    {
        f->buf[0] = v; f->buf[1] = v; f->buf[2] = v;
        f->idx = 0;
        f->primed = true;
        return v;
    }

    // 覆盖最旧值并推进循环索引
    f->buf[f->idx] = v;
    uint8_t ni = (uint8_t)(f->idx + 1u);
    if (ni >= 3u) ni = 0u;
    f->idx = ni;

    // 直接比较取中位：a 介于 b、c 之间 → a；否则 b 介于 a、c → b；否则 c
    int16_t a = f->buf[0], b = f->buf[1], c = f->buf[2];
    if ((a <= b && a >= c) || (a <= c && a >= b)) return a;
    if ((b <= a && b >= c) || (b <= c && b >= a)) return b;
    return c;
}

/* ========== 2. EWMA 指数加权移动平均 ========== */

void filter_ewma_init(filter_ewma_t *f, uint8_t shift)
{
    // shift 上限 15，防止 shift>=16 时 32 位移位产生未定义行为
    if (shift > 15u) shift = 15u;
    f->shift = shift;
    f->value = 0;
}

void filter_ewma_reset(filter_ewma_t *f, int16_t v)
{
    // 直接设置当前滤波值
    f->value = v;
}

int16_t filter_ewma_update(filter_ewma_t *f, int16_t v)
{
    // value + (diff >> shift) 介于旧值与新值之间，int16 不会溢出
    int32_t diff = (int32_t)v - (int32_t)f->value;
    f->value = (int16_t)(f->value + (diff >> f->shift));
    return f->value;
}

/* ========== 3. 滑动平均滤波 ========== */

void filter_movavg_init(filter_movavg_t *f, int16_t *mem, uint16_t size)
{
    f->buf    = mem;
    f->size   = size;
    f->idx    = 0;
    f->sum    = 0;
    f->filled = false;

    // 缓冲区有效时清零窗口
    if (mem != NULL && size > 0u)
    {
        for (uint16_t i = 0u; i < size; i++) mem[i] = 0;
    }
}

int16_t filter_movavg_push(filter_movavg_t *f, int16_t v)
{
    // 缓冲区无效时直接返回输入
    if (f->buf == NULL || f->size == 0u)
        return v;

    // 填窗期：累加并填满窗口，返回当前值避免从 0 爬升
    if (!f->filled)
    {
        f->buf[f->idx] = v;
        f->sum += v;
        f->idx++;
        if (f->idx >= f->size)
        {
            f->idx = 0u;
            f->filled = true;
        }
        return v;
    }

    // 稳态：减旧加新，O(1) 更新运行和
    int16_t old = f->buf[f->idx];
    f->sum += (int32_t)v - (int32_t)old;
    f->buf[f->idx] = v;

    // 推进环形索引
    f->idx++;
    if (f->idx >= f->size) f->idx = 0u;

    // 返回窗口均值
    return (int16_t)(f->sum / (int32_t)f->size);
}