/*
 * lib_filter.h
 *
 *  纯算法信号滤波：3 点中值 / EWMA / 滑动平均。
 *
 *  特性：
 *    - 零平台依赖，仅使用 <stdint.h> 和 <stdbool.h>，不依赖任何 HAL / RTOS / 驱动层符号。
 *    - 全部 int16_t 数据类型，与现有轮速数据类型一致。
 *    - 中值滤波：使用 primed 标志判启动并首拍填满 3 格，避免原“判 0 当未初始化”的缺陷。
 *    - EWMA：一阶低通，右移实现整数除法，避免浮点运算；从 0 起算，shift 限制 0~15。
 *    - 滑动平均：环形缓冲区 + 运行和 O(1) 更新，支持任意窗口大小。
 *
 *  使用前提：
 *    1. 滑动平均需由调用方提供缓冲区内存（int16_t 数组），大小任意正整数。
 *    2. EWMA 的 shift 参数应在 0~15 之间（超过自动截断为 15）。
 *    3. 滤波器结构体由调用方持有，初始化后方可使用。
 *
 *  示例：
 *    // 中值滤波
 *    filter_median_t med;
 *    filter_median_init(&med);
 *    int16_t med_out = filter_median_push(&med, adc_raw);
 *
 *    // EWMA 滤波
 *    filter_ewma_t ewma;
 *    filter_ewma_init(&ewma, 1);   // alpha = 1/2
 *    int16_t ewma_out = filter_ewma_update(&ewma, input);
 *
 *    // 滑动平均（窗口大小 8）
 *    int16_t movavg_buf[8];
 *    filter_movavg_t movavg;
 *    filter_movavg_init(&movavg, movavg_buf, 8);
 *    int16_t movavg_out = filter_movavg_push(&movavg, input);
 */

#ifndef LIB_LIB_FILTER_H_
#define LIB_LIB_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 3 点中值滤波 ========== */
typedef struct {
    int16_t buf[3];       /* 滑动窗口（3 个样本） */
    uint8_t idx;          /* 当前写入位置（0~2） */
    bool    primed;       /* 首拍填满 3 格，之后正常轮转 */
} filter_median_t;

/**
 * @brief 初始化 3 点中值滤波器。
 * @param f 滤波器对象指针
 * @note  初始化后内部缓冲区清零，primed 置 false。
 */
void    filter_median_init (filter_median_t *f);

/**
 * @brief 输入一个样本，返回中值滤波结果。
 * @param f 滤波器对象指针
 * @param v 当前样本值
 * @return 3 点中值
 * @note  首次输入会用当前值填满窗口，避免输出初始零。
 */
int16_t filter_median_push (filter_median_t *f, int16_t v);

/* ========== 2. EWMA 指数加权移动平均 ========== */
typedef struct {
    int16_t value;        /* 当前滤波输出 */
    uint8_t shift;        /* 右移位数，alpha = 1/2^shift（0~15） */
} filter_ewma_t;

/**
 * @brief 初始化 EWMA 滤波器。
 * @param f     滤波器对象指针
 * @param shift 右移位数（0~15），值越大平滑越强
 * @note  shift 超过 15 时自动截断为 15。
 * @note  初始滤波值为 0。
 */
void    filter_ewma_init   (filter_ewma_t *f, uint8_t shift);

/**
 * @brief 重置 EWMA 滤波器值为指定值。
 * @param f 滤波器对象指针
 * @param v 要设置的当前滤波值
 * @note  用于超时等场景同步滤波器状态。
 */
void    filter_ewma_reset  (filter_ewma_t *f, int16_t v);

/**
 * @brief 输入一个样本，返回 EWMA 滤波结果。
 * @param f 滤波器对象指针
 * @param v 当前样本值
 * @return 滤波后的值
 * @note  公式：value += (v - value) >> shift，避免了浮点运算。
 */
int16_t filter_ewma_update (filter_ewma_t *f, int16_t v);

/* ========== 3. 滑动平均滤波 ========== */
typedef struct {
    int16_t *buf;         /* 窗口缓冲区（由调用方提供） */
    uint16_t size;        /* 窗口长度 */
    uint16_t idx;         /* 当前写入位置 */
    int32_t  sum;         /* 窗口内样本和（int32 防止溢出） */
    bool     filled;      /* 窗口是否已填满 */
} filter_movavg_t;

/**
 * @brief 初始化滑动平均滤波器。
 * @param f    滤波器对象指针
 * @param mem  外部提供的缓冲区内存（长度至少 size）
 * @param size 窗口长度（>0）
 * @note  若 mem 为 NULL 或 size 为 0，滤波器无效，push 会直接返回输入值。
 * @note  初始化时会将缓冲区清零。
 */
void    filter_movavg_init (filter_movavg_t *f, int16_t *mem, uint16_t size);

/**
 * @brief 输入一个样本，返回滑动平均滤波结果。
 * @param f 滤波器对象指针
 * @param v 当前样本值
 * @return 滑动平均值
 * @note  填窗期（前 size-1 次）返回当前输入值，避免从 0 爬升；
 *        之后返回窗口内所有样本的算术平均值。
 */
int16_t filter_movavg_push (filter_movavg_t *f, int16_t v);

#ifdef __cplusplus
}
#endif

#endif /* LIB_LIB_FILTER_H_ */