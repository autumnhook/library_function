/*
 * lib_ringbuf.h
 *
 *  纯算法环形缓冲区：单生产者（ISR push）/ 单消费者（task pop），无锁安全。
 *
 *  特性：
 *    - 零平台依赖，仅使用 <stdint.h> 和 <stdbool.h>。
 *    - 使用 head/tail 两个索引管理读写，O(1) 入队出队。
 *    - 容量为 2 的幂时自动使用位掩码（&mask）快速回绕，否则回退取模（%size）。
 *    - 采用“留一个空位”策略区分空/满，有效容量为 size-1。
 *    - 利用无符号整数减法的自动回绕特性简化已用空间计数。
 *
 *  使用前提：
 *    1. 缓冲区内存由调用方提供（静态数组或动态分配均可）。
 *    2. 单生产者单消费者模型下无锁安全（例如中断写入、主循环读取）。
 *    3. 推荐容量为 2 的幂（如 64、256、512），可获得最佳性能。
 *
 *  示例：
 *    // 定义缓冲区和内存
 *    ringbuf_t rb;
 *    uint8_t   buf[256];
 *
 *    // 初始化
 *    ringbuf_init(&rb, buf, sizeof(buf));
 *
 *    // 中断中写入
 *    ringbuf_push(&rb, byte);
 *
 *    // 主循环中读取
 *    uint8_t data;
 *    if (ringbuf_pop(&rb, &data)) { ... }
 */

#ifndef LIB_LIB_RINGBUF_H_
#define LIB_LIB_RINGBUF_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 环形缓冲区对象 ========== */
typedef struct {
    uint8_t           *buf;     /* 外部提供内存（NULL 表示不启用） */
    volatile uint16_t  head;    /* 生产者写指针（ISR 中修改） */
    volatile uint16_t  tail;    /* 消费者读指针（主循环中修改） */
    uint16_t           size;    /* 缓冲区总容量（字节数） */
    uint16_t           mask;    /* size 为 2 的幂时 = size-1（走 &mask），否则 0（走 %size） */
} ringbuf_t;

/* ========== 函数接口 ========== */

/**
 * @brief 初始化环形缓冲区。
 * @param rb   环形缓冲区对象指针
 * @param mem  外部提供的缓冲区内存数组
 * @param size 缓冲区容量（字节数），推荐为 2 的幂
 * @note  初始化后 head/tail 均为 0，缓冲区为空。
 */
void ringbuf_init(ringbuf_t *rb, uint8_t *mem, uint16_t size);

/**
 * @brief 向缓冲区写入一个字节。
 * @param rb   环形缓冲区对象指针
 * @param byte 要写入的字节
 * @return true 表示写入成功；false 表示缓冲区已满。
 * @note  采用“留一个空位”策略判满，有效容量为 size-1。
 */
bool ringbuf_push(ringbuf_t *rb, uint8_t byte);

/**
 * @brief 从缓冲区读取一个字节。
 * @param rb   环形缓冲区对象指针
 * @param byte 输出参数，用于接收读取到的字节
 * @return true 表示读取成功；false 表示缓冲区为空。
 */
bool ringbuf_pop(ringbuf_t *rb, uint8_t *byte);

/**
 * @brief 获取当前缓冲区中已存储的字节数。
 * @param rb 环形缓冲区对象指针
 * @return 已存储的字节数（0 ~ size-1）
 * @note  使用无符号减法自动回绕特性，无需加锁。
 */
uint16_t ringbuf_count(ringbuf_t *rb);

/**
 * @brief 判断缓冲区是否为空。
 * @param rb 环形缓冲区对象指针
 * @return true 表示为空；false 表示非空。
 */
bool ringbuf_is_empty(ringbuf_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* LIB_LIB_RINGBUF_H_ */