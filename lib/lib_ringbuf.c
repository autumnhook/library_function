/*
 * lib_ringbuf.c
 *
 *
 *  设计要点：
 *    - 使用 head/tail 两个索引管理读写，O(1) 入队出队。
 *    - 容量为 2 的幂时自动使用位掩码（&mask）快速回绕，
 *      否则回退到取模运算（%size），兼顾性能与通用性。
 *    - 采用“留一个空位”策略区分空/满状态，有效容量为 size-1。
 *    - 利用无符号整数减法的自动回绕特性简化已用空间计数。
 *
 *  使用前提：
 *    - 缓冲区内存由调用方提供，大小可为任意正整数（推荐 2 的幂）。
 *    - 单生产者单消费者模型下无锁安全（例如中断写入、主循环读取）。
 */

#include "lib_ringbuf.h"
#include <stddef.h>

/* ========== 1. 初始化环形缓冲区 ========== */
void ringbuf_init(ringbuf_t *rb, uint8_t *mem, uint16_t size)
{
    rb->buf  = mem;
    rb->head = 0;
    rb->tail = 0;
    rb->size = size;
    /* 大小为 2 的幂时用 &mask 快速回卷，否则回退 %size */
    rb->mask = ((size != 0U) && ((size & (size - 1U)) == 0U)) ? (uint16_t)(size - 1U) : 0U;
}

/* ========== 2. 索引推进（自动回绕） ========== */
static uint16_t ringbuf_advance(const ringbuf_t *rb, uint16_t idx)
{
    if (rb->mask != 0U)
        return (uint16_t)((idx + 1U) & rb->mask);
    else
        return (uint16_t)((idx + 1U) % rb->size);
}

/* ========== 3. 入队一个字节 ========== */
bool ringbuf_push(ringbuf_t *rb, uint8_t byte)
{
    if (rb->buf == NULL || rb->size == 0U)
        return false;

    uint16_t next = ringbuf_advance(rb, rb->head);
    if (next == rb->tail)          /* 满（留 1 空位判满，有效容量 = size-1）*/
        return false;

    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

/* ========== 4. 出队一个字节 ========== */
bool ringbuf_pop(ringbuf_t *rb, uint8_t *byte)
{
    if (byte == NULL || rb->buf == NULL || rb->size == 0U)
        return false;

    if (rb->head == rb->tail)      /* 空 */
        return false;

    *byte = rb->buf[rb->tail];
    rb->tail = ringbuf_advance(rb, rb->tail);
    return true;
}

/* ========== 5. 获取当前已用字节数 ========== */
uint16_t ringbuf_count(ringbuf_t *rb)
{
    if (rb->buf == NULL || rb->size == 0U)
        return 0U;

    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    if (rb->mask != 0U)
        return (uint16_t)((h - t) & rb->mask);
    else
        return (uint16_t)(((int32_t)h - (int32_t)t + (int32_t)rb->size) % (int32_t)rb->size);
}

/* ========== 6. 判断缓冲区是否为空 ========== */
bool ringbuf_is_empty(ringbuf_t *rb)
{
    return (rb->head == rb->tail);
}