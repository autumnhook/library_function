/*
 * drv_uart.h
 *
 *  串口驱动层：将 STM32 HAL 的 UART + DMA + IDLE 中断封装为一个对象。
 *
 *  特性：
 *    - RX：循环模式 DMA 接收 + IDLE 空闲中断搬运数据到 ringbuf。
 *      中断里只读 NDTR 计算新数据区间，不调用 HAL（无锁竞态、无重启
 *      失败导致接收永久失效的风险）；UART 错误回调 + DMAR 自愈兜底。
 *    - TX：非阻塞 DMA 发送，附带单帧待发缓冲。
 *    - 队列满时丢弃新调试帧并计数，不阻塞控制主循环。
 *    - 多实例支持：drv_uart_init 自动注册到静态表，HAL 回调按 huart 反查实例。
 *
 *  使用前提：
 *    1. UART 已在 CubeMX 中配置为 DMA 收发模式（RX 使用循环 DMA），
 *       并通过 __HAL_LINKDMA 绑定到 huart->hdmarx / hdmatx。
 *    2. 需要在 USARTx_IRQHandler 的 USER CODE 段调用 drv_uart_rx_idle_handler()。
 *    3. RX ring buffer 容量（DRV_UART_RX_RING_SIZE）必须为 2 的幂，
 *       以使用 &mask 快速回卷。
 *
 *  示例：
 *    // 全局定义
 *    drv_uart_t g_uart;
 *
 *    // 初始化
 *    drv_uart_init(&g_uart, &huart1);
 *
 *    // 在 USART1_IRQHandler 中调用
 *    drv_uart_rx_idle_handler(&g_uart);
 *
 *    // 主循环中读取数据
 *    uint8_t buf[64];
 *    uint16_t n = drv_uart_read(&g_uart, buf, sizeof(buf));
 *
 *    // 发送字符串
 *    drv_uart_printf(&g_uart, "Hello\r\n");
 */

#ifndef DRV_DRV_UART_H_
#define DRV_DRV_UART_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "lib/lib_ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DMA 接收缓冲区大小（与原 rx_buffer[100] 一致）*/
#define DRV_UART_RX_DMA_SIZE   100
/* ringbuf 容量（2 的幂 -> 走 &mask 快速回卷）*/
#define DRV_UART_RX_RING_SIZE  256
/* printf 格式化缓冲区大小（与原 SendBuff[200] 一致）*/
#define DRV_UART_TX_BUF_SIZE   200

typedef struct {
    /* ---- 硬件句柄（CubeMX 生成，DMA 已绑定在其中）---- */
    UART_HandleTypeDef *huart;

    /* ---- RX ---- */
    uint8_t           dma_rx_buf[DRV_UART_RX_DMA_SIZE]; /* DMA 循环写入的接收缓冲区 */
    uint16_t          rx_read_pos;      /* DMA 缓冲读位置（仅 IDLE 中断维护）*/
    ringbuf_t         rx_ring;                          /* 供主循环读取的环形缓冲区 */
    uint8_t           rx_ring_mem[DRV_UART_RX_RING_SIZE];
    volatile uint32_t rx_frame_count;   /* IDLE 中断次数（帧计数）*/
    volatile uint32_t rx_drop_count;    /* ring 满丢字节计数 */
    volatile uint32_t err_count;        /* UART 错误/自愈重启计数 */

    /* ---- TX ---- */
    volatile bool     tx_busy;          /* DMA 正在发送 tx_dma_buf */
    volatile bool     tx_pending_full;  /* tx_pending_buf 中有排队数据 */
    volatile uint16_t tx_pending_len;   /* 排队帧的长度 */
    volatile uint32_t tx_drop_count;    /* 排队帧丢弃计数 */
    volatile uint32_t tx_err_count;     /* 发送错误计数 */
    uint8_t           tx_dma_buf[DRV_UART_TX_BUF_SIZE];     /* 正在发送的 DMA 缓冲区 */
    uint8_t           tx_pending_buf[DRV_UART_TX_BUF_SIZE]; /* 单帧待发缓冲区 */
    uint8_t           tx_buf[DRV_UART_TX_BUF_SIZE];         /* printf 格式化缓冲区 */
} drv_uart_t;

/**
 * @brief 初始化串口驱动对象。
 * @param u 驱动对象指针（调用方持有全局实例）
 * @param huart CubeMX 生成的 UART 句柄（如 &huart1）
 * @note  初始化内部会：清零对象 -> 初始化 ringbuf -> 注册到回调表 ->
 *        开启 IDLE 中断 -> 启动 DMA 接收。
 */
void drv_uart_init(drv_uart_t *u, UART_HandleTypeDef *huart);

/**
 * @brief IDLE 中断处理（在 stm32f1xx_it.c 的 USARTx_IRQHandler USER CODE 段调用）。
 * @param u 驱动对象指针
 * @note  检测 IDLE 标志 -> 按 NDTR 计算新数据区间 -> 压入 ringbuf。
 *        常规路径不调用 HAL；检测到 RX DMA 被错误路径中止时自动重启（自愈）。
 */
void drv_uart_rx_idle_handler(drv_uart_t *u);

/**
 * @brief 批量读取已接收的字节。
 * @param u   驱动对象指针
 * @param dst 目标缓冲区
 * @param max 最大读取字节数
 * @return 实际读取的字节数（从 ringbuf pop，非阻塞）。
 */
uint16_t drv_uart_read(drv_uart_t *u, uint8_t *dst, uint16_t max);

/**
 * @brief 读取 1 字节。
 * @param u    驱动对象指针
 * @param byte 输出字节指针
 * @return true 表示读取成功，false 表示缓冲区为空。
 */
bool drv_uart_read_byte(drv_uart_t *u, uint8_t *byte);

/**
 * @brief 非阻塞 DMA 发送。源数据会被复制到内部缓冲区。
 * @param u   驱动对象指针
 * @param src 待发送数据指针
 * @param len 数据长度（字节）
 * @return 成功排队返回实际排队长度（可能被截断到内部缓冲区大小），
 *         启动失败或队列已满返回 0。
 * @note  如果发送忙且有 pending 缓冲，会尝试存入 pending；如果 pending 已满则丢弃。
 */
int drv_uart_write(drv_uart_t *u, const uint8_t *src, uint16_t len);

/**
 * @brief printf 风格格式化发送（内部缓冲区，多实例安全）。
 * @param u   驱动对象指针
 * @param fmt 格式化字符串
 * @param ... 可变参数
 * @return 成功返回实际发送的字节数（可能被截断），失败返回 0。
 */
int drv_uart_printf(drv_uart_t *u, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DRV_DRV_UART_H_ */