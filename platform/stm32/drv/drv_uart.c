/*
 * drv_uart.c
 *
 *  串口驱动层实现（STM32 HAL 版本）
 *
 *  设计要点：
 *    - 多实例管理：HAL 回调无 p_context，通过 huart 反查实例。
 *    - RX DMA 采用循环模式：避免 IDLE 中断中 abort/restart 引发的
 *      HAL 锁竞态，DMA 永不停机，接收位置由 NDTR 计算。
 *    - 发送采用 DMA 非阻塞 + 单帧待发缓冲，提高吞吐并减少阻塞。
 *    - 错误自愈：UART 错误会中止 RX DMA，通过 IDLE 或错误回调重启。
 *
 *  使用前提：
 *    - UART 已由 CubeMX 配置为 DMA 收发模式（RX 使用循环 DMA）。
 *    - 需在 USARTx_IRQHandler 中调用 drv_uart_rx_idle_handler()。
 */

#include "drv_uart.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ========== 1. 多实例注册表（HAL 回调无 p_context，靠 huart 反查） ========== */
#define DRV_UART_MAX_INSTANCES  3
static drv_uart_t *s_instances[DRV_UART_MAX_INSTANCES];
static int          s_instance_count = 0;

static drv_uart_t *find_instance(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < s_instance_count; i++)
        if (s_instances[i]->huart == huart)
            return s_instances[i];
    return NULL;
}

/* ========== 2. 初始化 ========== */
void drv_uart_init(drv_uart_t *u, UART_HandleTypeDef *huart)
{
    memset(u, 0, sizeof(*u));
    u->huart = huart;
    ringbuf_init(&u->rx_ring, u->rx_ring_mem, DRV_UART_RX_RING_SIZE);

    /* 注册到回调表，供 HAL_UART_TxCpltCallback 反查 */
    if (s_instance_count < DRV_UART_MAX_INSTANCES)
        s_instances[s_instance_count++] = u;

    /* RX DMA 切换为循环模式：IDLE 中断里不再 abort/restart。
     * （旧方案在 IRQ 里调 HAL_UART_Receive_DMA，与主循环 printf 的
     * TX DMA 存在 HAL 锁竞态，一旦撞上接收永久失效。）
     * 循环模式下 DMA 永不停机，写入位置 = 缓冲大小 - NDTR。 */
    if (huart->hdmarx != NULL)
    {
        huart->hdmarx->Init.Mode = DMA_CIRCULAR;
        (void)HAL_DMA_Init(huart->hdmarx);
    }

    /* 开启 IDLE 空闲中断 + 启动 DMA 接收 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, u->dma_rx_buf, DRV_UART_RX_DMA_SIZE);
}

/* ========== 3. IDLE 中断处理（在 USARTx_IRQHandler 的 USER CODE 段调用） ==========
 * 循环 DMA 不停机：本函数只根据 NDTR 计算新数据区间并搬运到 ringbuf，
 * 常规路径不调用任何 HAL 接口（无锁竞态，耗时与字节数成正比）。 */
void drv_uart_rx_idle_handler(drv_uart_t *u)
{
    uint16_t write_pos;
    uint16_t i;

    if (u == NULL || u->huart == NULL)
        return;

    if (__HAL_UART_GET_FLAG(u->huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(u->huart);

        /* 写入位置 = 缓冲区大小 - DMA 剩余计数（循环回卷安全） */
        write_pos = (uint16_t)(DRV_UART_RX_DMA_SIZE
                    - __HAL_DMA_GET_COUNTER(u->huart->hdmarx));

        for (i = u->rx_read_pos; i != write_pos;
             i = (uint16_t)((i + 1u) % DRV_UART_RX_DMA_SIZE))
        {
            if (!ringbuf_push(&u->rx_ring, u->dma_rx_buf[i]))
                u->rx_drop_count++;
        }
        u->rx_read_pos  = write_pos;
        u->rx_frame_count++;

        /* RX 自愈兜底：UART 错误（ORE/FE/NE）路径会中止 RX DMA 并清
         * DMAR 位。检测到即重启（HAL_BUSY 则下个 IDLE 再试，10ms 级）。 */
        if ((u->huart->Instance->CR3 & USART_CR3_DMAR) == 0u)
        {
            if (HAL_UART_Receive_DMA(u->huart, u->dma_rx_buf,
                                     DRV_UART_RX_DMA_SIZE) == HAL_OK)
            {
                u->rx_read_pos = 0u;
                u->err_count++;
            }
        }
    }
}

/* ========== 4. 读取接口 ========== */
uint16_t drv_uart_read(drv_uart_t *u, uint8_t *dst, uint16_t max)
{
    if (u == NULL || dst == NULL)
        return 0U;

    uint16_t n = 0U;
    while (n < max && ringbuf_pop(&u->rx_ring, &dst[n]))
        n++;
    return n;
}

bool drv_uart_read_byte(drv_uart_t *u, uint8_t *byte)
{
    if (u == NULL || byte == NULL)
        return false;
    return ringbuf_pop(&u->rx_ring, byte);
}

/* ========== 5. 发送接口（DMA 非阻塞 + 单帧待发缓冲） ========== */
static bool drv_uart_start_dma_tx(drv_uart_t *u, const uint8_t *src, uint16_t len)
{
    if (HAL_UART_Transmit_DMA(u->huart, (uint8_t *)src, len) != HAL_OK)
    {
        u->tx_busy = false;
        u->tx_err_count++;
        return false;
    }
    return true;
}

int drv_uart_write(drv_uart_t *u, const uint8_t *src, uint16_t len)
{
    if (u == NULL || src == NULL || len == 0U)
        return 0;

    if (len > DRV_UART_TX_BUF_SIZE)
    {
        len = DRV_UART_TX_BUF_SIZE;
    }

    __disable_irq();
    if (!u->tx_busy)
    {
        memcpy(u->tx_dma_buf, src, len);
        u->tx_busy = true;
        if (!drv_uart_start_dma_tx(u, u->tx_dma_buf, len))
        {
            __enable_irq();
            return 0;
        }
    }
    else if (!u->tx_pending_full)
    {
        memcpy(u->tx_pending_buf, src, len);
        u->tx_pending_len = len;
        u->tx_pending_full = true;
    }
    else
    {
        __enable_irq();
        u->tx_drop_count++;
        return 0;
    }

    __enable_irq();
    return (int)len;
}

/* ========== 6. printf 格式化发送 ========== */
int drv_uart_printf(drv_uart_t *u, const char *fmt, ...)
{
    if (u == NULL)
        return 0;

    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)u->tx_buf, DRV_UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (len <= 0)
        return 0;
    if (len > DRV_UART_TX_BUF_SIZE)
        len = DRV_UART_TX_BUF_SIZE;

    return drv_uart_write(u, u->tx_buf, (uint16_t)len);
}

/* ========== 7. HAL 发送完成回调（覆盖 HAL 弱定义） ========== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    drv_uart_t *u = find_instance(huart);
    if (u)
    {
        if (u->tx_pending_full)
        {
            u->tx_pending_full = false;
            (void)drv_uart_start_dma_tx(u, u->tx_pending_buf, u->tx_pending_len);
        }
        else
        {
            u->tx_busy = false;
        }
    }
}

/* ========== 8. HAL 错误回调（覆盖 HAL 弱定义） ==========
 * 阻塞型错误（ORE/FE/NE）会中止 RX DMA；此处立即重启接收。
 * 若接收并未被中止（如纯 TX 错误），HAL 返回 BUSY，无副作用；
 * 若重启失败（锁竞态），IDLE 处理里的 DMAR 自愈兜底会继续重试。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    drv_uart_t *u = find_instance(huart);

    if (u == NULL)
        return;
    u->err_count++;
    if (HAL_UART_Receive_DMA(huart, u->dma_rx_buf,
                             DRV_UART_RX_DMA_SIZE) == HAL_OK)
    {
        u->rx_read_pos = 0u;
    }
}