/*
 * drv_adc.h
 *  ADC 驱动层:RTOS-free 的 FSP R_ADC 封装 + ISR 对象化分发。
 *
 *  关于分层(对齐 drv_uart/drv_encoder/drv_pwm 的约定):
 *    - drv 只依赖 FSP,不 include FreeRTOS;
 *    - 扫描完成"ISR->任务"的唤醒由应用层通过 ready 钩注入,驱动只置
 *      scan_done + 调钩,不知道钩里是任务通知还是别的(换 OS/换唤醒方式只换钩)。
 *
 *  关于多实例: FSP 一个 ADC unit 只能挂一个全局回调名 adc_callback;
 *    这里用一个按 unit 索引的小注册表,让全局回调转手分发到对应 drv_adc_t。
 *
 *  Author: Yaojunyi
 */
#ifndef DRV_DRV_ADC_H_
#define DRV_DRV_ADC_H_

#include "r_adc.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC 驱动对象。一个实例 = 一个 ADC unit 上一次单次扫描的所有通道。 */
typedef struct {
    adc_instance_ctrl_t     *p_ctrl;         /* FSP 控制块,如 &g_adc0_ctrl          */
    adc_cfg_t const         *p_cfg;          /* FSP 配置,  如 &g_adc0_cfg           */
    adc_channel_cfg_t const *p_channel_cfg;  /* FSP 通道配置,如 &g_adc0_channel_cfg */
    uint16_t                 unit;           /* ADC unit 号(open 时从 cfg->unit 拷出)*/
    volatile bool            scan_done;      /* ISR 置位;支持无 ready 钩的轮询用法   */
    /* ready 钩:扫描完成 ISR 中调用;ctx 由应用层传入(典型为 TaskHandle_t)。
     * 驱动不解释 ctx,只透传。 */
    void                   (*p_ready_hook)(void *ctx);
    void                    *p_ready_ctx;
} drv_adc_t;

/* 打开并配置 ADC(Open + ScanCfg);按 unit 注册进分发表便于 dispatch。
 * 返回 R_ADC_Open 的 fsp_err_t;成功时会顺带 ScanCfg。
 * 失败时成员已初始化但 ADC 未 open,后续 start/read 会失败,
 * 应用层按返回值决定(重试/死循环)。 */
fsp_err_t drv_adc_open (drv_adc_t *a, adc_instance_ctrl_t *ctrl,
                        adc_cfg_t const *cfg, adc_channel_cfg_t const *chcfg);

/* 关闭 ADC(Close)并从分发表解注册;容错/重建路径用。 */
void      drv_adc_close(drv_adc_t *a);

/* 软触发一次单次扫描(ScanStart);内部清 scan_done,避免读到旧标志。
 * 返回 ScanStart 的 fsp_err_t(IN_USE 等交给应用层,如重建)。 */
fsp_err_t drv_adc_start(drv_adc_t *a);

/* 查询/清 scan_done(无 ready 钩的轮询流派用)。 */
bool      drv_adc_is_done(drv_adc_t *a);
void      drv_adc_clear (drv_adc_t *a);

/* 读指定通道 12-bit 结果(假定本次扫描已完成)。 */
uint16_t  drv_adc_read (drv_adc_t *a, adc_channel_t ch);

/* 设置 ready 钩;钩在 ISR 上下文调用,必须 ISR-safe。 */
void      drv_adc_set_ready_hook(drv_adc_t *a, void (*fn)(void *), void *ctx);

/* 全局回调分发入口:应用层的 adc_callback() 应转手调本函数。
 * 按 p_args->unit 找实例;只对扫描完成事件置 scan_done+调钩,其它事件忽略
 * (转换错误等留给应用层靠超时重建捕获,这里不把异常当正常)。 */
void      drv_adc_dispatch(adc_callback_args_t *p_args);

#ifdef __cplusplus
}
#endif
#endif /* DRV_DRV_ADC_H_ */