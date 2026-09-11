/*
 * drv_adc.c
 *  ADC 驱动层实现。见 drv_adc.h 设计说明。
 */
#include "drv/drv_adc.h"
#include <stddef.h>

/* RA6M5 ADC unit 数上限(ADC0)。留 2 以防部分型号有 unit 1。
 * 带 unit 边界保护:越界只丢弃,绝不内存越界。 */
#define DRV_ADC_MAX_UNITS   2u

static drv_adc_t *s_inst[DRV_ADC_MAX_UNITS];

fsp_err_t drv_adc_open(drv_adc_t *a, adc_instance_ctrl_t *ctrl,
                       adc_cfg_t const *cfg, adc_channel_cfg_t const *chcfg)
{
    if (NULL == a || NULL == ctrl || NULL == cfg || NULL == chcfg)
        return FSP_ERR_ASSERTION;

    a->p_ctrl         = ctrl;
    a->p_cfg          = cfg;
    a->p_channel_cfg  = chcfg;
    a->unit           = cfg->unit;
    a->scan_done      = false;
    /* ready 钩由应用层配;这里故意不清,容错重建(close->open)时钩得以保留,
     * 否则 ISR 完成后将无人唤醒任务 -> 超时重建 -> 死循环。 */

    fsp_err_t err = R_ADC_Open(a->p_ctrl, a->p_cfg);
    if (FSP_SUCCESS == err)
    {
        (void) R_ADC_ScanCfg(a->p_ctrl, a->p_channel_cfg);
        /* 仅 Open 成功才注册;失败时不残留未打开实例到分发表。 */
        if ((uint32_t)a->unit < (uint32_t)DRV_ADC_MAX_UNITS)
            s_inst[(uint32_t)a->unit] = a;
    }
    return err;
}

void drv_adc_close(drv_adc_t *a)
{
    if (NULL == a) return;
    if (NULL != a->p_ctrl) (void) R_ADC_Close(a->p_ctrl);
    if ((uint32_t)a->unit < (uint32_t)DRV_ADC_MAX_UNITS && s_inst[(uint32_t)a->unit] == a)
        s_inst[(uint32_t)a->unit] = NULL;
    a->scan_done = false;
    /* ready 钩同样保留不清。 */
}

fsp_err_t drv_adc_start(drv_adc_t *a)
{
    if (NULL == a) return FSP_ERR_ASSERTION;
    a->scan_done = false;                     /* 软触发前清,确保等的是这次结果 */
    return R_ADC_ScanStart(a->p_ctrl);
}

bool drv_adc_is_done(drv_adc_t *a)
{
    if (NULL == a) return false;
    return a->scan_done;
}

void drv_adc_clear(drv_adc_t *a)
{
    if (NULL == a) return;
    a->scan_done = false;
}

uint16_t drv_adc_read(drv_adc_t *a, adc_channel_t ch)
{
    uint16_t val = 0u;
    if (NULL == a) return 0u;
    (void) R_ADC_Read(a->p_ctrl, ch, &val);
    return val;
}

void drv_adc_set_ready_hook(drv_adc_t *a, void (*fn)(void *), void *ctx)
{
    if (NULL == a) return;
    a->p_ready_hook = fn;
    a->p_ready_ctx  = ctx;
}

void drv_adc_dispatch(adc_callback_args_t *p_args)
{
    if (NULL == p_args) return;
    uint16_t u = p_args->unit;
    drv_adc_t *a = ((uint32_t)u < (uint32_t)DRV_ADC_MAX_UNITS) ? s_inst[(uint32_t)u] : NULL;
    if (NULL == a) return;
    /* 只对扫描完成事件置标志 + 唤醒;其它(如转换错误)不留这里——
     * 应用层靠 scan 超时触发重建,避免把异常状态当正常采样。 */
    if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
    {
        a->scan_done = true;
        if (NULL != a->p_ready_hook) a->p_ready_hook(a->p_ready_ctx);
    }
}