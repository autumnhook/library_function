/*
 * lib_pid.c
 *
 * PID 库实现。
 *
 * 设计要点：
 *   - 位置式：P + I + D，积分分离抗饱和，输出限幅；行为与旧版一致。
 *   - 增量式：Δu 累加到 integ，每拍输出限幅即抗饱和，本项目暂未使用。
 *   - 软开关：pid_enable() 关断后，所有计算接口恒返 0。
 *   - 微分：位置式内部差分，支持 d_on_meas；首拍 D=0，防微分冲击。
 *   - 角度环：pid_calc_rate() 由调用方提供误差与外部测量速率，D=-Kd*meas_rate。
 *   - 扩展接口：pid_calc_ext() 由调用方直接提供 D 项。
 *   - 免依赖 <math.h>，用足够大数值 PID_NO_LIMIT 表示“无限制”。
 *
 * 使用前提：
 *   - 调用方需先 pid_init() 设置 mode，再设置增益、限幅等参数。
 *   - 无 dt 参数，Ki/Kd 按固定采样周期整定，量纲隐含采样周期。
 *   - 位置式与增量式共用 integ，但含义不同：
 *       位置式中 integ 是积分项；
 *       增量式中 integ 是输出累积器。
 *     切换模式或混用接口前建议 pid_reset()。
 *   - 默认不限幅，若不调用 pid_set_limits()，输出与积分实际上不受限。
 */

#include "lib_pid.h"

/* ========== 1. 内部工具 ========== */

/* 免依赖 <math.h>，用足够大的数值表示“无限制”（默认不启积分钳制） */
#define PID_NO_LIMIT  (1.0e30f)

static float pid_clampf(float v, float lo, float hi)
{
    // 上下限钳制
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ========== 2. 初始化与配置 ========== */

void pid_init(pid_t *pid, pid_mode_t mode)
{
    // 增益默认 0，调用方按需设置
    pid->kp = 0.0f; pid->ki = 0.0f; pid->kd = 0.0f;

    // 输出限幅、积分限幅默认不限
    pid->out_min   = -PID_NO_LIMIT; pid->out_max   =  PID_NO_LIMIT;
    pid->integ_min = -PID_NO_LIMIT; pid->integ_max =  PID_NO_LIMIT;

    // mode 决定位置式 / 增量式
    pid->mode      = mode;

    // 默认 D 作用在误差上；true 时 D 作用在测量值上
    pid->d_on_meas = false;

    // 软开关默认使能
    pid->enabled   = true;

    // 运行历史清零，primed=false 表示尚未跑过第一拍
    pid->integ     = 0.0f;
    pid->prev_err  = 0.0f; pid->prev2_err = 0.0f; pid->prev_meas = 0.0f;
    pid->primed    = false;
}

void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    // 设置三个增益
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
}

void pid_set_limits(pid_t *pid, float out_min, float out_max,
                                   float integ_min, float integ_max)
{
    // 设置输出限幅和积分限幅
    pid->out_min   = out_min;   pid->out_max   = out_max;
    pid->integ_min = integ_min; pid->integ_max = integ_max;
}

void pid_set_d_on_meas(pid_t *pid, bool on)
{
    // true：D 作用在测量值上；false：D 作用在误差上
    pid->d_on_meas = on;
}

void pid_enable(pid_t *pid, bool en)
{
    // 软开关：关闭后 pid_calc() / pid_calc_rate() / pid_calc_ext() 恒返 0
    pid->enabled = en;
}

void pid_reset(pid_t *pid)
{
    // 只清运行历史，不清 enabled / 参数 / 限幅
    pid->integ    = 0.0f;
    pid->prev_err = 0.0f; pid->prev2_err = 0.0f; pid->prev_meas = 0.0f;
    pid->primed   = false;
}

/* ========== 3. 位置式核心 ========== */

/* ---- 位置式核心（调用方可外带 error + D 项） ----
 * 积分分离抗饱和：P+I 预览判饱和，与速度环原实现一致。
 */
static float pid_step_pos(pid_t *pid, float error, float d_term)
{
    // 比例项
    float p = pid->kp * error;

    // 预览 P + I：若继续积分只会加深饱和，则本拍不允许积分
    float preview = p + pid->integ;
    bool  sat_hi  = (preview > pid->out_max);
    bool  sat_lo  = (preview < pid->out_min);
    bool  allow   = (!sat_hi && !sat_lo)
                  || (sat_hi && error < 0.0f)
                  || (sat_lo && error > 0.0f);

    // 允许积分时累加，随后做积分限幅
    if (allow)
        pid->integ += pid->ki * error;
    pid->integ = pid_clampf(pid->integ, pid->integ_min, pid->integ_max);

    // 输出 = P + I + D，再做输出限幅
    float out = p + pid->integ + d_term;
    return pid_clampf(out, pid->out_min, pid->out_max);
}

/* ========== 4. 增量式核心 ========== */

/* ---- 增量式核心：integ 充当输出累积器 out[n-1]，每拍 clamp 即天然抗饱和 ---- */
static float pid_step_inc(pid_t *pid, float error)
{
    // 增量 = Kp*Δe + Ki*e + Kd*(e - 2e_prev + e_prev2)
    float delta = pid->kp * (error - pid->prev_err)
                + pid->ki * error
                + pid->kd * (error - 2.0f * pid->prev_err + pid->prev2_err);

    // 累积输出并限幅，clamp 即抗饱和
    float out = pid->integ + delta;
    out = pid_clampf(out, pid->out_min, pid->out_max);

    // 保存输出累积器和误差历史
    pid->integ     = out;
    pid->prev2_err = pid->prev_err;
    pid->prev_err  = error;
    pid->primed    = true;
    return out;
}

/* ========== 5. 对外计算接口 ========== */

float pid_calc(pid_t *pid, float setpoint, float measurement)
{
    // 软开关关闭：恒返 0
    if (!pid->enabled)
        return 0.0f;

    float error = setpoint - measurement;

    // 增量式：直接走增量核心
    if (pid->mode == PID_MODE_INCREMENTAL)
        return pid_step_inc(pid, error);

    // 位置式：D 为内部差分；首拍 primed=false 时 D=0，防微分冲击
    float d = 0.0f;
    if (pid->kd != 0.0f && pid->primed)
    {
        if (pid->d_on_meas)
            d = -pid->kd * (measurement - pid->prev_meas);
        else
            d =  pid->kd * (error - pid->prev_err);
    }

    float out = pid_step_pos(pid, error, d);

    // 刷新 prev_err / prev_meas；切换 d_on_meas 后旧历史自动作废
    pid->prev_err  = error;
    pid->prev_meas = measurement;
    pid->primed    = true;
    return out;
}

/* ---- 角度环：调用方提供误差与外部测量速率，D = -Kd*meas_rate ----
 * 始终按位置式处理，增量式不适合外部速率输入。
 * 注意：本接口不更新 prev_meas，只更新 prev_err / primed。
 */
float pid_calc_rate(pid_t *pid, float error, float meas_rate)
{
    // 软开关关闭：恒返 0
    if (!pid->enabled)
        return 0.0f;

    // D 项直接使用外部测量速率
    float d = (pid->kd != 0.0f) ? -pid->kd * meas_rate : 0.0f;

    float out = pid_step_pos(pid, error, d);

    // 刷新误差历史
    pid->prev_err  = error;
    pid->primed    = true;
    return out;
}

/* ---- 扩展接口：调用方直接提供 D 项 ----
 * 例如球速来自相机，而不是对位置做差分。
 */
float pid_calc_ext(pid_t *pid, float setpoint, float measurement,
                   float d_term)
{
    // 软开关关闭：恒返 0
    if (!pid->enabled)
        return 0.0f;

    float error = setpoint - measurement;

    float out = pid_step_pos(pid, error, d_term);

    // 刷新误差与测量历史
    pid->prev_err  = error;
    pid->prev_meas = measurement;
    pid->primed    = true;
    return out;
}