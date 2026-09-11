/*
 * dev_wheel.c
 *
 *  设备层实现：把左/右轮的测速、滤波、缓加速、速度环 PID、方向控制、
 *  PWM 输出全部收进一个对象。
 *
 *  特性：
 *    - 每个轮子一个 dev_wheel_t 对象，全局实例 g_tLeftWheel / g_tRightWheel。
 *    - 方向控制：每轮双脚(IN1/IN2)互补换向；双脚同高=电机短路抱死。
 *    - 占空比输出：PID 输出(±100)直接取绝对值作占空比百分比，符号决定方向。
 *    - 输出斜率限制：PID 输出每拍变化 ≤ WHEEL_OUT_SLEW(%)，平滑占空比。
 *    - 缓加速：SoftSpeed 按 WHEEL_SOFT_STEP 逐步逼近 ExpectSpeed，末步补差不过冲。
 *    - ISR 只做硬件读数+累加，滤波/PID 全部放在主循环上下文的 update 里。
 *
 *  使用前提：
 *    1. 调用方需先 init 好 encoder / pwm 对象，再传给 dev_wheel_init。
 *    2. dev_wheel_on_sample_isr 需由中频(10ms)采样定时器调用。
 *    3. dev_wheel_update 需由主循环周期调用(每拍约20ms)。
 *
 *  示例：
 *    // 初始化
 *    dev_wheel_init(&g_tLeftWheel, &pwm_left, &enc_left,
 *                   GPIOA, GPIO_PIN_0, GPIO_PIN_1, 1);
 *
 *    // ISR 中
 *    dev_wheel_on_sample_isr(&g_tLeftWheel);
 *
 *    // 主循环中
 *    dev_wheel_update(&g_tLeftWheel);
 *    dev_wheel_set_speed(&g_tLeftWheel, 300);   // 300 mm/s
 *    int16_t v = dev_wheel_get_speed(&g_tLeftWheel);
 */

#include "dev_wheel.h"
#include "drv_gpio.h"
#include <string.h>   /* memset */
#include <stdlib.h>   /* abs    */

/* 全局实例(取代原 TimeCapture.c 的 volatile WheelSruct_t) */
dev_wheel_t g_tLeftWheel;
dev_wheel_t g_tRightWheel;

/* ============ 1. 工具:电平/引脚 ============ */

/**
 * @brief 取反电平。
 * @param lvl 原电平（0/1）
 * @return 1-lvl
 */
static uint8_t opp_level(uint8_t lvl)
{
    return lvl ? 0u : 1u;
}

/**
 * @brief 写方向脚（IN1/IN2 互补）。
 * @param self 轮对象指针
 * @param lvl  前进方向电平（=fwd_level 表示前进，=!fwd_level 表示后退）
 * @note  dir_pin/dir_pin2 互补：lvl=前进电平时 (IN2=1, IN1=0)。
 * @note  内部调用 drv_gpio_write_pair()，方向语义留在本层。
 */
static void write_dir_pins(dev_wheel_t *self, uint8_t lvl)
{
    /* dir_pin/dir_pin2 互补: lvl=前进电平时 (IN2=1, IN1=0) */
    drv_gpio_write_pair(self->dir_port, self->dir_pin,
                        self->dir_pin2, lvl != 0u);
}

/* ============ 2. 初始化 ============ */

/**
 * @brief 初始化轮对象。
 * @param self      轮对象指针
 * @param pwm       PWM 驱动对象指针（调用方已 init）
 * @param encoder   编码器驱动对象指针（调用方已 init）
 * @param dir_port  方向脚端口
 * @param dir_pin   方向脚 IN1
 * @param dir_pin2  方向脚 IN2
 * @param fwd_level 前进电平（IN1=fwd_level 时前进）
 * @note  内部会 memset 清零；WheelRetreat==0，Direction 必须显式置 Forward，
 *        否则 process_sample 里“符号随 Direction”会让实测速度符号反。
 */
void dev_wheel_init(dev_wheel_t *self,
                    drv_pwm_t *pwm,
                    drv_encoder_t *encoder,
                    GPIO_TypeDef *dir_port, uint16_t dir_pin,
                    uint16_t dir_pin2, uint8_t fwd_level)
{
    memset(self, 0, sizeof(*self));               /* 全清 */
    self->pwm      = pwm;
    self->encoder  = encoder;     /* 存指针:调用方持全局 encoder 对象,先 init 好再传 */
    self->dir_port = dir_port;
    self->dir_pin  = dir_pin;
    self->dir_pin2 = dir_pin2;
    self->fwd_level = fwd_level;
    self->locked   = false;       /* memset 已清 0,显式写明语义 */
    self->isr_last_count = drv_encoder_get_count(self->encoder);

    /* 滤波器:memset 后 ewma.shift=0 会退化,须显式设 */
    filter_median_init(&self->median);
    filter_ewma_init(&self->ewma, WHEEL_EWMA_SHIFT);

    /* 速度环 PID:位置式+条件积分 */
    pid_init(&self->speed_pid, PID_MODE_POSITION);
    pid_set_gains (&self->speed_pid, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD);
    pid_set_limits(&self->speed_pid,
                   -WHEEL_PID_LIMIT,  WHEEL_PID_LIMIT,
                   -WHEEL_PID_LIMIT,  WHEEL_PID_LIMIT);

    /* 关键:WheelRetreat==0,memset 后必须显式置 Forward,
       否则 process_sample 里"符号随 Direction"会让实测速度符号反。 */
    self->Direction    = WheelForward;
    self->ExpectSpeed  = 0;
    self->SoftSpeed    = 0;
    self->soft_div_cnt = 0;
    self->sample_ready = false;
}

/* ============ 3. ISR 与测速滤波 ============ */

/**
 * @brief 中频(10ms)采样定时器 ISR 体。
 * @param self 轮对象指针
 * @note  只做硬件读数+16位回卷安全减法+累加，不做滤波/PID（<5us）。
 * @note  delta/周期数由主循环消费时清零：主循环偶尔慢一拍（如 printf 阻塞），
 *        下一次 update 会拿到 2 个周期的累加值，按周期数折算回速度，不丢脉冲。
 */
void dev_wheel_on_sample_isr(dev_wheel_t *self)
{
    uint32_t now = drv_encoder_get_count(self->encoder);
    self->pending_delta += (uint16_t)(now - self->isr_last_count);  /* 16位回卷自动处理 */
    self->isr_last_count = now;
    self->pending_periods++;
    self->sample_ready = true;
}

/**
 * @brief 消费 ISR 累加的 delta/周期数，更新 InstantSpeed / RealSpeed。
 * @param self 轮对象指针
 * @note  关中断快照+清零，防止复制途中 ISR 再写入而丢脉冲。
 * @note  漏拍时 delta 覆盖 n 个周期，按 n 折算回速度。
 * @note  抱死锁定期间不累计里程，测速/滤波照常。
 */
static void process_sample(dev_wheel_t *self)
{
    uint32_t delta;
    uint8_t  periods;

    __disable_irq();                                /* 临界区:读+清必须原子 */
    delta   = self->pending_delta;
    periods = self->pending_periods;
    self->pending_delta   = 0;
    self->pending_periods = 0;
    __enable_irq();

    if (periods == 0)                                /* 防御:无新数据 */
        periods = 1;

    /* 瞬时速度(mm/s) = delta*周长*1000/(PPR*采样周期*periods) */
    if (delta > 0)
        self->InstantSpeed = (int16_t)((delta * WHEEL_SPEED_COEF)
                                       / (WHEEL_ENCODER_PPR * periods));
    else
        self->InstantSpeed = 0;

    if (delta > 0)
    {
        self->timeout = 0;
        /* 抱死锁定期间编码器会有噪声脉冲（实测停车后凭空多出 ~29.9 万脉冲），
         * 虚假里程会推高 trip，让 70% 圈终点门槛提前满足 -> 提前停车。
         * locked 期间不累计，测速/滤波照常。 */
        if (!self->locked)
            self->total_pulse += (int32_t)delta;    /* 里程累计(不分方向,与原一致) */

        int16_t signed_speed = self->InstantSpeed;
        if (self->Direction == WheelRetreat)          /* 符号随"命令方向",保留原行为 */
            signed_speed = -signed_speed;

        int16_t med = filter_median_push(&self->median, signed_speed);
        self->RealSpeed = filter_ewma_update(&self->ewma, med);
    }
    else
    {
        self->timeout++;
        if (self->timeout >= WHEEL_STOP_TIMEOUT)
        {
            /* 超时判定为完全停止 */
            self->RealSpeed    = 0;
            self->InstantSpeed = 0;
            self->timeout      = WHEEL_STOP_TIMEOUT;  /* 防溢出 */
            filter_ewma_reset(&self->ewma, self->RealSpeed);
        }
        else
        {
            self->RealSpeed >>= 1;                     /* 快速衰减,每次减半 */
            if (abs(self->RealSpeed) < 1)              /* 绝对值<1 直接置0 */
                self->RealSpeed = 0;
            filter_ewma_reset(&self->ewma, self->RealSpeed);
        }
    }
}

/* ============ 4. 缓加速与运动更新 ============ */

/**
 * @brief 缓加速：SoftSpeed 按 WHEEL_SOFT_STEP 逼近 ExpectSpeed。
 * @param self 轮对象指针
 * @note  末步补差精确到位：步进越界时直接取期望差值，不会过冲。
 */
static void soft_adjust(dev_wheel_t *self)
{
    int16_t step = 0;
    if (self->SoftSpeed < self->ExpectSpeed)
    {
        step = WHEEL_SOFT_STEP;
        if (self->SoftSpeed + step > self->ExpectSpeed)
            step = self->ExpectSpeed - self->SoftSpeed;
    }
    else if (self->SoftSpeed > self->ExpectSpeed)
    {
        step = -WHEEL_SOFT_STEP;
        if (self->SoftSpeed + step < self->ExpectSpeed)
            step = self->ExpectSpeed - self->SoftSpeed;
    }
    self->SoftSpeed += step;
}

/**
 * @brief PID -> 占空比 -> 方向：每拍跑一次速度环并刷新 PWM/方向脚。
 * @param self 轮对象指针
 * @note  死区内软关断 PID 并 reset，重新起步积分从 0 开始。
 * @note  SoftSpeed!=0 时禁止反向制动，且保最小蠕动 10(%)，避免原地抖动。
 * @note  输出斜率限制每拍最多变化 ±WHEEL_OUT_SLEW(%)，滤掉反馈量化噪声。
 */
static void motion_update(dev_wheel_t *self)
{
    int16_t pid;

    if (abs(self->SoftSpeed) < WHEEL_DEADBAND)
    {
        pid = 0;
        /* 停转区:统一软关断(即使误调 pid_calc 也恒输出0)+清历史,
         * 重新起步积分从0开始(与旧版每拍 reset 行为一致) */
        pid_enable(&self->speed_pid, false);
        pid_reset(&self->speed_pid);
        self->out_slew = 0;                        /* 同步清零:重新起步从0缓升 */
    }
    else
    {
        pid_enable(&self->speed_pid, true);        /* 离开死区:恢复闭环 */
        pid = (int16_t)pid_calc(&self->speed_pid,
                               (float)self->SoftSpeed,
                               (float)self->RealSpeed);
        /* forbid reverse braking while SoftSpeed!=0, min creep 10 */
        if (self->SoftSpeed > 0)      { if (pid <  10) pid =  10; }
        else if (self->SoftSpeed < 0) { if (pid > -10) pid = -10; }

        /* 输出斜率限制:每拍最多变化 ±WHEEL_OUT_SLEW(%),
         * 滤掉反馈量化噪声(1脉冲≈4.6mm/s)引起的逐拍占空比抖动 */
        if (pid > self->out_slew + WHEEL_OUT_SLEW)
            pid = self->out_slew + WHEEL_OUT_SLEW;
        else if (pid < self->out_slew - WHEEL_OUT_SLEW)
            pid = self->out_slew - WHEEL_OUT_SLEW;
        self->out_slew = pid;
    }

    if (pid < 0)                                     /* 后退(双脚真换向) */
    {
        write_dir_pins(self, opp_level(self->fwd_level));
        self->Direction = WheelRetreat;
        drv_pwm_set_duty_percent(self->pwm, (uint8_t)(-pid));
    }
    else if (pid > 0)                                /* 前进 */
    {
        write_dir_pins(self, self->fwd_level);
        self->Direction = WheelForward;
        drv_pwm_set_duty_percent(self->pwm, (uint8_t)pid);
    }
    else                                             /* 停转:占空比0+前进图案 */
    {
        drv_pwm_set_duty_percent(self->pwm, 0);
        write_dir_pins(self, self->fwd_level);
        self->Direction = WheelStop;
    }
}

/* ============ 5. 对外周期更新与控制 ============ */

/**
 * @brief 周期更新（主循环上下文，每拍约 20ms）。
 * @param self 轮对象指针
 * @note  抱死锁定期间测速照跑，但里程不累计；同时跳过斜坡/PID，防止覆盖刹车图案。
 * @note  缓加速每 WHEEL_SOFT_DIV 次 update 一步（默认 2 次 = 20ms）。
 * @note  PID+PWM 每拍都跑。
 */
void dev_wheel_update(dev_wheel_t *self)
{
    if (self->sample_ready)
    {
        self->sample_ready = false;
        process_sample(self);      /* 抱死期间测速照跑,但里程不累计(见 process_sample) */
    }

    if (self->locked)              /* 抱死保持:跳过斜坡/PID,防止覆盖刹车图案 */
        return;

    if (++self->soft_div_cnt >= WHEEL_SOFT_DIV)     /* 每 2 次 update 一步 = 20ms */
    {
        self->soft_div_cnt = 0;
        soft_adjust(self);
    }

    motion_update(self);                             /* 每拍都跑 PID+PWM(拍距约20ms) */
}

/**
 * @brief 设置期望速度，并解除抱死锁定。
 * @param self       轮对象指针
 * @param speed_mm_s 期望速度（mm/s，带符号，负为后退）
 * @note  不立即改 PWM，靠缓加速渐变到目标。
 * @note  同时清除 locked 并解除 PID 软关断，恢复闭环。
 */
void dev_wheel_set_speed(dev_wheel_t *self, int16_t speed_mm_s)
{
    self->ExpectSpeed = speed_mm_s;                  /* 不立即改 PWM,靠缓加速渐变 */
    self->locked      = false;                       /* 速度命令=解除抱死,恢复闭环 */
    pid_enable(&self->speed_pid, true);              /* 同步解除软关断 */
}

/**
 * @brief 硬抱死并保持。
 * @param self 轮对象指针
 * @note  两方向脚同置高（IN1=IN2=1，电机短路刹车），占空比拉满，
 *        并置 locked 让 update 跳过斜坡/PID——否则下一拍 motion_update
 *        会用前进电平覆盖刹车图案。dev_wheel_set_speed 解除锁定。
 * @note  占空比给 100 而非 0：TB6612 两态皆抱死；L298 类 EN=高才是刹车。
 * @note  同时软关断 PID 并 reset，锁定期间任何误调用恒输出 0。
 */
void dev_wheel_brake(dev_wheel_t *self)
{
    drv_pwm_set_duty_percent(self->pwm, 100);
    HAL_GPIO_WritePin(self->dir_port,
                      self->dir_pin | self->dir_pin2, GPIO_PIN_SET);
    self->Direction   = WheelStop;
    self->ExpectSpeed = 0;
    self->SoftSpeed   = 0;
    self->locked      = true;
    pid_enable(&self->speed_pid, false);   /* 软关断:锁定期间任何误调用恒输出0 */
    pid_reset(&self->speed_pid);
}

/**
 * @brief 读取当前实测速度。
 * @param self 轮对象指针
 * @return 实测速度（mm/s，带符号）
 */
int16_t dev_wheel_get_speed(dev_wheel_t *self)
{
    return self->RealSpeed;
}

/**
 * @brief 读取累计里程脉冲数。
 * @param self 轮对象指针
 * @return 累计脉冲数（不分方向，抱死锁定期间不累计）
 */
int32_t dev_wheel_get_distance(dev_wheel_t *self)
{
    return self->total_pulse;
}