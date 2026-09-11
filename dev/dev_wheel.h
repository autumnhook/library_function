/*
 * dev_wheel.h
 *
 *  设备层：把一个驱动轮的"测速+滤波+缓加速+速度环PID+方向+PWM输出"
 *  全部收进一个 dev_wheel_t 对象，左/右轮共用同一套代码。
 *
 *  特性：
 *    - 左/右轮各一个 dev_wheel_t 对象，全局实例 g_tLeftWheel / g_tRightWheel。
 *    - 方向控制：每轮两脚（IN1/IN2）互补换向；双脚同高=电机短路抱死。
 *    - PWM 输出：PID 输出（0~100）直接映射占空比百分比，越大越快；符号决定方向。
 *    - 速度环 PID 内嵌对象，取代旧的全局 g_pid_left / g_pid_right。
 *    - 测速滤波：3 点中值（去尖峰）+ EWMA（平滑）。
 *    - 缓加速：SoftSpeed 按 WHEEL_SOFT_STEP 逐步逼近 ExpectSpeed。
 *    - ISR 只读编码器计数并累加；滤波/PID/输出全在主循环 update 里。
 *
 *  分层原则：
 *    - drv/lib 已分层；dev_wheel 只依赖 drv + lib，不直接碰 HAL。
 *    - 方向引脚通过 drv_gpio_write_pair() 输出（中性双引脚互补写），
 *      电机“前进/后退”语义留在本层。
 *
 *  使用前提：
 *    1. 调用方需先 init 好 pwm / encoder 对象，再传给 dev_wheel_init。
 *    2. dev_wheel_on_sample_isr 由中频(10ms)采样定时器调用。
 *    3. dev_wheel_update 由主循环周期调用（每拍约20ms）。
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
 *
 *  本工程第一版目标：循迹车。循迹模块最后写，先保证双轮速度一致走直线。
 */

#ifndef DEV_DEV_WHEEL_H_
#define DEV_DEV_WHEEL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "drv/drv_pwm.h"
#include "drv/drv_encoder.h"
#include "lib/lib_pid.h"
#include "lib/lib_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 运动方向 ========== */

/* ===== 轮子运动方向 =====
 *  注意：WheelRetreat=0 是 memset 清零后的默认值，故 dev_wheel_init 必须
 *        显式把 Direction 置成 WheelForward，否则实测速度符号会反过来。*/
typedef enum {
    WheelRetreat = 0,
    WheelForward,
    WheelStop
} WheelDirection_e;

/* ========== 2. 编译期物理/控制参数（按实车标定，待调） ========== */

#define WHEEL_PERIMETER_MM   255     /* 轮子周长 mm（待实车测量）  */
#define WHEEL_ENCODER_PPR    56000   /* 轮子转一圈计数：500线*4倍频*减速比28 */
#define WHEEL_SAMPLE_MS      10      /* 测速采样周期 ms（==TIM1 中断周期）*/
#define WHEEL_STOP_TIMEOUT   6       /* 停转超时(6*20ms=120ms 无脉冲判停) */
#define WHEEL_EWMA_SHIFT     1       /* EWMA alpha=1/2^shift=0.5   */
#define WHEEL_DEADBAND       10      /* |SoftSpeed|<此值视为停转,PID 关闭 */


#define WHEEL_SOFT_STEP      30       /* 缓加速每步行进 mm/s(斜率=STEP/(DIV*20ms)
                                          =750mm/s^2, 0->200mm/s 约0.4s) */
#define WHEEL_SOFT_DIV       2       /* 每 N 次 update 走一步(N*20ms) */
#define WHEEL_PID_KP         0.20f   /* 响应/振荡折中:0.3左轮±100mm/s极限环(一顿一顿),
                                          0.2循迹修正变钝(偏离不回),0.25为平衡点;
                                          仍钝->调app_follow.h的APP_DIFF_GAIN补偿 */
#define WHEEL_PID_KI         0.03f
#define WHEEL_PID_KD         0.0f
#define WHEEL_PID_LIMIT      100.0f  /* 输出/积分限幅（占空比百分比）*/
#define WHEEL_OUT_SLEW       10      /* 输出斜率限制:每拍(20ms)占空比最大变化%
                                          滤掉反馈量化噪声导致的输出抖动 */
/* 速度系数 = 周长*1000/采样周期ms (=25500): mm/s 每脉冲 */
#define WHEEL_SPEED_COEF     (WHEEL_PERIMETER_MM * 1000 / WHEEL_SAMPLE_MS)

/* ========== 3. 轮子对象 ========== */

typedef struct {
    /* ---- 硬件句柄 ---- */
    drv_pwm_t      *pwm;            /* PWM 输出(两个轮共享一个 TIM,这里持指针) */
    drv_encoder_t  *encoder;        /* 编码器测速指针(调用方持全局对象,先 init 再传入) */

    /* ---- 方向引脚(HAL GPIO,每轮两脚互补换向; 双脚同高=短路抱死) ---- */
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;          /* 前进脚(IN2): 前进=fwd_level, 后退=!fwd_level */
    uint16_t      dir_pin2;         /* 反向脚(IN1): 电平恒与 dir_pin 互补          */
    uint8_t       fwd_level;        /* 前进时 dir_pin 电平: 1=SET 0=RESET          */

    /* ---- 速度环 PID(内嵌,取代全局 g_pid_left/right) ---- */
    pid_t speed_pid;

    /* ---- 速度(mm/s 有符号;跨上下文读写 -> volatile) ---- */
    volatile int16_t ExpectSpeed;    /* 期望速度(应用层写) */
    volatile int16_t SoftSpeed;      /* 缓加速后的速度(PID 基准) */
    volatile int16_t RealSpeed;      /* 滤波后实测速度(PID 反馈) */
    volatile int16_t InstantSpeed;   /* 瞬时速度(无滤波) */
    volatile int32_t total_pulse;    /* 里程累计脉冲数(不分方向) */
    volatile WheelDirection_e Direction;  /* 当前运动方向 */

    /* ---- 测速中间量(ISR 写,主循环读+清,读取须短暂关中断防丢) ---- */
    volatile uint32_t pending_delta;   /* ISR 累加的脉冲增量(16位回卷安全) */
    volatile uint8_t  pending_periods;  /* ISR 累加的采样周期数(主循环漏拍折算用) */
    uint32_t          isr_last_count;   /* ISR 上一次读的计数值(仅 ISR 读写) */
    uint8_t           timeout;          /* 编码器超时计数 */

    /* ---- 滤波 ---- */
    filter_median_t median;          /* 3 点中值(去单周期尖峰) */
    filter_ewma_t   ewma;            /* EWMA 平滑 shift=1 */

    /* ---- 缓加速分频计数(dev_wheel 自管) ---- */
    uint8_t soft_div_cnt;

    /* ---- 输出斜率限制(上一拍限幅后的占空比输出;
              停转区清0 -> 重新起步输出从0缓升) ---- */
    int16_t out_slew;

    /* ---- 抱死锁定(仅主循环上下文读写) ---- */
    bool    locked;         /* brake 置位: update 跳过斜坡/PID,保持刹车图案;
                               set_speed 清零恢复闭环                           */

    /* ---- ISR->主循环 标志(本轮独立) ---- */
    volatile bool sample_ready;

} dev_wheel_t;

/* 全局实例 */
extern dev_wheel_t g_tLeftWheel;
extern dev_wheel_t g_tRightWheel;

/* ========== 4. API ========== */

/**
 * @brief 初始化轮对象。
 * @param self      轮对象指针
 * @param pwm       PWM 驱动对象指针（调用方已 init）
 * @param encoder   编码器驱动对象指针（调用方已 init）
 * @param dir_port  方向脚端口
 * @param dir_pin   方向脚 IN1
 * @param dir_pin2  方向脚 IN2
 * @param fwd_level 前进电平（dir_pin=fwd_level 时前进）
 * @note  内部 memset 清零；WheelRetreat==0，必须显式置 Direction 为 WheelForward。
 */
void   dev_wheel_init(dev_wheel_t *self,
                      drv_pwm_t *pwm,
                      drv_encoder_t *encoder,
                      GPIO_TypeDef *dir_port, uint16_t dir_pin,
                      uint16_t dir_pin2, uint8_t fwd_level);

/**
 * @brief 中频采样 ISR 体：读计数并累加 delta / 周期数。
 * @param self 轮对象指针
 * @note  中断上下文，只读硬件 + 累加，不做滤波/PID。
 * @note  delta/周期数由主循环消费时清零；漏拍时按周期数折算回速度。
 */
void   dev_wheel_on_sample_isr(dev_wheel_t *self);

/**
 * @brief 主循环每拍（约 20ms）更新：测速滤波 + 缓加速 + PID 输出。
 * @param self 轮对象指针
 * @note  抱死锁定期间测速照跑但里程不累计，同时跳过斜坡/PID。
 */
void   dev_wheel_update(dev_wheel_t *self);

/**
 * @brief 设置期望速度，并解除抱死锁定。
 * @param self       轮对象指针
 * @param speed_mm_s 期望速度（mm/s，带符号，负为后退）
 * @note  只改 ExpectSpeed，不立即改 PWM；靠缓加速渐变到目标。
 */
void   dev_wheel_set_speed(dev_wheel_t *self, int16_t speed_mm_s);

/**
 * @brief 硬抱死并保持。
 * @param self 轮对象指针
 * @note  两方向脚同置高（IN1=IN2=1，电机短路刹车），占空比拉满，
 *        并置 locked 让 update 跳过斜坡/PID，保持刹车图案。
 * @note  dev_wheel_set_speed 可解除锁定。
 */
void   dev_wheel_brake(dev_wheel_t *self);

/**
 * @brief 读取当前实测速度。
 * @param self 轮对象指针
 * @return 实测速度（mm/s，带符号）
 */
int16_t dev_wheel_get_speed(dev_wheel_t *self);

/**
 * @brief 读取累计里程脉冲数。
 * @param self 轮对象指针
 * @return 累计脉冲数（不分方向，抱死锁定期间不累计）
 */
int32_t dev_wheel_get_distance(dev_wheel_t *self);

#ifdef __cplusplus
}
#endif

#endif /* DEV_DEV_WHEEL_H_ */