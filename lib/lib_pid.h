/*
 * lib_pid.h
 *
 *  通用 PID 库。
 *
 *  特性：
 *    - 零平台依赖，仅使用 <stdint.h> 和 <stdbool.h>，不依赖任何 HAL / RTOS / 编译器符号。
 *    - 位置式 / 增量式两种模式（本项目仅用位置式，功能与旧版一致）。
 *    - 位置式积分分离抗饱和：P+I 预览判饱和，不含 D 项。
 *    - 微分可对测量值求导（d_on_meas），避免 setpoint 突变微分冲击。
 *    - 首拍 D=0，防微分冲击。
 *    - 软开关 pid_enable()：false 时输出恒 0 且不累积。
 *    - 角度环接口 pid_calc_rate()：caller 自行算误差并提供外部速率作为 D 项。
 *    - 扩展接口 pid_calc_ext()：caller 直接提供 D 项。
 *
 *  标准“并联（误差决定）”形式：
 *    位置式：out = Kp*e + Ki*int(e) + Kd*de
 *    增量式：out += Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2)
 *
 *  使用前提：
 *    1. 调用方需先 pid_init() 设置 mode，再设置增益、限幅等参数。
 *    2. 无 dt 参数，Ki/Kd 按固定采样周期整定，量纲隐含采样周期。
 *    3. 位置式与增量式共用 integ，但含义不同：
 *         位置式中 integ 是积分项；
 *         增量式中 integ 是输出累积器。
 *       切换模式或混用接口前建议 pid_reset()。
 *    4. 默认不限幅，若不调用 pid_set_limits()，输出与积分实际上不受限。
 *
 *  示例：
 *    // 速度环（位置式）
 *    pid_t pid;
 *    pid_init(&pid, PID_MODE_POSITION);
 *    pid_set_gains(&pid, 1.2f, 0.05f, 0.01f);
 *    pid_set_limits(&pid, -400.0f, 400.0f, -100.0f, 100.0f);
 *    pid_set_d_on_meas(&pid, true);
 *    pid_enable(&pid, true);
 *    float out = pid_calc(&pid, target_speed, real_speed);
 *
 *    // 角度环（外部速率作 D）
 *    float err  = wrap_pm180(target_angle - angle);
 *    float rate = gyro_z;
 *    float u    = pid_calc_rate(&pid, err, rate);
 */

#ifndef LIB_LIB_PID_H_
#define LIB_LIB_PID_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 模式与结构体 ========== */

/**
 * @brief PID 工作模式。
 */
typedef enum {
    PID_MODE_POSITION,        /* 位置式：out = Kp*e + Ki*int(e) + Kd*de          */
    PID_MODE_INCREMENTAL,     /* 增量式：out += Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2) */
} pid_mode_t;

typedef struct {
    /* ---- 参数区（按需修改） ---- */
    float      kp, ki, kd;             /* 三个增益                            */
    float      out_min, out_max;       /* 输出限幅（速度环 ±400 等）           */
    float      integ_min, integ_max;   /* 积分限幅                             */
    pid_mode_t mode;                   /* 位置式 / 增量式                      */
    bool       d_on_meas;              /* true: D 对测量值，D = -Kd*dmeas      */

    /* ---- 运行状态（调用方别改） ---- */
    bool       enabled;                /* false: 软关断，输出恒 0              */
    float      integ;                  /* 位置式=积分项；增量式=输出累积(当前out) */
    float      prev_err, prev2_err;    /* 增量式历史误差 / 备用                */
    float      prev_meas;              /* d_on_meas 用 dmeas                   */
    bool       primed;                 /* 是否跑过一拍（内部微分防首次冲击）    */
} pid_t;

/* ========== 2. 初始化与配置 ========== */

/**
 * @brief 初始化 PID 对象。
 * @param pid   PID 对象指针
 * @param mode  工作模式（位置式 / 增量式）
 * @note  增益清零，输出与积分限幅设为“无限制”，enabled=true；
 *        运行历史（integ / prev_* / primed）清零。
 */
void  pid_init          (pid_t *pid, pid_mode_t mode);

/**
 * @brief 设置三个增益。
 * @param pid PID 对象指针
 * @param kp  比例增益
 * @param ki  积分增益
 * @param kd  微分增益
 */
void  pid_set_gains     (pid_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限幅与积分限幅。
 * @param pid       PID 对象指针
 * @param out_min   输出下限
 * @param out_max   输出上限
 * @param integ_min 积分下限
 * @param integ_max 积分上限
 * @note  不调用本函数时，输出与积分实际上不受限。
 */
void  pid_set_limits    (pid_t *pid, float out_min, float out_max,
                                       float integ_min, float integ_max);

/**
 * @brief 选择微分是否作用在测量值上。
 * @param pid PID 对象指针
 * @param on  true: D 对测量值，D = -Kd*dmeas；false: D 对误差，D = Kd*de
 * @note  切换后旧历史自动作废（首拍 D=0）。
 */
void  pid_set_d_on_meas (pid_t *pid, bool on);

/**
 * @brief 软开关。
 * @param pid PID 对象指针
 * @param en  false: 软关断，所有计算接口恒返 0 且不累积；true: 使能
 * @note  不清积分与历史，重新使能会接着旧状态跑；如需清干净请调 pid_reset()。
 */
void  pid_enable        (pid_t *pid, bool en);

/**
 * @brief 清积分与历史，保留参数 / 限幅 / enabled。
 * @param pid PID 对象指针
 * @note  适用于模式切换、重新使能前、需要无扰重启时。
 */
void  pid_reset         (pid_t *pid);

/* ========== 3. 计算接口 ========== */

/**
 * @brief 标准接口：内部算 error = setpoint - measurement，D 为内部微分。
 * @param pid         PID 对象指针
 * @param setpoint    目标值
 * @param measurement 测量值
 * @return 控制量；enabled=false 时恒返 0
 * @note  增量式模式下直接走增量核心；位置式下 D 支持 d_on_meas；
 *        首拍 primed=false 时 D=0，防微分冲击。
 */
float pid_calc          (pid_t *pid, float setpoint, float measurement);

/**
 * @brief 角度环接口：caller 完全接管误差（如 ±180° 最短路径误差），
 *        并提供外部测量速率作为 D 项。
 * @param pid       PID 对象指针
 * @param error     外部误差（caller 自行计算，含角度归一化）
 * @param meas_rate 外部测量速率（如陀螺仪角速度）
 * @return 控制量；enabled=false 时恒返 0
 * @note  D = -Kd*meas_rate，始终按位置式处理（增量式不适合外部速率输入）。
 * @note  本接口不更新 prev_meas，只更新 prev_err / primed。
 */
float pid_calc_rate     (pid_t *pid, float error, float meas_rate);

/**
 * @brief 扩展接口：caller 直接提供 D 项（如摄像头速度阻尼替代位置微分）。
 * @param pid         PID 对象指针
 * @param setpoint    目标值
 * @param measurement 测量值
 * @param d_term      外部 D 项
 * @return 控制量；enabled=false 时恒返 0
 * @note  内部：out = Kp*e + I + d_term，I 带积分分离抗饱和处理。
 */
float pid_calc_ext      (pid_t *pid, float setpoint, float measurement,
                         float d_term);

#ifdef __cplusplus
}
#endif

#endif /* LIB_LIB_PID_H_ */