# library_function

嵌入式分层库：包含平台无关算法层（`lib/`）、芯片驱动层（`platform/stm32/`、`platform/renesas/`）和设备抽象层（`dev/`）。

## 软件架构

```
┌─────────────────────────────────────────────────────┐
│                   应用层 (main)                      │
│           调用 dev 层 API 完成业务逻辑                │
├─────────────────────────────────────────────────────┤
│  dev/  设备层                                        │
│    dev_key       按键扫描 + 消抖 + 按下事件           │
│    dev_track     8 路灰度/循迹读取 + 偏差计算         │
│    dev_wheel     电机轮：测速+滤波+PID+方向+PWM       │
│    dev_servo     SG90/MG996R 舵机：角度→脉宽映射     │
│    dev/OLED/SSD1306/dev_oled  OLED 文字显示（帧缓存）│
├─────────────────────────────────────────────────────┤
│  lib/  算法层（零平台依赖）                           │
│    lib_filter     3 点中值 / EWMA / 滑动平均         │
│    lib_pid        位置式 / 增量式 PID（抗饱和等）     │
│    lib_ringbuf    无锁单生产者-单消费者环形缓冲       │
│    lib_font       OLED 点阵 ASCII 字体（4 种尺寸）    │
├─────────────────────────────────────────────────────┤
│  platform/  驱动层（按芯片选型，二选一或并存）         │
│                                                  │
│  platform/stm32/drv/        platform/renesas/        │
│  （STM32 HAL 封装）          （Renesas FSP 封装）     │
│    drv_gpio   GPIO 输出/输入    drv_gpio  GPIO 输出  │
│    drv_pwm    TIM PWM 输出      drv_adc   ADC 扫描   │
│    drv_encoder TIM 编码器模式                          │
│    drv_uart   UART+DMA+IDLE                            │
├─────────────────────────────────────────────────────┤
│  STM32 HAL / CubeMX 生成代码  或  Renesas FSP / e2    │
└─────────────────────────────────────────────────────┘
```

## 安装教程

### STM32 平台

1. 使用 STM32CubeMX 生成工程，配置好 GPIO / TIM / UART / DMA 外设。
2. 将 `lib/`、`dev/`、`platform/stm32/drv/` 目录复制到工程中。
3. 在 IDE（Keil / STM32CubeIDE / Makefile）中添加源文件和头文件路径：
   - Include Paths: `lib/`、`dev/`、`platform/stm32/drv/`（或按实际目录层级调整）
   - Source Files: 各目录下的 `.c` 文件
4. 根据实际芯片型号修改驱动头文件中的 `#include "stm32f1xx_hal.h"`。

### Renesas 平台

1. 使用 e² studio / FSP 配置工程，生成 `hal_data.h` 及 FSP 实例。
2. 将 `lib/`、`dev/`、`platform/renesas/` 目录复制到工程中。
3. 添加 Include Paths：`lib/`、`dev/`、`platform/renesas/`，并加入对应 `.c` 文件。
4. 驱动直接依赖 FSP 生成的控制块（如 `g_adc0_ctrl`），按各驱动头文件注释传入。

## 使用说明

### 按键（dev_key）

```c
dev_key_init(&g_tKey, GPIOE, GPIO_PIN_3, "KEY");
dev_key_init(&g_tKeyOk, GPIOE, GPIO_PIN_4, "OK");

// 主循环固定节拍（约 20ms）调用
dev_key_scan(&g_tKey);
dev_key_scan(&g_tKeyOk);

if (dev_key_get_press(&g_tKey)) { /* 按下事件 */ }
```

### 循迹传感器（dev_track）

```c
dev_track_init(&g_tTrack);

// 定时器中断（推荐 10ms）中调用
dev_track_update_isr(&g_tTrack);

// 主循环中读取
int8_t  err   = g_tTrack.error;    // -7 ~ +7
uint8_t cross = g_tTrack.line_all; // 1 = 十字/丢线
```

### 电机轮（dev_wheel）

```c
// 初始化（PWM / 编码器需先 init）
dev_wheel_init(&g_tLeftWheel, &pwm_left, &enc_left,
               GPIOA, GPIO_PIN_0, GPIO_PIN_1, 1);

// 10ms 采样定时器中断中
dev_wheel_on_sample_isr(&g_tLeftWheel);

// 主循环（约 20ms）中
dev_wheel_update(&g_tLeftWheel);
dev_wheel_set_speed(&g_tLeftWheel, 300);  // 300 mm/s
```

### 舵机（dev_servo）

```c
// 初始化（CubeMX 已配好 TIM：1MHz 计数、50Hz 周期，ARR=19999）
dev_servo_init(&pwm_servo);

// 设置角度（整度，越界自动夹到 [0,180]）
dev_servo_set_angle(90);

// 毫度接口（内部四舍五入到整度，拒绝亚度噪声）
dev_servo_set_mdeg(90500);   // 90.5° → 实际输出 91°

int16_t deg  = dev_servo_get_angle();  // 读取整度
int32_t mdeg = dev_servo_get_mdeg();   // 读取毫度
```

### OLED 显示（dev_oled）

```c
// 1. 实现底层 I2C 发送函数（封装 HAL）
static void oled_i2c_send(uint8_t addr, uint8_t *data, uint8_t len)
{
    HAL_I2C_Master_Transmit(&hi2c1, addr, data, len, 100);
}

// 2. 初始化（MX_I2C1_Init() 之后延时约 20ms 再调）
dev_oled_init(oled_i2c_send);

// 3. 主循环：文字写入显存，再整帧推送
dev_oled_new_frame();
dev_oled_print_string(0, 0, "Speed:300", &g_font16x8, DEV_OLED_COLOR_NORMAL);
dev_oled_show_frame();   // 阻塞约 25ms，禁止在中断中调用
```

### PID（lib_pid）

```c
pid_t pid;
pid_init(&pid, PID_MODE_POSITION);
pid_set_gains(&pid, 1.2f, 0.05f, 0.01f);
pid_set_limits(&pid, -400.0f, 400.0f, -100.0f, 100.0f);
pid_set_d_on_meas(&pid, true);   // 微分对测量值求导，避免设定值突变冲击
pid_enable(&pid, true);          // 软开关，false 时输出恒 0 且不累积积分

float out = pid_calc(&pid, target, measured);
```

### 滤波（lib_filter）

```c
// 3 点中值滤波（去尖峰）
filter_median_t med;
filter_median_init(&med);
int16_t med_out = filter_median_push(&med, adc_raw);

// EWMA 滤波（平滑，alpha = 1/2^shift）
filter_ewma_t ewma;
filter_ewma_init(&ewma, 1);
int16_t ewma_out = filter_ewma_update(&ewma, input);

// 滑动平均滤波（窗口大小 8）
int16_t movavg_buf[8];
filter_movavg_t movavg;
filter_movavg_init(&movavg, movavg_buf, 8);
int16_t movavg_out = filter_movavg_push(&movavg, input);
```

### 环形缓冲（lib_ringbuf）

```c
ringbuf_t rb;
uint8_t   buf[256];
ringbuf_init(&rb, buf, sizeof(buf));
ringbuf_push(&rb, data);        // ISR / 生产者
ringbuf_pop(&rb, &byte);        // 主循环 / 消费者
```

## 已知问题

1. 部分源文件编码为 GBK/GB2312（如 `drv_gpio.h`、`lib_filter.h`），部分为 UTF-8，跨平台查看时中文注释可能乱码。建议统一转为 UTF-8。

## 参与贡献

1. Fork 本仓库
2. 新建 `Feat_xxx` 分支
3. 提交代码
4. 新建 Pull Request
