# library_function

嵌入式 STM32 分层库：包含平台无关算法层（`lib/`）、STM32 HAL 驱动层（`platform/stm32/drv/`）和设备抽象层（`dev/`）。

## 软件架构

```
┌─────────────────────────────────────────────────┐
│                 应用层 (main)                    │
│         调用 dev 层 API 完成业务逻辑              │
├─────────────────────────────────────────────────┤
│  dev/  设备层                                    │
│    dev_key     按键扫描 + 消抖 + 按下事件         │
│    dev_track   8 路灰度/循迹读取 + 偏差计算       │
│    dev_wheel   电机轮：测速+滤波+PID+方向+PWM     │
├─────────────────────────────────────────────────┤
│  lib/  算法层（零平台依赖）                       │
│    lib_filter   3 点中值 / EWMA / 滑动平均       │
│    lib_pid      位置式 / 增量式 PID              │
│    lib_ringbuf  无锁单生产者-单消费者环形缓冲     │
│    lib_font     OLED 点阵 ASCII 字体（4 种尺寸）  │
├─────────────────────────────────────────────────┤
│  platform/stm32/drv/  驱动层（STM32 HAL 封装）    │
│    drv_gpio     GPIO 输出 / 输入封装             │
│    drv_pwm      TIM PWM 输出封装                 │
│    drv_encoder  TIM 编码器模式封装               │
│    drv_uart     UART + DMA 循环接收 + IDLE 中断   │
├─────────────────────────────────────────────────┤
│  STM32 HAL / CubeMX 生成代码                     │
└─────────────────────────────────────────────────┘
```

## 安装教程

1. 使用 STM32CubeMX 生成工程，配置好 GPIO / TIM / UART / DMA 外设。
2. 将 `lib/`、`dev/`、`platform/stm32/drv/` 目录复制到工程中。
3. 在 IDE（Keil / STM32CubeIDE / Makefile）中添加源文件和头文件路径：
   - Include Paths: `lib/`、`dev/`、`platform/stm32/drv/`（或按实际目录层级调整）
   - Source Files: 各目录下的 `.c` 文件
4. 根据实际芯片型号修改驱动头文件中的 `#include "stm32f1xx_hal.h"`。

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

### PID（lib_pid）

```c
pid_t pid;
pid_init(&pid, PID_MODE_POSITION);
pid_set_gains(&pid, 1.2f, 0.05f, 0.01f);
pid_set_limits(&pid, -400.0f, 400.0f, -100.0f, 100.0f);
float out = pid_calc(&pid, target, measured);
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

1. `dev_wheel.c` 调用了 `drv_gpio_write_pair()`，但该函数未在 `drv_gpio` 中定义，链接时会报错。
2. 部分源文件编码为 GBK/GB2312，部分为 UTF-8，跨平台查看时中文注释可能乱码。

## 参与贡献

1. Fork 本仓库
2. 新建 `Feat_xxx` 分支
3. 提交代码
4. 新建 Pull Request
