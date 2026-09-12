/*
 * dev_oled.h
 *
 *  设备层：0.96 寸 OLED（SSD1306，128x64，I2C 7 位地址 0x3C），仅文字显示。
 *
 *  特性：
 *    - 帧缓存模式：print 系列只写显存，dev_oled_show_frame() 整帧推送，
 *      文字写入与屏幕刷新解耦，避免边写边刷的撕裂。
 *    - 支持中英混排（UTF-8）：中文字模查不到时显示空格，ASCII 字符回退到
 *      font->ascii 显示。
 *    - 颜色双模：DEV_OLED_COLOR_NORMAL 画亮点，DEV_OLED_COLOR_REVERSED
 *      画暗点（可用于在已绘制内容上局部擦除）。
 *    - 文字 API 均带越界保护，超屏坐标直接丢弃，不崩溃、不画错位。
 *    - 显存 1KB 与 I2C 发送缓冲均为模块内部静态资源，调用者无需管理。
 *
 *  分层原则：
 *    - 本模块不包含任何平台相关头文件，可在任意平台复用。
 *    - 底层 I2C 发送通过 dev_oled_send_fn 函数指针注入（dev_oled_init 时
 *      传入），更换平台只需重新实现该函数，本模块无需修改。
 *    - 字体资源见 lib_font.h（g_font8x6 / g_font12x6 / g_font16x8 /
 *      g_font24x12），本模块按 font_ascii_t / font_t 描述取字模。
 *
 *  使用前提：
 *    1. I2C 硬件初始化由 CubeMX 生成的 i2c.c 完成
 *       （I2C1：PB6=SCL / PB7=SDA，400kHz）。
 *    2. MX_I2C1_Init() 之后延时约 20ms（MCU 启动快于 OLED 上电），
 *       再调 dev_oled_init(send_fn)。
 *    3. I2C 为阻塞传输，dev_oled_show_frame() 约 25ms，请在主循环上下文
 *       调用本模块；禁止在中断服务函数中使用。
 *
 *  坐标系：原点在左上角，x 向右 0~127，y 向下 0~63。
 *
 *  示例：
 *    // 1. 实现底层发送函数（示例，封装 HAL）
 *    static void oled_i2c_send(uint8_t addr, uint8_t *data, uint8_t len)
 *    {
 *        HAL_I2C_Master_Transmit(&hi2c1, addr, data, len, 100);
 *    }
 *
 *    // 2. 初始化（MX_I2C1_Init() 之后延时约 20ms 再调）
 *    dev_oled_init(oled_i2c_send);
 *
 *    // 3. 主循环：文字写入与刷新分离
 *    dev_oled_new_frame();                                      // 开始新一帧（清空显存）
 *    dev_oled_print_string(0, 0, "速度:300", &g_font16x8,       // 写入显存
 *                          DEV_OLED_COLOR_NORMAL);
 *    dev_oled_show_frame();                                     // 整帧推送（阻塞约25ms）
 *
 *  移植自波特律动 OLED 驱动（keysking，MIT License），按本工程需求裁剪为
 *  仅文字显示（图形绘制 API 已移除），命名按本工程规范调整为 dev_oled_xxx。
 */

#ifndef DEV_DEV_OLED_H_
#define DEV_DEV_OLED_H_

#include "lib/lib_font.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 颜色模式 ========== */

/* ===== 显存/点阵绘制颜色 =====
 *  NORMAL   画亮点（黑底白字）
 *  REVERSED 画暗点（用于在已绘制内容上局部擦除） */
typedef enum {
    DEV_OLED_COLOR_NORMAL = 0,    /* 正常模式：画亮点（黑底白字） */
    DEV_OLED_COLOR_REVERSED       /* 反色模式：画暗点（局部擦除用） */
} dev_oled_color_t;

/* ========== 2. 底层发送函数注入 ========== */

/**
 * @brief 底层 I2C 发送函数指针类型。
 * @param addr  从机地址（已左移，例如 0x78；HAL 发送时会清 bit0）
 * @param data  待发送数据缓冲区（首字节为控制字节）
 * @param len   发送字节数
 * @note  调用者需自行实现该函数（例如封装 HAL_I2C_Master_Transmit），
 *        并在 dev_oled_init() 时注入，实现与具体 I2C 方式（硬件/软件/
 *        其他平台）的解耦。更换平台时只需重新实现该函数。
 * @note  函数应保证 data 在调用期间有效（本模块传入的是静态缓冲）。
 */
typedef void (*dev_oled_send_fn)(uint8_t addr, uint8_t *data, uint8_t len);

/* ========== 3. 初始化与显示控制 ========== */

/**
 * @brief 初始化 OLED。
 * @param send_fn 底层发送函数（不可为 NULL，否则直接返回）
 * @note  必须在 MX_I2C1_Init() 之后、并延时约 20ms 后调用（MCU 上电快于
 *        OLED，过早发命令可能丢）。
 * @note  下发 SSD1306 128x64 标准命令序列，最后清屏并开启显示；返回后
 *        屏幕即为可用状态。
 */
void dev_oled_init(dev_oled_send_fn send_fn);

/**
 * @brief 开启显示：先开电荷泵，再点亮屏幕。
 * @note  显存内容保留，重新点亮后立即恢复原画面。
 */
void dev_oled_display_on(void);

/**
 * @brief 关闭显示：先关电荷泵再关屏幕，进入省电模式。
 * @note  显存内容保留，下次 display_on 后可继续显示。
 */
void dev_oled_display_off(void);

/**
 * @brief 设置整屏颜色模式（硬件指令，不经过显存）。
 * @param mode 正常（黑底白字）/ 反色（白底黑字）
 * @note  只影响显示输出，显存内容不变；适合做闪烁提示等效果。
 */
void dev_oled_set_color_mode(dev_oled_color_t mode);

/* ========== 4. 帧刷新 ========== */

/**
 * @brief 清空显存，开始绘制新的一帧。
 * @note  只清显存，屏幕在 show_frame() 前不变。
 */
void dev_oled_new_frame(void);

/**
 * @brief 将显存整帧推送到屏幕（按页分 8 次传输）。
 * @note  400kHz 下整帧约 25ms，为阻塞操作，请在主循环上下文调用。
 * @note  推完屏幕才更新，避免边写边刷的撕裂。
 */
void dev_oled_show_frame(void);

/* ========== 5. 文字显示（只写显存，show_frame 后生效） ========== */

/**
 * @brief 显示一个 ASCII 字符。
 * @param x,y  字符左上角像素坐标
 * @param ch   待显示字符（' '~'~' 之外按空格处理）
 * @param font ASCII 字模（font_ascii_t，含宽高与字模数据）
 * @param color 颜色模式（NORMAL 画亮点 / REVERSED 画暗点）
 * @note  非可打印字符直接替换为空格，防止字模数组越界（原实现会越界）。
 */
void dev_oled_print_ascii_char(uint8_t x, uint8_t y, char ch, const font_ascii_t *font, dev_oled_color_t color);

/**
 * @brief 显示一个 ASCII 字符串。
 * @param x,y  起始像素坐标
 * @param str  以 '\0' 结尾的字符串
 * @param font ASCII 字模
 * @param color 颜色模式
 * @note  每个字符按 font->w 步进；超出屏宽 256 后 x 回绕（uint8_t 自然
 *        回绕，与参考实现一致），注意控制内容长度。
 */
void dev_oled_print_ascii_string(uint8_t x, uint8_t y, const char *str, const font_ascii_t *font, dev_oled_color_t color);

/**
 * @brief 显示字符串（UTF-8，可中英混排，需配套字库）。
 * @param x,y  起始像素坐标
 * @param str  UTF-8 字符串（编译器字符集需设为 UTF-8）
 * @param font 中英混排字库（font_t，含 chars 表与 ascii 回退字模）
 * @param color 颜色模式
 * @note  中文字模查不到时显示空格；ASCII 字符回退到 font->ascii 显示。
 * @note  字库请在波特律动取模工具生成：https://led.baud-dance.com
 * @note  逐字符在字库中线性查找（TODO：可优化为二分/哈希，当前字符数少够用）。
 */
void dev_oled_print_string(uint8_t x, uint8_t y, const char *str, const font_t *font, dev_oled_color_t color);

#ifdef __cplusplus
}
#endif

#endif /* DEV_DEV_OLED_H_ */