/*
 * lib_font.h
 *
 *  库层：OLED 点阵字体资源（纯数据、无硬件依赖，故放 lib 层）。
 *
 *  特性：
 *    - 提供 4 套 ASCII 点阵字体（宽x高，单位像素）：6x8 / 6x12 / 8x16 / 12x24。
 *    - 字模为列行式（先列后行，每列按字节自上而下排列），与波特律动 OLED
 *      驱动的取模格式一致，可直接喂给 dev_oled 的 set_block。
 *    - ASCII 字体 chars 从空格 ' ' 起连续排列共 95 个可打印字符（' '~'~'），
 *      取字模只需按 (ch - ' ') 索引，无需查表。
 *    - 支持 UTF-8 混合字体（font_t）：可同时包含中文字模，ASCII 查不到时
 *      回退 font_ascii_t 显示。
 *
 *  分层原则：
 *    - 本层只有数据与结构体定义，不碰硬件、不依赖任何平台头文件。
 *    - 谁需要显示谁 include，dev_oled 的 print 系列按结构体描述取字模。
 *
 *  中文扩展（本项目默认不带中文字库）：
 *    - font_t：字库每项前 4 字节存字符的 UTF-8 编码（不足 4 字节补 0），
 *      其后为字模；打印时先按 UTF-8 编码匹配，命中即取后面的字模。
 *    - 中文字模可用波特律动取模工具生成（https://led.baud-dance.com），
 *      生成后追加到 lib_font.c 并在此补充 extern 声明即可。
 *
 *  使用前提：
 *    - 字模为列行式，取模方向/字节序需与 dev_oled_set_block 一致；
 *      换取模工具时务必对齐，否则字会横竖颠倒或错位。
 */

#ifndef LIB_LIB_FONT_H_
#define LIB_LIB_FONT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. ASCII 字体描述 ========== */

/* ===== ASCII 字体结构 =====
 *  chars 从空格 ' ' 起连续排列共 95 个可打印字符，取模时按 (ch - ' ') 索引。
 *  h/w 单位为像素，h 不要求是 8 的倍数（末尾不足一字节由 set_block 处理）。 */
typedef struct {
    uint8_t           h;      /* 字高（像素）          */
    uint8_t           w;      /* 字宽（像素）          */
    const uint8_t    *chars;  /* 字模数据（列行式）     */
} font_ascii_t;

/* ========== 2. UTF-8 混合字体描述 ========== */

/* ===== UTF-8 混合字体结构 =====
 *  字库每一项：前 4 字节 = 字符 UTF-8 编码（不足 4 字节补 0），其后为字模。
 *  len 为字符数；ascii 为缺省 ASCII 字体（查不到字模时回退使用）。 */
typedef struct {
    uint8_t                h;      /* 字高（像素）                            */
    uint8_t                w;      /* 字宽（像素）                            */
    const uint8_t         *chars;  /* 字库数据（UTF-8 编码 + 字模，列行式）     */
    uint8_t                len;    /* 字库字符数（>255 时请改为 uint16_t）     */
    const font_ascii_t    *ascii;  /* 缺省 ASCII 字体（查不到字模时回退使用）  */
} font_t;

/* ========== 3. 全局字体实例 ========== */

/* 全局字体实例（定义在 lib_font.c） */
extern const font_ascii_t g_font8x6;     /* 6 宽 x 8 高  */
extern const font_ascii_t g_font12x6;    /* 6 宽 x 12 高 */
extern const font_ascii_t g_font16x8;    /* 8 宽 x 16 高 */
extern const font_ascii_t g_font24x12;   /* 12 宽 x 24 高 */

#ifdef __cplusplus
}
#endif

#endif /* LIB_LIB_FONT_H_ */