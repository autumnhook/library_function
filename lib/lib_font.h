/*
 * lib_font.h
 *
 *  字体库：OLED 点阵字体资源（纯数据、无硬件依赖，故放 lib 层）。
 *  提供 4 套 ASCII 点阵字体（宽x高，单位像素）：6x8 / 6x12 / 8x16 / 12x24。
 *  字模为列行式（先列后行，每列按字节自上而下排列），
 *  与波特律动 OLED 驱动的取模格式一致。
 *
 *  中文扩展（本项目默认不带中文字库）：
 *    - font_t  UTF-8 混合字体：字库每项前 4 字节存字符的 UTF-8 编码，
 *              其余为字模；ASCII 字符查不到时回退 font_ascii_t 显示
 *    - 字模可用波特律动取模工具生成（https://led.baud-dance.com），
 *      生成后追加到 lib_font.c 并在此补充 extern 声明即可
 */

#ifndef LIB_LIB_FONT_H_
#define LIB_LIB_FONT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ASCII 字体：chars 从空格 ' ' 起连续排列共 95 个可打印字符 ---- */
typedef struct {
    uint8_t           h;      /* 字高（像素）          */
    uint8_t           w;      /* 字宽（像素）          */
    const uint8_t    *chars;  /* 字模数据（列行式）     */
} font_ascii_t;

/* ---- UTF-8 混合字体：可同时包含中文字模 ----
 * 字库每一项：前 4 字节 = 字符 UTF-8 编码（不足 4 字节补 0），其后为字模 */
typedef struct {
    uint8_t                h;      /* 字高（像素）                            */
    uint8_t                w;      /* 字宽（像素）                            */
    const uint8_t         *chars;  /* 字库数据（UTF-8 编码 + 字模，列行式）     */
    uint8_t                len;    /* 字库字符数（>255 时请改为 uint16_t）     */
    const font_ascii_t    *ascii;  /* 缺省 ASCII 字体（查不到字模时回退使用）  */
} font_t;

/* 全局字体实例（定义在 lib_font.c） */
extern const font_ascii_t g_font8x6;     /* 6 宽 x 8 高  */
extern const font_ascii_t g_font12x6;    /* 6 宽 x 12 高 */
extern const font_ascii_t g_font16x8;    /* 8 宽 x 16 高 */
extern const font_ascii_t g_font24x12;   /* 12 宽 x 24 高 */

#ifdef __cplusplus
}
#endif

#endif /* LIB_LIB_FONT_H_ */
