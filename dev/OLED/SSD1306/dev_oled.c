/*
 * dev_oled.c
 *
 *  设备层实现：SSD1306 OLED（128x64）驱动，仅文字显示。
 *
 *  特性：
 *    - 帧缓存模式：print 系列只写显存 s_gram，dev_oled_show_frame() 按页
 *      （8 页 x 128 列）将整帧显存推到屏幕，文字写入与刷新解耦。
 *    - 显存与内部显存操作函数全部 static 私有化，不暴露给调用者。
 *    - 文字写入带越界保护：页/列超范围直接丢弃，不崩溃、不画错位。
 *    - ASCII 字符带可打印范围检查，控制字符按空格处理，防字模数组越界。
 *    - 颜色支持 NORMAL（画亮点）/ REVERSED（画暗点，局部擦除用）。
 *
 *  分层原则：
 *    - 本文件不包含任何平台相关头文件，可在任意平台复用。
 *    - 底层 I2C 发送通过 dev_oled_init() 注入的 dev_oled_send_fn 完成，
 *      更换平台只需重新实现该函数，本文件无需修改。
 *
 *  相对参考实现（波特律动 keysking，MIT License）的调整：
 *    - 按需求裁剪：移除全部图形绘制 API（线/矩形/三角/圆/椭圆/图片）及其
 *      专用底层函数（set_pixel / set_byte），仅保留文字显示与显示控制。
 *    - 命名按工程规范改为 dev_oled_xxx，显存与内部显存操作函数私有化
 *      （static）。
 *    - print_ascii_char 增加可打印范围检查（原实现收到控制字符会数组越界）。
 *    - 底层 I2C 发送改为函数指针注入，解除平台耦合。
 *    - 字库差异见 lib_font.c 头注释（8x6 补齐 '|'/'}'/'~' 三个字模）。
 */

#include "dev/dev_oled.h"
#include <string.h>

/* OLED 器件地址：7 位地址 0x3C，0x78 = 0x3C << 1（HAL 发送时清除 bit0） */
#define DEV_OLED_ADDRESS     0x78u

/* OLED 几何参数（SSD1306 128x64） */
#define DEV_OLED_PAGE        8u                     /* 页数（每页 8 行） */
#define DEV_OLED_ROW         (8u * DEV_OLED_PAGE)   /* 总行数 64        */
#define DEV_OLED_COLUMN      128u                   /* 总列数           */

/* 帧缓存显存：[页][列]，每字节覆盖 8 行（列行式） */
static uint8_t s_gram[DEV_OLED_PAGE][DEV_OLED_COLUMN];

/* 底层发送函数指针（由 dev_oled_init 注入） */
static dev_oled_send_fn s_send = NULL;

/* ============ 1. 底层通信 ============ */

/**
 * @brief 向 OLED 发送一段 I2C 数据（首字节为控制字节，由调用者填好）。
 * @param data 待发送缓冲区（首字节为控制字节）
 * @param len  发送字节数
 * @note  实际发送由注入的 s_send 完成；未注入时静默丢弃（初始化前调用
 *        也不会崩）。
 */
static void dev_oled_send(uint8_t *data, uint8_t len)
{
    if (s_send != NULL)
        s_send(DEV_OLED_ADDRESS, data, len);
}

/**
 * @brief 向 OLED 发送一条指令。
 * @param cmd 指令字节
 * @note  控制字节固定 0x00（后续字节按命令解析）。
 * @note  send_buffer 用 static：避免每次在栈上开缓冲，且生命周期覆盖整个
 *        发送过程。
 */
static void dev_oled_send_cmd(uint8_t cmd)
{
    static uint8_t send_buffer[2] = {0};    /* [0] = 0x00：后续字节为指令 */

    send_buffer[1] = cmd;
    dev_oled_send(send_buffer, 2u);
}

/* ============ 2. 初始化与显示控制 ============ */

/**
 * @brief 初始化 OLED（SSD1306 128x64 标准命令序列）。
 * @param send_fn 底层发送函数（不可为 NULL，否则直接返回）
 * @note  必须在 MX_I2C1_Init() 之后调用；上电后建议先延时约 20ms。
 * @note  命令序列依次配置：寻址模式 / 扫描方向 / 对比度 / 段重映射 / 复用率 /
 *        电荷泵等，最后清屏并开启显示。
 */
void dev_oled_init(dev_oled_send_fn send_fn)
{
    s_send = send_fn;
    if (s_send == NULL)
        return;

    dev_oled_send_cmd(0xAE);           /* 关闭显示                            */

    dev_oled_send_cmd(0x20);
    dev_oled_send_cmd(0x10);           /* 寻址模式：页模式                    */

    dev_oled_send_cmd(0xB0);           /* 起始页地址 = 0                      */
    dev_oled_send_cmd(0xC8);           /* COM 扫描方向：COM0 -> COM63         */

    dev_oled_send_cmd(0x00);           /* 列起始地址低 4 位 = 0               */
    dev_oled_send_cmd(0x10);           /* 列起始地址高 4 位 = 0               */

    dev_oled_send_cmd(0x40);           /* 显示起始行 = 0                      */

    dev_oled_send_cmd(0x81);
    dev_oled_send_cmd(0xDF);           /* 对比度 = 0xDF                       */

    dev_oled_send_cmd(0xA1);           /* 段重映射：列 127 映射到 SEG0        */
    dev_oled_send_cmd(0xA6);           /* 正常显示（非反色）                  */

    dev_oled_send_cmd(0xA8);
    dev_oled_send_cmd(0x3F);           /* 复用率 = 64 行                      */

    dev_oled_send_cmd(0xA4);           /* 输出跟随显存内容                    */

    dev_oled_send_cmd(0xD3);
    dev_oled_send_cmd(0x00);           /* 显示偏移 = 0                        */

    dev_oled_send_cmd(0xD5);
    dev_oled_send_cmd(0xF0);           /* 振荡频率/分频（高刷新率）           */

    dev_oled_send_cmd(0xD9);
    dev_oled_send_cmd(0x22);           /* 预充电周期                          */

    dev_oled_send_cmd(0xDA);
    dev_oled_send_cmd(0x12);           /* COM 引脚配置（128x64 备用模式）     */

    dev_oled_send_cmd(0xDB);
    dev_oled_send_cmd(0x20);           /* VCOMH 电压                          */

    dev_oled_send_cmd(0x8D);
    dev_oled_send_cmd(0x14);           /* 开启电荷泵                          */

    dev_oled_new_frame();
    dev_oled_show_frame();             /* 清屏                               */

    dev_oled_send_cmd(0xAF);           /* 开启显示                            */
}

/**
 * @brief 开启 OLED 显示（重新点亮屏幕）。
 * @note  先开电荷泵再点亮；显存内容保留，重新点亮后立即恢复原画面。
 */
void dev_oled_display_on(void)
{
    dev_oled_send_cmd(0x8D);           /* 电荷泵使能 */
    dev_oled_send_cmd(0x14);           /* 开启电荷泵 */
    dev_oled_send_cmd(0xAF);           /* 点亮屏幕   */
}

/**
 * @brief 关闭 OLED 显示（省电，显存内容保留）。
 * @note  先关电荷泵再关屏幕；显存内容保留，下次 display_on 后可继续显示。
 */
void dev_oled_display_off(void)
{
    dev_oled_send_cmd(0x8D);           /* 电荷泵使能 */
    dev_oled_send_cmd(0x10);           /* 关闭电荷泵 */
    dev_oled_send_cmd(0xAE);           /* 关闭屏幕   */
}

/**
 * @brief 设置整屏颜色模式（硬件指令，不经过显存）。
 * @param mode 正常（黑底白字）/ 反色（白底黑字）
 * @note  只影响显示输出，显存内容不变；适合做闪烁提示等效果。
 */
void dev_oled_set_color_mode(dev_oled_color_t mode)
{
    if (mode == DEV_OLED_COLOR_NORMAL)
        dev_oled_send_cmd(0xA6);       /* 正常显示 */
    else
        dev_oled_send_cmd(0xA7);       /* 反色显示 */
}

/* ============ 3. 帧刷新 ============ */

/**
 * @brief 清空显存，开始新的一帧。
 * @note  只清显存，屏幕在 show_frame() 前不变。
 */
void dev_oled_new_frame(void)
{
    memset(s_gram, 0, sizeof(s_gram));
}

/**
 * @brief 将显存整帧推送到屏幕（按页分 8 次传输）。
 * @note  400kHz 下整帧约 25ms，为阻塞操作，请在主循环上下文调用。
 * @note  send_buffer 用 static：129 字节在栈上偏大，且发送由底层阻塞完成。
 */
void dev_oled_show_frame(void)
{
    static uint8_t send_buffer[DEV_OLED_COLUMN + 1u];

    send_buffer[0] = 0x40;             /* 控制字节：后续为显存数据 */
    for (uint8_t page = 0u; page < DEV_OLED_PAGE; page++)
    {
        dev_oled_send_cmd(0xB0u + page);            /* 页地址        */
        dev_oled_send_cmd(0x00u);                   /* 列地址低 4 位 */
        dev_oled_send_cmd(0x10u);                   /* 列地址高 4 位 */
        memcpy(&send_buffer[1], s_gram[page], DEV_OLED_COLUMN);
        dev_oled_send(send_buffer, (uint8_t)(DEV_OLED_COLUMN + 1u));
    }
}

/* ============ 4. 显存操作（内部） ============ */

/**
 * @brief 设置显存中某个字节的第 start~end 位为 data 对应位（位级写入）。
 * @param page   页号 0~7
 * @param column 列号 0~127
 * @param data   待写入数据（REVERSED 时自动取反）
 * @param start  起始位 0~7
 * @param end    结束位 0~7（start <= end）
 * @param color  颜色模式（REVERSED 时 data 取反）
 * @note  越界（page/column 超范围）直接丢弃，不崩溃。
 * @note  以像素坐标操作的接口请用 set_bits / set_bits_fine（可跨页）。
 */
static void dev_oled_set_byte_fine(uint8_t page, uint8_t column, uint8_t data,
                                   uint8_t start, uint8_t end, dev_oled_color_t color)
{
    uint8_t mask;    /* start~end 区间的位掩码 */

    if ((page >= DEV_OLED_PAGE) || (column >= DEV_OLED_COLUMN))
        return;
    if (color == DEV_OLED_COLOR_REVERSED)
        data = (uint8_t)~data;

    mask = (uint8_t)((0xFFu << start) & (0xFFu >> (7u - end)));
    s_gram[page][column] = (uint8_t)((s_gram[page][column] & (uint8_t)~mask) | (data & mask));
}

/**
 * @brief 从像素坐标 (x,y) 向下写 len 位数据（len 1~8，可跨页）。
 * @param x,y  起始像素坐标
 * @param data 待写入数据（低位对齐）
 * @param len  写入位数 1~8
 * @param color 颜色模式
 * @note  bit+len>8 时数据会拆成上下两个字节分别写入相邻两页。
 */
static void dev_oled_set_bits_fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, dev_oled_color_t color)
{
    uint8_t page = y / 8u;
    uint8_t bit  = y % 8u;

    if (bit + len > 8u)    /* 跨页：拆成上下两个字节 */
    {
        dev_oled_set_byte_fine(page, x, (uint8_t)(data << bit), bit, 7u, color);
        dev_oled_set_byte_fine(page + 1u, x, (uint8_t)(data >> (8u - bit)),
                               0u, (uint8_t)(len + bit - 1u - 8u), color);
    }
    else
    {
        dev_oled_set_byte_fine(page, x, (uint8_t)(data << bit), bit, (uint8_t)(bit + len - 1u), color);
    }
}

/**
 * @brief 从像素坐标 (x,y) 向下写 8 位数据（可跨页）。
 * @param x,y  起始像素坐标
 * @param data 待写入数据（低位对齐）
 * @param color 颜色模式
 * @note  本质是 set_bits_fine 的 len=8 特化版本，字模取模时最常用。
 */
static void dev_oled_set_bits(uint8_t x, uint8_t y, uint8_t data, dev_oled_color_t color)
{
    uint8_t page = y / 8u;
    uint8_t bit  = y % 8u;

    dev_oled_set_byte_fine(page, x, (uint8_t)(data << bit), bit, 7u, color);
    if (bit != 0u)
        dev_oled_set_byte_fine(page + 1u, x, (uint8_t)(data >> (8u - bit)), 0u, (uint8_t)(bit - 1u), color);
}

/**
 * @brief 从像素坐标 (x,y) 写入 w*h 的列行式点阵块（字模通用）。
 * @param x,y  起始像素坐标
 * @param data 列行式数据：先按列排列，每列自上而下按字节分组
 * @param w    点阵宽度（列数）
 * @param h    点阵高度（行数）
 * @param color 颜色模式
 * @note  高度 h 不一定是 8 的倍数，末尾不足一字节的部分用 set_bits_fine
 *        按实际位数写入，避免越界。
 */
static void dev_oled_set_block(uint8_t x, uint8_t y, const uint8_t *data,
                               uint8_t w, uint8_t h, dev_oled_color_t color)
{
    uint8_t full_row = h / 8u;    /* 完整字节的行数       */
    uint8_t part_bit = h % 8u;    /* 末尾不足一字节的位数 */

    for (uint8_t i = 0u; i < w; i++)
    {
        for (uint8_t j = 0u; j < full_row; j++)
            dev_oled_set_bits(x + i, y + j * 8u, data[i + j * w], color);
    }
    if (part_bit != 0u)
    {
        uint16_t full_num = (uint16_t)w * full_row;    /* 完整字节数 */

        for (uint8_t i = 0u; i < w; i++)
            dev_oled_set_bits_fine(x + i, y + full_row * 8u, data[full_num + i], part_bit, color);
    }
}

/* ============ 5. 文字显示 ============ */

/**
 * @brief 显示一个 ASCII 字符。
 * @param x,y  字符左上角像素坐标
 * @param ch   待显示字符
 * @param font ASCII 字模
 * @param color 颜色模式
 * @note  非可打印字符（' '~'~' 之外）按空格处理，防止字模数组越界
 *        （原实现收到控制字符会数组越界）。
 */
void dev_oled_print_ascii_char(uint8_t x, uint8_t y, char ch, const font_ascii_t *font, dev_oled_color_t color)
{
    if ((ch < ' ') || (ch > '~'))
        ch = ' ';

    dev_oled_set_block(x, y,
                       font->chars + (uint16_t)((uint8_t)(ch - ' ') * (((font->h + 7u) / 8u) * font->w)),
                       font->w, font->h, color);
}

/**
 * @brief 显示一个 ASCII 字符串。
 * @param x,y  起始像素坐标
 * @param str  以 '\0' 结尾的字符串
 * @param font ASCII 字模
 * @param color 颜色模式
 * @note  超出屏宽 256 后 x 回绕（uint8_t 自然回绕，与参考实现一致），
 *        注意控制内容长度。
 */
void dev_oled_print_ascii_string(uint8_t x, uint8_t y, const char *str, const font_ascii_t *font, dev_oled_color_t color)
{
    uint8_t x0 = x;    /* 光标位置 */

    while (*str != '\0')
    {
        dev_oled_print_ascii_char(x0, y, *str, font, color);
        x0 += font->w;
        str++;
    }
}

/**
 * @brief 获取 UTF-8 字符的字节长度（1~4），非法编码返回 0。
 * @param string 指向 UTF-8 字符首字节
 * @return 1~4 字节长度；非法编码返回 0
 * @note  仅按首字节高位模式判断，不做完整合法性校验（字符集固定为 UTF-8
 *        时够用）。
 */
static uint8_t dev_oled_get_utf8_len(const char *string)
{
    if ((string[0] & 0x80) == 0x00)
        return 1u;
    else if ((string[0] & 0xE0) == 0xC0)
        return 2u;
    else if ((string[0] & 0xF0) == 0xE0)
        return 3u;
    else if ((string[0] & 0xF8) == 0xF0)
        return 4u;
    return 0u;
}

/**
 * @brief 显示字符串（UTF-8，可中英混排，需配套字库）。
 * @param x,y  起始像素坐标
 * @param str  UTF-8 字符串
 * @param font 中英混排字库（font_t）
 * @param color 颜色模式
 * @note  中文字模查不到时显示空格；ASCII 字符回退到 font->ascii 显示。
 * @note  为保证中文正常识别，请将编译器字符集设置为 UTF-8，并使用波特律动
 *        取模工具生成字库（https://led.baud-dance.com）。
 * @note  逐字符在字库中线性查找（TODO：可优化为二分/哈希，当前字符数少够用）。
 * @note  非法 UTF-8 序列直接停止显示，防止死循环。
 */
void dev_oled_print_string(uint8_t x, uint8_t y, const char *str, const font_t *font, dev_oled_color_t color)
{
    uint16_t i = 0;    /* 字符串索引 */
    uint8_t  one_len = (uint8_t)((((font->h + 7u) / 8u) * font->w) + 4u);    /* 单个字模占字节数 */

    if (font->ascii == NULL)    /* 缺省 ASCII 字体缺失时无法回退显示 */
        return;

    while (str[i] != '\0')
    {
        uint8_t found = 0;
        uint8_t utf8_len = dev_oled_get_utf8_len(&str[i]);

        if (utf8_len == 0u)
            break;    /* 非法 UTF-8 序列，停止显示 */

        /* 在字库中线性查找字符（TODO：可优化为二分/哈希） */
        for (uint8_t j = 0u; j < font->len; j++)
        {
            const uint8_t *head = font->chars + (uint32_t)j * one_len;

            if (memcmp(&str[i], head, utf8_len) == 0)
            {
                dev_oled_set_block(x, y, head + 4u, font->w, font->h, color);
                x += font->w;    /* 光标右移一个字宽 */
                i += utf8_len;
                found = 1u;
                break;
            }
        }

        /* 未找到字模：ASCII 字符回退显示，非 ASCII 字符显示空格 */
        if (found == 0u)
        {
            dev_oled_print_ascii_char(x, y, (utf8_len == 1u) ? str[i] : ' ', font->ascii, color);
            x += font->ascii->w;
            i += utf8_len;
        }
    }
}