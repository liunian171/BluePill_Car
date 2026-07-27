/**
 * @file    oled_driver.h
 * @brief   SSD1315 OLED 驱动 — 128x64，8×16 ASCII + 16×16 汉字
 *
 * 8×16: 每行 16 字符，共 4 行
 *     行 0: page 0(上) + page 1(下)
 *     行 1: page 2(上) + page 3(下)
 *     行 2: page 4(上) + page 5(下)
 *     行 3: page 6(上) + page 7(下)
 *
 * 汉字 16×16: 占用两行，显示在 (row, row+1)
 */

#ifndef __OLED_DRIVER_H__
#define __OLED_DRIVER_H__

#include <stdint.h>

#ifdef __cplusplus

class OledDriver {
public:
    OledDriver();

    /** @brief 初始化显示 */
    int8_t init();

    /** @brief 清屏 */
    void clear();

    /**
     * @brief 显示 ASCII 字符串
     * @param row  0~3 文本行
     * @param col  0~15 字符列
     */
    void show_string(uint8_t row, uint8_t col, const char *str);

    /**
     * @brief 显示 16×16 汉字
     * @param row      汉字上端所在行 (0~2)
     * @param col      字符列 (0~7，汉字占 2 个 ASCII 字符宽度)
     * @param index    字库索引 (CN_INDEX_LIU / CN_INDEX_NIAN)
     */
    void show_chinese(uint8_t row, uint8_t col, uint8_t index);

    /**
     * @brief 显示 6×8 小字字符串
     * @param page 0~7 页 (每页 8 像素高)
     * @param col  0~20 字符列 (128/6=21列)
     */
    void show_string_small(uint8_t page, uint8_t col, const char *str);

private:
    void write_cmd(uint8_t cmd);
    void write_cmd_multi(const uint8_t *cmds, uint16_t len);
    void write_data(uint8_t data);
    void set_pos(uint8_t page, uint8_t col_byte);
};

#endif /* __cplusplus */

#endif
