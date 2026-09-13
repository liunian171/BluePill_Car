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
 *
 * ▸ 平台依赖（C1，2026-09-13）◂
 *   本文件与本实现**零 HAL / 零 CubeMX 依赖**：I2C 以"写事务函数"注入
 *   （`oled_write_fn`，见 oled_platform_ops.h）；HAL 绑定在平台侧文件。
 *   原实现硬编码 `&hi2c2` 并把 `i2c.h` 传染进器件层，已去除。
 *
 * ▸ 家族契约 ◂ 桥接层 `oled_bridge` 是本驱动的 C 门面（单例，见
 *   doc/桥接层家族契约.md §2.7）。
 */

#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include <stdint.h>
#include "oled_platform_ops.h"   /* oled_write_fn */

#ifdef __cplusplus

class OledDriver {
public:
    OledDriver();

    /**
     * @brief 绑定平台并初始化显示
     * @param i2c_context 平台资源（STM32 为 I2C_HandleTypeDef*）
     * @param write       写事务函数（不得为 NULL）
     * @param addr7       从机 7 位地址（本器件 0x3C）
     * @retval 0 成功 / -1 参数非法
     */
    int8_t init(void *i2c_context, oled_write_fn write, uint8_t addr7);

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

    /**
     * @brief 整行写入 6×8 小字 (单次 I2C 事务, 高速刷新用)
     * @param page 0~7 页
     * @param str  最长 21 字符, 不足/超出自动截断补齐, 从列 0 开始
     */
    void show_line_small(uint8_t page, const char *str);

private:
    void *        ctx_   = 0;   /* 平台资源 */
    oled_write_fn write_ = 0;   /* 写事务注入 */
    uint8_t       addr_  = 0;   /* 从机 7 位地址 */

    void write(uint8_t reg, const uint8_t *data, uint16_t len);
    void write_cmd(uint8_t cmd);
    void write_cmd_multi(const uint8_t *cmds, uint16_t len);
    void write_data(uint8_t data);
    void set_pos(uint8_t page, uint8_t col_byte);
};

#endif /* __cplusplus */

#endif /* OLED_DRIVER_H */
