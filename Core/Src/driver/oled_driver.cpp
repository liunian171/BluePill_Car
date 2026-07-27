/**
 * @file    oled_driver.cpp
 * @brief   SSD1315 OLED 驱动实现 — 8×16 ASCII + 16×16 汉字
 *
 * @note I2C 适配说明
 * ───────────────────────────────────────────────────────────
 * 原工程 (F427) 使用软件 I2C（useri2c.h 的 I2C_Handle + ops 表）。
 * 移植到 Bluepill 时改为直接调用 HAL_I2C_Mem_Write（硬件 I2C2）。
 *
 * TODO: 等 i2c_hardware_ops.c 实现后，改回 useri2c.h 接口，
 *       保持 I2C 驱动框架的 ops 抽象一致性。
 * ───────────────────────────────────────────────────────────
 */

#include "oled_driver.h"
#include "oled_font.h"
#include "i2c.h"            /* hi2c2 */

static const uint8_t kOledAddr = 0x3C << 1;  /* HAL 需要 7bit 左移 1 位 */

static void write(uint8_t reg, const uint8_t *data, uint16_t len) {
    HAL_I2C_Mem_Write(&hi2c2, kOledAddr, reg, I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)data, len, 100);
}

/* ========================================================================== */

void OledDriver::write_cmd(uint8_t cmd)        { write(0x00, &cmd, 1); }
void OledDriver::write_cmd_multi(const uint8_t *cmds, uint16_t len) { write(0x00, cmds, len); }
void OledDriver::write_data(uint8_t data)       { write(0x40, &data, 1); }

void OledDriver::set_pos(uint8_t page, uint8_t col_byte) {
    uint8_t cmd[3] = {
        (uint8_t)(0xB0 + page),
        (uint8_t)(col_byte & 0x0F),
        (uint8_t)(0x10 + (col_byte >> 4)),
    };
    write_cmd_multi(cmd, 3);
}

/* ========================================================================== */

OledDriver::OledDriver() {}

int8_t OledDriver::init() {
    const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB,
        0x40, 0xA4, 0xA6, 0xAF,
    };
    write_cmd_multi(cmds, sizeof(cmds));
    clear();
    return 0;
}

void OledDriver::clear() {
    for (uint8_t pg = 0; pg < 8; pg++) {
        set_pos(pg, 0);
        for (uint16_t i = 0; i < 128; i++) write_data(0x00);
    }
}

/* ==========================================================================
 *  ASCII 8×16 字符显示
 *
 *  每个字符 16 字节:
 *     [0..7]  = 上半部分 (8 列)
 *     [8..15] = 下半部分 (8 列)
 *
 *  row(0~3) 对应:
 *     上 page = row * 2
 *     下 page = row * 2 + 1
 * ========================================================================== */

void OledDriver::show_string(uint8_t row, uint8_t col, const char *str) {
    if (row > 3 || !str) return;

    while (*str && col < 16) {
        uint8_t idx = (uint8_t)(*str) - 0x20;
        if (idx > 94) { str++; continue; }

        uint8_t byte_col = col * 8;   // 每个字符 8 列像素

        /* ---- 上半（row × 2）---- */
        set_pos(row * 2, byte_col);
        write(0x40, OLED_F8x16[idx], 8);           // 前 8 字节

        /* ---- 下半（row × 2 + 1）---- */
        set_pos(row * 2 + 1, byte_col);
        write(0x40, OLED_F8x16[idx] + 8, 8);       // 后 8 字节

        col++;
        str++;
    }
}

/* ==========================================================================
 *  16×16 汉字显示 — 占用两行 (上+下)
 * ========================================================================== */

void OledDriver::show_chinese(uint8_t row, uint8_t col, uint8_t index) {
    if (row > 2 || col > 14) return;  // col 在 ASCII 8 像素单位下
    if (index > 1) return;

    uint8_t byte_col = col * 8;  // 汉字宽 16 像素 = 2 个 ASCII 列

    /* 上半：row */
    set_pos(row * 2, byte_col);
    write(0x40, font_cn_16x16[index], 16);         // 前 16 字节

    /* 下半：row + 1 */
    set_pos(row * 2 + 1, byte_col);
    write(0x40, font_cn_16x16[index] + 16, 16);    // 后 16 字节
}

/* ==========================================================================
 *  6×8 小字显示 — 每行 21 个字符
 *
 *  page(0~7) 对应 Y 坐标的第几页（8 像素一组）
 *  col(0~20) 代表第几个 6 像素宽的字符列
 * ========================================================================== */
void OledDriver::show_string_small(uint8_t page, uint8_t col, const char *str) {
    if (page > 7 || !str) return;

    while (*str && col < 21) {
        uint8_t idx = (uint8_t)(*str) - 0x20;
        if (idx > 94) { str++; continue; }

        uint8_t byte_col = col * 6;  /* 每字符 6 列 */

        set_pos(page, byte_col);
        write(0x40, OLED_F6x8[idx], 6);

        col++;
        str++;
    }
}
