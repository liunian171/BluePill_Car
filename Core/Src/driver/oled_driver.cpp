/**
 * @file    oled_driver.cpp
 * @brief   SSD1315 OLED 驱动实现 — 8×16 ASCII + 16×16 汉字
 *
 * @note I2C 适配说明（C1，2026-09-13）
 * ───────────────────────────────────────────────────────────
 * 本文件**零 HAL / 零 CubeMX 依赖**：I2C 写事务由构造注入的
 * `oled_write_fn` 完成，HAL 绑定在 `oled_platform_ops.c`。
 *
 * 演进：F427 原工程用软件 I2C（useri2c ops 表）→ 移植时改为在本文件
 *       直接调 HAL_I2C_Mem_Write + 硬编码 &hi2c2（器件层拖 HAL，且无法
 *       换 I2C/多屏）→ 本次改为写事务注入，方向与 useri2c ops 表一致，
 *       但只暴露 OLED 真正需要的那一个操作。
 * ───────────────────────────────────────────────────────────
 */

#include "oled_driver.h"
#include "oled_font.h"

/* 写事务：显示链按"尽力而为"处理，忽略返回码（错误不刷屏、不打断控制链） */
void OledDriver::write(uint8_t reg, const uint8_t *data, uint16_t len) {
    if (write_ == 0) return;
    (void)write_(ctx_, addr_, reg, data, len);
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

int8_t OledDriver::init(void *i2c_context, oled_write_fn write_fn, uint8_t addr7) {
    if (write_fn == 0) return -1;      /* 平台未绑定：显式失败，不静默出空屏 */
    ctx_   = i2c_context;
    write_ = write_fn;
    addr_  = addr7;

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

/* ==========================================================================
 *  整行写入 — 128 字节单次 I2C 事务
 *
 *  相比逐字符写 (每字符 2 次事务), 整行仅 2 次事务 (定位 + 128B 数据),
 *  全屏 8 行刷新从 ~550 次事务降到 16 次, I2C 流量降约 10 倍。
 * ========================================================================== */
void OledDriver::show_line_small(uint8_t page, const char *str) {
    if (page > 7 || !str) return;

    uint8_t buf[128];  /* 21 字符 × 6 字节 = 126, 补齐到 128 */

    for (uint8_t ch = 0; ch < 21; ch++) {
        char c = str[ch] ? str[ch] : ' ';
        uint8_t idx = (uint8_t)c - 0x20;
        if (idx > 94) idx = 0;
        for (uint8_t k = 0; k < 6; k++)
            buf[ch * 6 + k] = OLED_F6x8[idx][k];
    }
    for (uint8_t i = 126; i < 128; i++) buf[i] = 0x00;

    set_pos(page, 0);
    write(0x40, buf, 128);
}
