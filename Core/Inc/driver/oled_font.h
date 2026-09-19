/**
 * @file    oled_font.h
 * @brief   字库声明 — 6×8 ASCII（本车唯一在用的字库）
 *
 *  [Flash 瘦身 2026-09-19] 原 8×16 ASCII 字库（`OLED_F8x16`）与 16×16 汉字库
 *  （`font_cn_16x16` / `CN_INDEX_*`）已随 `Core/Src/driver/oled_font_data.c` 一并删除：
 *  其唯一消费者是已删的 show_string/show_chinese，而字库本身常驻 ≈1.6KB Flash。
 *  需要大字号或汉字时，从 git 历史取回该 .c 与对应接口，并重新 cmake configure。
 */

#ifndef __OLED_FONT_H__
#define __OLED_FONT_H__

#include <stdint.h>

/* ---- ASCII 6×8 小字库 (95字符, 每个6字节) ---- */
extern const uint8_t OLED_F6x8[][6];

#endif
