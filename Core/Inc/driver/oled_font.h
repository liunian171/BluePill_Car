/**
 * @file    oled_font.h
 * @brief   字库声明 — 6×8 + 8×16 ASCII + 16×16 汉字
 */

#ifndef __OLED_FONT_H__
#define __OLED_FONT_H__

#include <stdint.h>

/* ---- ASCII 6×8 小字库 (95字符, 每个6字节) ---- */
extern const uint8_t OLED_F6x8[][6];

/* ---- ASCII 8×16 大字库 (95字符, 每个16字节) ---- */
extern const uint8_t OLED_F8x16[][16];

/* ---- 汉字 16×16 字库 — 流年 ---- */
#define CN_INDEX_LIU  0
#define CN_INDEX_NIAN 1
extern const uint8_t font_cn_16x16[2][32];

#endif
