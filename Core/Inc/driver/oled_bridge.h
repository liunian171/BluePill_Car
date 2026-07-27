/**
 * @file    oled_bridge.h
 * @brief   OLED C 桥接层 — main.c 可调用的 C 接口
 */

#ifndef __OLED_BRIDGE_H__
#define __OLED_BRIDGE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void oled_bridge_init(void);
void oled_bridge_show_string(uint8_t row, uint8_t col, const char *str);
void oled_bridge_show_chinese(uint8_t row, uint8_t col, uint8_t index);
void oled_bridge_show_string_small(uint8_t page, uint8_t col, const char *str);

#ifdef __cplusplus
}
#endif

#endif
