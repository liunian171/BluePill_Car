/**
 * @file    oled_bridge.cpp
 * @brief   OLED C 桥接实现
 */

#include "oled_bridge.h"
#include "oled_driver.h"

static OledDriver g_oled;

void oled_bridge_init(void)          { g_oled.init(); }

void oled_bridge_show_string(uint8_t row, uint8_t col, const char *str)
{
    g_oled.show_string(row, col, str);
}

void oled_bridge_show_chinese(uint8_t row, uint8_t col, uint8_t index)
{
    g_oled.show_chinese(row, col, index);
}

void oled_bridge_show_string_small(uint8_t page, uint8_t col, const char *str)
{
    g_oled.show_string_small(page, col, str);
}
