/**
 * @file    oled_bridge.cpp
 * @brief   OLED C 桥接实现（家族单例特例，见 oled_bridge.h）
 */

#include "oled_bridge.h"
#include "oled_driver.h"
#include <stddef.h>

static OledDriver g_oled;

bridge_ret_t oled_bridge_init(const oled_cfg_t *cfg)
{
    if (cfg == NULL)        return BRIDGE_ERR_BAD_ARG;
    if (cfg->write == NULL) return BRIDGE_ERR_BAD_ARG;   /* 平台未绑定：显式失败 */

    if (g_oled.init(cfg->i2c_context, cfg->write, cfg->addr7) != 0)
        return BRIDGE_ERR_IO;

    return BRIDGE_OK;
}

/* [Flash 瘦身 2026-09-19] oled_bridge_show_string(8×16) / oled_bridge_show_chinese
 * 已删除：无调用者 + 携带 1.5KB 字库（详 oled_driver.cpp 同批说明与代码风格指南 §7） */

void oled_bridge_show_string_small(uint8_t page, uint8_t col, const char *str)
{
    g_oled.show_string_small(page, col, str);
}

void oled_bridge_show_line_small(uint8_t page, const char *str)
{
    g_oled.show_line_small(page, str);
}

/* ---- 诊断接口（2026-09-19 节拍拉长专案补：失败必须可见）---- */

uint16_t oled_bridge_tx_fail_count(void)
{
    return g_oled.tx_fail_count();
}

uint8_t oled_bridge_fused(void)
{
    return g_oled.fused();
}
