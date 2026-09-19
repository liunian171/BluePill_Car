#ifndef OLED_BRIDGE_H
#define OLED_BRIDGE_H

/**
 * @file    oled_bridge.h
 * @brief   桥接层 — oled_bridge（OLED C 门面，**家族单例特例**）
 *
 *  ▸ 家族契约（正文 doc/桥接层家族契约.md，返回码 bridge_ret.h）◂
 *    · init 用配置结构注入 + 返回 bridge_ret_t
 *    · **单例豁免**：运行时接口不带 id —— 单屏是硬件事实，双屏是投机需求（YAGNI）。
 *      触发归一的条件：出现第二块屏 / 第二路 I2C 时，给 cfg 加 id 并给运行时接口补 id。
 *    · 平台通过 `oled_write_fn` 注入（器件层零 HAL），组装层传
 *      `oled_platform_write_stm32`（见 oled_platform_ops.h）
 *
 *  ▸ 显示链错误策略 ▸ 按"尽力而为"：写失败不刷屏、不打断控制链
 *    （显示是辅助通道，不应因 I2C 异常阻塞 50ms 控制环——2026-09-13 实测踩坑）
 *    **但失败必须可见**（2026-09-19 节拍拉长专案补）：显示链曾因"沉默重试"把
 *    主循环拖到 100ms（OLED 排线接触不良 × 平台超时）而无人察觉 → 现补两项：
 *    ① 器件层连续失败熔断（`oled_driver.cpp`，失败代价有硬上界）
 *    ② 失败计数经本桥暴露（`oled_bridge_tx_fail_count`），组装层显示于 OLED 页 7
 * ============================================================================
 */

#include <stdint.h>
#include "bridge_ret.h"
#include "oled_platform_ops.h"   /* oled_write_fn */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（组装层注入）---- */
typedef struct {
    void *        i2c_context;   /* 平台资源（STM32 为 I2C_HandleTypeDef*） */
    oled_write_fn write;         /* 必填（NULL → BAD_ARG） */
    uint8_t       addr7;         /* 从机 7 位地址（本器件 0x3C） */
} oled_cfg_t;

/**
 * @brief  初始化显示（绑定平台 + SSD1315 初始化序列 + 清屏）
 * @retval BRIDGE_OK / BAD_ARG（cfg/write 为 NULL）/ BRIDGE_ERR_IO（器件 init 失败）
 */
bridge_ret_t oled_bridge_init(const oled_cfg_t *cfg);

/* ---- 显示接口（单例，无 id）----
 * [Flash 瘦身 2026-09-19] 原 `oled_bridge_show_string`(8×16) / `oled_bridge_show_chinese`
 * 已删除（无调用者 + 携带 1.5KB 字库）；本车显示统一走下方 6×8 两条路径。 */
void oled_bridge_show_string_small(uint8_t page, uint8_t col, const char *str);
void oled_bridge_show_line_small(uint8_t page, const char *str);   /* 整行单事务写入 */

/* ---- 诊断接口（取值类：无副作用，可随时调用）---- */
/** @brief 累计写事务失败次数（0 = 从未失败；非 0 = 屏/排线有问题，见专案文档） */
uint16_t oled_bridge_tx_fail_count(void);

/** @brief 熔断状态：1 = 处于停刷冷却期（连续失败后主动静默，约 3s 后自动重试） */
uint8_t  oled_bridge_fused(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_BRIDGE_H */
