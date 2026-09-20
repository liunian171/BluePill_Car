/**
 * ============================================================================
 *  app_display.h — 100ms 显示节拍宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（健康数据经 io 注入, 保 PC 桩）。
 *  职责: OLED 8 页单页轮转组版（定宽整行写, 禁裸清屏）+ 熔断呈现。
 *  换平台: 不动; 换屏: 显示链下层本来就是 oled_bridge 注入, 本模块无感。
 *
 *  吞掉的 main.c 现块: 主循环 ③ 每 100ms 显示块（L1150-1268）。
 *  组件数据（IMU/编码器/PID 增益）经组件层 getter 直读 —— 合法下行（感官呈现）。
 * ============================================================================
 */

#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <stdint.h>
#include "driver/bridge_ret.h"
#include "app/app_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置 ---- */
typedef struct {
    uint32_t disp_period_ms;      /* 页刷新节拍 (CAR_DISP_PERIOD_MS, 现 100) */
    uint8_t  usb_enabled;         /* P3: USB 链路下线时页 7 不宣称在线 (恒 OFF) */
} app_display_cfg_t;

/* ---- 依赖注入（健康数据源; 组件数据不注入, 直读组件 getter） ---- */
typedef struct {
    const char *(*owner_str)(void *ctx);   /* "USB"/"UART" (app_tx) */
    uint8_t  (*usb_on)(void *ctx);         /* USB 链路在线 (app_tx) */
    uint16_t (*rx_overflow)(void *ctx);    /* ringbuf 溢出和 (app_link) */
    uint32_t (*uart_silence_s)(void *ctx); /* 距最近 UART 字节秒数 (app_link) */
    int32_t  (*enc_count)(uint8_t id, void *ctx); /* 编码器读 (id 0/1; 页2) */
    uint8_t  (*gray_read)(uint8_t idx, void *ctx); /* 5 路灰度 idx 0~4 (页3; P2-4 收敛点) */
    void *ctx;
} app_display_io_t;

/* ---- 生命周期 ---- */
bridge_ret_t app_display_init(const app_display_cfg_t *cfg, const app_display_io_t *io);

/* 节拍入口: 每 disp_period_ms 刷 1 页, 8 页一轮。
 * OLED 写失败熔断逻辑随 oled_driver 原样生效（页7 异常态 E/O/FZ）。 */
void app_display_task(uint32_t now);

/* ---- 喂数（仅供 app_wiring 填进 app_page_note_t, 业务勿直调） ---- */
void app_display_note_cmd(const char *s, void *ctx);   /* → OLED 页6 */
void app_display_note_resp(const char *s, void *ctx);  /* → OLED 页4 */

#ifdef __cplusplus
}
#endif

#endif /* APP_DISPLAY_H */
