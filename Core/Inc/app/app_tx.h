/**
 * ============================================================================
 *  app_tx.h — 发送出口与路由（组装层, HAL 直调唯一收口）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。⚠️ 本模块是组装层 HAL 直调的三个合法点之一
 *        （另两处: app_wiring 判活绑定 / 装载层 ISR 壳）—— 其余 app_* 零 HAL。
 *  职责: USB/UART 两个原始发送出口 + 应答按来源回 + 广播按 owner 回 + P3 链路开关。
 *  换平台: 替换本模块内的原始出口实现（每芯片一套, 语义不变）。
 *
 *  吞掉的 main.c 现块: usb_send_raw/uart_send_raw/link_tx/link_owner_tx/
 *  owner_main_link（L301-368）。链路禁用 = 出口 no-op（P3 "接线不存在"语义不变）。
 * ============================================================================
 */

#ifndef APP_TX_H
#define APP_TX_H

#include <stdint.h>
#include "driver/bridge_ret.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 链路编号（main 消费口径: 0=UART / 1=USB, 与仲裁枚举换算在此收口） ---- */
#define APP_LINK_UART   0
#define APP_LINK_USB    1
#define APP_LINK_COUNT  2

/* ---- 配置（值由 app_wiring 从 car_config.h 翻译注入） ---- */
typedef struct {
    uint8_t uart_enabled;         /* 0 → uart 出口 no-op, 仲裁 fallback 用 */
    uint8_t usb_enabled;          /* 0 → usb 出口 no-op */
    uint32_t usb_busy_timeout_ms; /* USB USBD_BUSY 限时 (现 10ms, 逾期静默弃) */
    uint32_t uart_block_timeout_ms; /* UART 阻塞发送超时 (现 100ms) */
} app_tx_cfg_t;

/* ---- 生命周期（io 表无: 本模块直接持有 HAL 出口; link_arbiter 为组件层合法直调） ---- */
bridge_ret_t app_tx_init(const app_tx_cfg_t *cfg);

/* ---- 发送（app_link 经 app_tx_sink_t 调到这里的实现） ---- */
void app_tx_send_by_link(uint8_t link, const char *buf, int n);  /* 应答: 谁的命令回谁 */
void app_tx_send_to_owner(const char *buf, int n);               /* 仲裁广播: 到达指挥权方 */

/* ---- 状态（供 app_link 门控与 app_display 页7; 未 init 返回安全默认值） ----
 * 2026-09-20 订正: 页7 "U:<秒>" 实为"距最近一次 UART 字节的秒数"(归 app_link,
 * 由其 rx 活动数据得出), app_tx 只负责 owner 字符串与 USB 在线状态。 */
uint8_t     app_tx_owner_main_link(void);  /* 指挥权(已换算到 APP_LINK_* 口径) */
void        app_tx_set_active(uint8_t link);       /* 命令来源链路登记 */
const char *app_tx_owner_str(void);                /* "USB"/"UART" */
uint8_t     app_tx_usb_on(void);                   /* USB 在线(dev_state+pClassData, 直读) */

#ifdef __cplusplus
}
#endif

#endif /* APP_TX_H */
