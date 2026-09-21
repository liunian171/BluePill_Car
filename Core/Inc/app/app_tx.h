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
 *  ▸ R3 (2026-09-21) 发送路径专项 3a+3b ◂
 *    两个原始出口从"同步阻塞"改为"统一 tx_stream 队列 + 分段异步发送"（详见
 *    doc/发送路径专项设计.md）：
 *      · send_by_link / send_to_owner 返回 bridge_ret_t（统一失败语义）。
 *      · 失败/丢弃在统计计数（app_tx_get_stats），OLED 页 7 可见（不沉默失败）。
 *      · UART 走 HAL_UART_Transmit_IT（非阻塞）；USB 走 CDC_Transmit_FS（非阻塞，
 *        忙留队续发, 不落地丢弃）。主循环调 app_tx_service() 推进队列。
 *    平台两原语（start/free）在本 .c 内实现（HAL 直调收口不变）。
 *
 *  吞掉的 main.c 现块: usb_send_raw/uart_send_raw/link_tx/link_owner_tx/
 *  owner_main_link（L301-368）。链路禁用 = 出口 no-op（P3 "接线不存在"语义不变）。
 * ============================================================================
 */

#ifndef APP_TX_H
#define APP_TX_H

#include <stdint.h>
#include "driver/bridge_ret.h"
#include "app/app_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（值由 app_wiring 从 car_config.h 翻译注入） ----
 * R3: 超时字段已删除（不再忙轮询/阻塞超时；改由 tx_stream 队列 + free() 门控）。 */
typedef struct {
    uint8_t uart_enabled;         /* 0 → uart 出口 no-op, 仲裁 fallback 用 */
    uint8_t usb_enabled;          /* 0 → usb 出口 no-op */
} app_tx_cfg_t;

/* ---- 发送统计（统一失败语义 + 沉默失败可见） ---- */
typedef struct {
    uint16_t drop;    /* 入队丢弃字节（队满, 累计） */
    uint16_t txerr;   /* 平台发送 start 失败计数 */
    uint32_t sent;    /* 已启动发送字节累计 */
    uint16_t pending; /* 队内未发字节 */
} app_tx_stats_t;

/* ---- 生命周期（io 表无: 本模块直接持有 HAL 出口; link_arbiter 为组件层合法直调） ---- */
bridge_ret_t app_tx_init(const app_tx_cfg_t *cfg);

/* ---- 发送（app_link 经 app_tx_sink_t 调到这里的实现） ----
 * 返回 BRIDGE_OK（尽力而为入队；失败/丢弃计入 stats, 不阻塞）。
 * buf 不必 NUL 结尾（按 n 字节发）。 */
bridge_ret_t app_tx_send_by_link(uint8_t link, const char *buf, int n);  /* 应答: 谁的命令回谁 */
bridge_ret_t app_tx_send_to_owner(const char *buf, int n);               /* 仲裁广播: 到达指挥权方 */

/* ---- 队列推进：主循环每轮调一次（R3 新出口；负责 UART IT / USB 段的续推） ---- */
void app_tx_service(void);

/* ---- 统计 / 状态（供 app_display 页7 与 SDK 查询; 未 init 返回 0） ---- */
void app_tx_get_stats(uint8_t link, app_tx_stats_t *st);   /* link=APP_LINK_* */
uint8_t     app_tx_owner_main_link(void);  /* 指挥权(已换算到 APP_LINK_* 口径) */
void        app_tx_set_active(uint8_t link);       /* 命令来源链路登记 */
uint8_t     app_tx_active(void);                   /* 最近命令来源链路 (应答路由读) */
const char *app_tx_owner_str(void);                /* "USB"/"UART" */
uint8_t     app_tx_usb_on(void);                   /* USB 在线(dev_state+pClassData, 直读) */

#ifdef __cplusplus
}
#endif

#endif /* APP_TX_H */
