/**
 * ============================================================================
 *  app_tx.c — 发送出口与路由（组装层, HAL 直调唯一收口）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主（本模块是组装层 HAL 直调的三个合法点之一）。
 *  搬迁自: main.c usb_send_raw / uart_send_raw / link_tx / link_owner_tx /
 *          owner_main_link（P1 双链路 + P2 仲裁 + P3 开关收口成果，语义不变）。
 *  换平台: 只替换本文件内的两个平台原语实现（start/free）。
 *
 *  ▸ R3 (2026-09-21) 发送路径专项 3a+3b ◂  —— 权威见 doc/发送路径专项设计.md
 *    旧实现 = 两个同步阻塞出口（USB 忙轮询 ≤10ms 静默弃 / UART 阻塞 ≤100ms）。
 *    新实现 = 两条链各持一个"tx_stream 发送流核心"（纯逻辑, common/tx_stream）：
 *      · send_by_link/send_to_owner → 入队（非阻塞）+ 返回 bridge_ret_t。
 *      · 平台原语 start(): UART=HAL_UART_Transmit_IT, USB=CDC_Transmit_FS。
 *      · 平台原语 free(): 判链路可收新段（UART=gState==READY, USB=CDC_IsTxFree）。
 *      · app_tx_service(): 主循环每轮推进两流队列（UART IT 自限速 / USB 忙留队续发）。
 *      · 失败/丢弃 → tx_stream 统计（dropped/txerr），OLED 页7 可见（不沉默失败）。
 *
 *  ▸ P3 链路开关语义 ◂
 *    uart_enabled / usb_enabled 来自 car_config.h（组装层 init 注入）。
 *    =0 的出口为运行时 no-op —— 与 CMake 的 stub 空壳替换、组装层消费门控
 *    共同构成"两层都关才算链路下线"。
 * ============================================================================
 */

#include "app/app_tx.h"
#include "driver/link_arbiter.h"
#include "common/tx_stream.h"
#include "usart.h"              /* huart2（CubeMX 生成） */
#include "usb_device.h"
#include "usbd_cdc_if.h"        /* CDC_Transmit_FS / CDC_IsTxFree */
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;   /* 定义在 usb_device.c */

static app_tx_cfg_t g_cfg;
static uint8_t      g_active_link = APP_LINK_USB;   /* 应答路由: 最近命令来源 */
static uint8_t      g_inited = 0;

/* 两条链的发送流（纯逻辑核心） */
static tx_stream_t g_tx[APP_LINK_COUNT];           /* 0=UART 1=USB (APP_LINK_*) */

/* USB 发送段落脚缓冲：CDC 栈在 IN 传输完成前持续读它（TxState!=0），
 * 仅当 CDC_IsTxFree() 为真才覆写（free() 门控保证不踩进行中传输）。 */
static uint8_t s_usb_buf[TX_SEG_MAX];

/* ---- 平台原语（HAL 直调收口） ---- */

static int tx_uart_start(const uint8_t *seg, uint16_t len, void *ctx)
{
    (void)ctx;
    /* seg 指向 tx_stream 的 seg（本 .c 内静态持有；free()=gState READY 前不复用）。
     * HAL_UART_Transmit_IT 经 ISR 从 seg 持续发送，段须在整次传输内有效 ——
     * free() 门控保证了下次覆写要等 gState 回到 READY。 */
    return (HAL_UART_Transmit_IT(&huart2, (uint8_t *)seg, len) == HAL_OK)
           ? TX_START_OK : TX_START_ERR;
}

static int tx_uart_free(void *ctx)
{
    (void)ctx;
    if (!g_cfg.uart_enabled) return 0;              /* [P3] 链路下线 no-op */
    return (huart2.gState == HAL_UART_STATE_READY) ? 1 : 0;
}

static int tx_usb_start(const uint8_t *seg, uint16_t len, void *ctx)
{
    (void)ctx;
    if (!g_cfg.usb_enabled) return TX_START_RETRY;  /* [P3] 链路下线：留队待开 */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
        hUsbDeviceFS.pClassData == NULL) return TX_START_RETRY; /* 未枚举 */
    if (!CDC_IsTxFree()) return TX_START_RETRY;     /* IN 忙：留队续发，不落地丢弃 */
    memcpy(s_usb_buf, seg, len);
    return (CDC_Transmit_FS(s_usb_buf, len) == USBD_OK) ? TX_START_OK : TX_START_RETRY;
}

static int tx_usb_free(void *ctx)
{
    (void)ctx;
    if (!g_cfg.usb_enabled) return 0;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
        hUsbDeviceFS.pClassData == NULL) return 0;
    return CDC_IsTxFree() ? 1 : 0;
}

/* uuid: 0=UART 1=USB */
static tx_stream_t *stream_of(uint8_t link)
{
    return (link == APP_LINK_USB) ? &g_tx[APP_LINK_USB] : &g_tx[APP_LINK_UART];
}

/* ---- 公开接口 ---- */

bridge_ret_t app_tx_init(const app_tx_cfg_t *cfg)
{
    if (cfg == NULL) return BRIDGE_ERR_BAD_ARG;
    if (!cfg->uart_enabled && !cfg->usb_enabled) return BRIDGE_ERR_BAD_CFG; /* 双 0 无意义 */
    g_cfg = *cfg;
    g_active_link = cfg->usb_enabled ? APP_LINK_USB : APP_LINK_UART;

    tx_stream_init(&g_tx[APP_LINK_UART]);
    tx_stream_init(&g_tx[APP_LINK_USB]);
    tx_stream_bind(&g_tx[APP_LINK_UART],
                   &(tx_stream_io_t){ tx_uart_start, tx_uart_free, NULL });
    tx_stream_bind(&g_tx[APP_LINK_USB],
                   &(tx_stream_io_t){ tx_usb_start, tx_usb_free, NULL });
    g_inited = 1;
    return BRIDGE_OK;
}

bridge_ret_t app_tx_send_by_link(uint8_t link, const char *buf, int n)
{
    if (!g_inited || buf == NULL || n <= 0) return BRIDGE_ERR_BAD_ARG;
    tx_stream_t *s = stream_of(link);
    if (s == NULL) return BRIDGE_ERR_BAD_ARG;
    /* 链路下线时仍入队（tx_stream 核心用 start/free 的 no-op 门控自然滞留），
     * 而非上一版的"出口 no-op 静默丢弃" —— 下线=不可发，队列滞留由 core 统计。 */
    (void)tx_stream_send(s, (const uint8_t *)buf, n);
    return BRIDGE_OK;
}

bridge_ret_t app_tx_send_to_owner(const char *buf, int n)
{
    /* 仲裁广播必须到达指挥权方 —— 与应答路由(send_by_link)不同 */
    return app_tx_send_by_link(app_tx_owner_main_link(), buf, n);
}

void app_tx_service(void)
{
    if (!g_inited) return;
    /* 两流各自 while-drain：UART 每次最多推进一段（IT 忙即停），USB 忙时停顿。
     * 主循环每轮调用即"时间片摊销"，重复 n 次只费 O(段数)。 */
    while (tx_stream_service(&g_tx[APP_LINK_UART])) {}
    while (tx_stream_service(&g_tx[APP_LINK_USB])) {}
}

void app_tx_get_stats(uint8_t link, app_tx_stats_t *st)
{
    if (st == NULL) return;
    tx_stream_t *s = stream_of(link);
    if (!g_inited || s == NULL || link >= APP_LINK_COUNT) {
        memset(st, 0, sizeof(*st));
        return;
    }
    st->drop    = tx_stream_dropped(s);
    st->txerr   = tx_stream_txerr(s);
    st->sent    = tx_stream_sent(s);
    st->pending = tx_stream_pending(s);
}

void app_tx_set_active(uint8_t link)
{
    g_active_link = (link == APP_LINK_USB) ? APP_LINK_USB : APP_LINK_UART;
}

uint8_t app_tx_active(void)
{
    return g_active_link;
}

uint8_t app_tx_owner_main_link(void)
{
    /* [P3 编号换算] main 消费循环 0=UART/1=USB, 仲裁枚举 0=USB/1=UART —
     * 两套编号相反, 换算在此唯一收口（2026-09-19 真机 bug 教训）。
     * 单链路模式直接返回唯一存活链路, 不依赖仲裁状态机（cfg 为编译期常量,
     * 分支可被优化; 门控本体仍在 CMake stub 取舍 + 组装层消费门控两处）。 */
    if (!g_cfg.usb_enabled)  return APP_LINK_UART;
    if (!g_cfg.uart_enabled) return APP_LINK_USB;
    return (link_arb_owner() == LINK_ARB_USB) ? APP_LINK_USB : APP_LINK_UART;
}

const char *app_tx_owner_str(void)
{
    return (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART";
}

uint8_t app_tx_usb_on(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
            hUsbDeviceFS.pClassData != NULL) ? 1u : 0u;
}