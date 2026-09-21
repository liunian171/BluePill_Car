/**
 * ============================================================================
 *  app_link.c — 命令链宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（时间基准传入; 发送/页记录/接管权经 io 注入）。
 *  搬迁自: main.c 双 ringbuf 消费循环 + 指挥权门控（BUSY 拒止）+ 帧重同步。
 *
 *  ▸ R2 (2026-09-21) 命令执行抽离 ◂
 *    本文件瘦身为 消费/门控/帧解析 骨架 —— 命令的**执行与应答编码**整体
 *    迁至 cmd_exec（纯逻辑、全函数指针注入、PC 桩 host_cmd_exec_test 24/24）。
 *    保留在本文件：双 ringbuf 消费 + 文本/二进制 vs owner 的 BUSY 门控
 *    + 帧/A326 重同步 + 应答编码助手（BUSY 拒止应答用）。
 *    语义承诺: 与 R2 前逐字节等价（放行命令的应答由 cmd_exec 产出，
 *    BUSY 拒止应答仍本文件直接产出）。
 *
 *  ▸ P1-8 (2026-09-20) ◂
 *    USB 收包经 usbd_cdc_if_register_rx_sink(app_link_usb_rx_isr) 注入。
 *
 *  ▸ C4 收敛 (2026-09-20) ◂
 *    11 处 line_follower_enable(0) 收敛为 io.override_manual 注入点。
 * ============================================================================
 */

#include "app/app_link.h"
#include "app/cmd_exec.h"
#include "driver/link_arbiter.h"
#include "driver/txt_cmd.h"
#include "driver/line_follower.h"
#include "common/ringbuf.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- 私有状态 ---- */

static app_link_cfg_t s_cfg;
static app_link_io_t  s_io;
static uint8_t        s_inited = 0;

/* 双 ringbuf (ISR 写/任务读) */
static RingBuffer s_rb_uart;
static RingBuffer s_rb_usb;

/* 按链路独立的解析状态 */
static uint8_t  s_txt[APP_LINK_COUNT][20];
static uint8_t  s_txt_len[APP_LINK_COUNT];
static uint8_t  s_frame[APP_LINK_COUNT][32];
static uint8_t  s_f_len[APP_LINK_COUNT];
static uint8_t  s_in_frame[APP_LINK_COUNT];
static uint32_t s_t_frame[APP_LINK_COUNT];

/* UART rx 记账 (页7 "U:<秒>" 数据源) */
static uint32_t s_uart_last_rx = 0;
static uint8_t  s_uart_seen    = 0;
static uint32_t s_now          = 0;

/* ---- 出口助手（BUSY 拒止应答 + 寻迹诊断用; 命令应答已迁 cmd_exec） ---- */

/* 裸发送: 不记录 OLED 应答页 */
static void tx_raw(const char *buf, int n)
{
    s_io.sink.send(buf, n, s_io.sink.ctx);
}

/* 文本回应 + 页4 记录 */
static void ack(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        tx_raw(buf, n);
        s_io.page_note.note_resp(buf, s_io.page_note.ctx);
    }
}

/* 页6 命令记录 */
static void cmd_note(const char *fmt, ...)
{
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s_io.page_note.note_cmd(buf, s_io.page_note.ctx);
}

/* 寻迹诊断代理: 转发 line_follower 事件到串口 (高频, 不占应答页) */
static void line_diag(const char *msg)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", msg);
    if (n > 0) tx_raw(buf, n);
}

/* [R2] 命令执行注入面: wiring 已经填好 io.cmd, 直接从 s_io 取, 命令执行委托 cmd_exec */
static const cmd_exec_io_t *cmd_io(void) { return &s_io.cmd; }

/* ---- 生命周期 ---- */

bridge_ret_t app_link_init(const app_link_cfg_t *cfg, const app_link_io_t *io)
{
    if (cfg == NULL || io == NULL) return BRIDGE_ERR_BAD_ARG;
    if (cfg->byte_budget == 0 || cfg->frame_timeout_ms == 0) return BRIDGE_ERR_BAD_CFG;
    if (io->sink.send == NULL || io->page_note.note_cmd == NULL ||
        io->page_note.note_resp == NULL || io->set_active == NULL ||
        io->owner_link == NULL || io->override_manual == NULL ||
        io->wd_feed == NULL || io->session_active == NULL)
        return BRIDGE_ERR_BAD_ARG;
    s_cfg = *cfg;
    s_io  = *io;
    ringbuf_init(&s_rb_uart);
    ringbuf_init(&s_rb_usb);
    memset(s_txt_len, 0, sizeof(s_txt_len));
    memset(s_in_frame, 0, sizeof(s_in_frame));
    memset(s_f_len, 0, sizeof(s_f_len));
    /* 寻迹诊断事件回调注册 (保留; cmd_exec 只做命令执行, 事件外泄仍归命令链宿主) */
    line_follower_set_event_cb(line_diag);
    s_inited = 1;
    return BRIDGE_OK;
}

/* ---- ISR 注入（只写 ringbuf, 不解析） ---- */

void app_link_uart_rx_isr(uint8_t byte) { ringbuf_write(&s_rb_uart, byte); }
void app_link_usb_rx_isr(uint8_t byte)  { ringbuf_write(&s_rb_usb, byte); }

/* ---- 只读查询 ---- */

uint16_t app_link_rx_overflow(void)
{
    uint16_t ovf = (uint16_t)ringbuf_overflow(&s_rb_uart);
    if (s_cfg.usb_enabled)
        ovf = (uint16_t)(ovf + ringbuf_overflow(&s_rb_usb));
    return ovf;
}

uint32_t app_link_uart_silence_s(void)
{
    if (!s_uart_seen) return 0xFFFFFFFFu;
    return (s_now - s_uart_last_rx) / 1000;
}

/* ---- 命令执行委托（门控后; BUSY 拒止仍在门控处） ---- */

/* 非 owner 拒止应答：文本/二进制共用 (拒绝命令已由调用方滤出) */
static void reply_busy(void)
{
    cmd_note("BUSY");
    ack("BUSY:%s\r\n", (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
}

/* ---- 消费任务 ---- */

void app_link_task(uint32_t now)
{
    if (!s_inited) return;
    s_now = now;

    static uint8_t b;   /* 单字节暂存 */

    for (uint8_t li = 0; li < APP_LINK_COUNT; li++) {
        if (li == APP_LINK_UART   && !s_cfg.uart_enabled) continue;
        if (li == APP_LINK_USB    && !s_cfg.usb_enabled)  continue;
        uint8_t budget = s_cfg.byte_budget;
        RingBuffer *rb = (li == APP_LINK_UART) ? &s_rb_uart : &s_rb_usb;
        while (budget-- && ringbuf_read(rb, &b) == 0) {
            s_io.set_active(li, s_io.ctx);
            if (li == APP_LINK_UART) { s_uart_last_rx = now; s_uart_seen = 1; }
            if (li == s_io.owner_link(s_io.ctx)) {
                s_io.wd_feed(s_io.ctx);   /* 看门狗: owner 字节算"活" */
            }
            if (b == 0xAA && !s_in_frame[li]) { s_f_len[li] = 0; s_in_frame[li] = 1; s_t_frame[li] = now; }
            if (s_in_frame[li]) {
                if (s_f_len[li] >= sizeof(s_frame[li])) { s_in_frame[li] = 0; continue; }
                s_frame[li][s_f_len[li]++] = b;
                s_t_frame[li] = now;
                if (s_f_len[li] >= 2 && s_frame[li][s_f_len[li]-2] == 0xFF && s_frame[li][s_f_len[li]-1] == 0xFF) {
                    uint8_t cmd = s_frame[li][1];
                    /* [R2] data = 帧参数字节 (AA/CMD 之后, FF/FF 之前) */
                    const uint8_t *data = &s_frame[li][2];
                    uint8_t len = (uint8_t)(s_f_len[li] - 2);
                    /* [P2 仲裁门控] 二进制帧: 非 owner 仅放行心跳 0xF0, 其余回 BUSY */
                    if (li != s_io.owner_link(s_io.ctx) && cmd != 0xF0) {
                        reply_busy();
                    } else {
                        cmd_exec_bin(cmd_io(), cmd, data, len, now);
                    }
                    s_in_frame[li] = 0; s_f_len[li] = 0;
                }
                continue;
            }
            if (b == '\r') continue;
            if (b == '\n') s_txt[li][s_txt_len[li]] = 0;
            else if (s_txt_len[li] < 19) { s_txt[li][s_txt_len[li]++] = b; continue; }
            if (s_txt_len[li] > 0) {
                TxtCmd tc;
                if (txt_cmd_parse((char *)s_txt[li], &tc)) {
                    /* [P2 仲裁门控] 非 owner 仅放行安全例外 (STOP/PING/LINK) */
                    if (li != s_io.owner_link(s_io.ctx) &&
                        tc.type != TXTCMD_PING && tc.type != TXTCMD_STOP &&
                        tc.type != TXTCMD_LINK) {
                        reply_busy();
                    } else {
                        cmd_exec_text(cmd_io(), &tc, now);
                    }
                } else {
                    cmd_note("ERR TXT");
                    ack("?\r\n");
                }
            }
            s_txt_len[li] = 0;
          }
        }

        /* 无线丢包保护: 二进制帧不完整超时丢弃 */
        for (uint8_t li = 0; li < APP_LINK_COUNT; li++)
            if (s_in_frame[li] && (now - s_t_frame[li]) > s_cfg.frame_timeout_ms) {
                s_in_frame[li] = 0; s_f_len[li] = 0;
            }
}