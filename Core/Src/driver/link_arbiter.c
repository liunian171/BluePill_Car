/**
 * @file    link_arbiter.c
 * @brief   双链路指挥权仲裁 — 策略实现 (纯逻辑, 零 HAL)
 *
 * 设计权威: doc/双链路仲裁设计文档.md §2 §4。
 * 安全铁律在分发层: STOP/LINK ? 不经本组件、任意链路直达。
 */

#include "link_arbiter.h"

/* ---- 内部状态 ---- */
static LinkArbOwner  g_owner;            /* 当前指挥权 */
static LinkArbCfg    g_cfg;              /* 配置 + IO 注入 (值拷贝, 消除悬垂指针) */
static int8_t  g_alive_prev;             /* 上一拍 usb_alive (1/0/-1) */
static uint32_t g_t0;                    /* 上电 tick (宽限期基准) */

void link_arb_init(const LinkArbCfg *cfg, int8_t usb_alive_init, uint32_t now)
{
    /* cfg 值拷贝: 调用方栈/静态生命周期不保证存活 → 只存指针曾致悬垂 HardFault
     * (2026-09-20 真机, app_wiring 需 static 规避)。拷贝后调用方返回也安全。
     * session_active/broadcast 是函数指针, 同族组件非空契约由调用方满足。 */
    g_cfg       = *cfg;
    g_t0        = now;
    g_alive_prev = usb_alive_init;
    /* 上电默认 USB 主导; 仅当"确定没插 USB"(init 即 0) 才直接 UART 接管。
     * 未知态(-1) 先按 USB 主导, 由宽限期逻辑兜底 (超时不活 → 自动切 UART)。 */
    g_owner = (usb_alive_init == 0) ? LINK_ARB_UART : LINK_ARB_USB;
}

static void do_failover(void)
{
    g_owner = LINK_ARB_UART;
    if (g_cfg.broadcast != 0)
        g_cfg.broadcast("USB LOST -> UART TAKEOVER\r\n");
}

LinkArbRet link_arb_request(LinkArbOwner target)
{
    if (target != LINK_ARB_USB && target != LINK_ARB_UART)
        return LINK_ARB_ERR_BAD_ARG;
    if (g_cfg.session_active != 0 && g_cfg.session_active())
        return LINK_ARB_ERR_SESSION;    /* 会话锁 (设计文档 §2.3) */
    g_owner = target;
    return LINK_ARB_OK;
}

uint8_t link_arb_notify(int8_t alive, uint32_t now)
{
    /* 未知态 (-1): 宽限期内观望 (上电枚举异步完成, 主循环喂的是 0/1 而非 -1);
     * 宽限内变活 → READY; 宽限已过仍是死/未知 → USB 未上线, UART 接管 */
    if (g_alive_prev == -1) {
        if (alive == 1) {
            g_alive_prev = 1;
            if (g_cfg.broadcast != 0)
                g_cfg.broadcast("USB READY\r\n");
            return 0;
        }
        if ((now - g_t0) < g_cfg.boot_grace_ms)
            return 0;                       /* 宽限期内: 继续观望, 不锁存不切换 */
        g_alive_prev = 0;
        if (g_owner == LINK_ARB_USB) {
            do_failover();
            return 1;
        }
        return 0;
    }

    if (alive == g_alive_prev) return 0;

    /* 0 → 1: 恢复 — 只广播, 不抢回 (用户决议 2026-09-19) */
    if (alive == 1) {
        g_alive_prev = 1;
        if (g_cfg.broadcast != 0)
            g_cfg.broadcast("USB READY\r\n");
        return 0;
    }

    /* 1 → 0: 失联 — 仅当 USB 是 owner 才 failover (UART owner 时 USB 掉线无所谓) */
    g_alive_prev = 0;
    if (g_owner == LINK_ARB_USB) {
        do_failover();
        return 1;
    }
    return 0;
}

LinkArbOwner link_arb_owner(void)
{
    return g_owner;
}
