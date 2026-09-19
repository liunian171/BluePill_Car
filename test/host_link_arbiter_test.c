/* host_link_arbiter_test.c — link_arbiter 纯逻辑 PC 测试桩
 * 覆盖: doc/双链路仲裁设计文档.md §8 T9 (状态机全转移) + T10 (边界)
 * 编译: gcc -Wall -Wextra -I Core/Inc/driver -o pc_test_link_arbiter.exe
 *       test/host_link_arbiter_test.c Core/Src/driver/link_arbiter.c
 */
#include <stdio.h>
#include <string.h>
#include "link_arbiter.h"

static int g_fail = 0;
static char g_last_broadcast[64];

static uint8_t stub_session_off(void) { return 0; }
static uint8_t stub_session_on(void)  { return 1; }
static void stub_broadcast(const char *msg) {
    strncpy(g_last_broadcast, msg, sizeof(g_last_broadcast) - 1);
    g_last_broadcast[sizeof(g_last_broadcast) - 1] = '\0';
}

#define CHECK(cond, name) do { \
    if (cond) printf("  PASS  %s\n", name); \
    else { printf("  FAIL  %s\n", name); g_fail++; } } while (0)

int main(void)
{
    LinkArbCfg cfg = { .session_active = stub_session_off,
                       .broadcast      = stub_broadcast,
                       .boot_grace_ms  = 3000 };

    /* ---- 1. 上电未知态: USB 主导, 宽限期内不 failover ---- */
    link_arb_init(&cfg, -1, 0);
    link_arb_notify(-1, 1500);              /* 宽限期内仍未知 */
    CHECK(link_arb_owner() == LINK_ARB_USB, "未知态宽限期内保持 USB owner");

    /* ---- 2. 未知态 → 变活: owner 不变 + USB READY ---- */
    g_last_broadcast[0] = '\0';
    link_arb_notify(1, 2000);
    CHECK(link_arb_owner() == LINK_ARB_USB, "未知→活 owner 不变");
    CHECK(strstr(g_last_broadcast, "USB READY") != 0, "变活广播 USB READY");

    /* ---- 3. 活 → 死: 自动 failover + 广播 ---- */
    g_last_broadcast[0] = '\0';
    (void)link_arb_notify(0, 5000);
    CHECK(link_arb_owner() == LINK_ARB_UART, "USB 失联自动 failover 到 UART");
    CHECK(strstr(g_last_broadcast, "USB LOST") != 0, "failover 广播 USB LOST");

    /* ---- 4. UART owner 时 USB 再失联: 不重复 failover ---- */
    CHECK(link_arb_notify(0, 6000) == 0, "已 UART owner, USB 死不产生切换");

    /* ---- 5. 恢复不抢回: 0→1 仅广播, owner 仍 UART ---- */
    g_last_broadcast[0] = '\0';
    link_arb_notify(1, 7000);
    CHECK(link_arb_owner() == LINK_ARB_UART, "USB 恢复不自动抢回");
    CHECK(strstr(g_last_broadcast, "USB READY") != 0, "恢复广播 USB READY");

    /* ---- 6. USB 显式抢回 ---- */
    CHECK(link_arb_request(LINK_ARB_USB) == LINK_ARB_OK, "LINK USB 抢回 OK");
    CHECK(link_arb_owner() == LINK_ARB_USB, "抢回后 owner=USB");

    /* ---- 7. 让出 ---- */
    CHECK(link_arb_request(LINK_ARB_UART) == LINK_ARB_OK, "LINK UART 让出 OK");
    CHECK(link_arb_owner() == LINK_ARB_UART, "让出后 owner=UART");

    /* ---- 8. 会话锁: 拒绝切换 ---- */
    link_arb_init(&cfg, 1, 0);              /* 重置, USB alive */
    cfg.session_active = stub_session_on;
    link_arb_init(&cfg, 1, 0);
    CHECK(link_arb_request(LINK_ARB_UART) == LINK_ARB_ERR_SESSION, "会话激活时切换被拒");
    CHECK(link_arb_owner() == LINK_ARB_USB, "被拒后 owner 不变");
    cfg.session_active = stub_session_off;
    link_arb_init(&cfg, 1, 0);

    /* ---- 9. 非法参数 ---- */
    CHECK(link_arb_request((LinkArbOwner)99) == LINK_ARB_ERR_BAD_ARG, "非法目标拒绝");

    /* ---- 10. 上电即确定没插 USB: 直接 UART owner ---- */
    link_arb_init(&cfg, 0, 0);
    CHECK(link_arb_owner() == LINK_ARB_UART, "上电无 USB 直接 UART owner");

    /* ---- 11. 未知态宽限超时仍死 → failover (真机喂法: 宽限内喂 0/1, 非持续 -1) ---- */
    link_arb_init(&cfg, -1, 0);
    link_arb_notify(0, 1500);               /* 宽限内明确死: 观望, 不锁存不切换 */
    CHECK(link_arb_owner() == LINK_ARB_USB, "宽限内喂 0 仍观望 (USB owner)");
    link_arb_notify(0, 2999);
    CHECK(link_arb_owner() == LINK_ARB_USB, "宽限期最后一刻仍观望");
    (void)link_arb_notify(0, 3000);
    CHECK(link_arb_owner() == LINK_ARB_UART, "宽限超时仍死 → UART 接管");

    /* ---- 11b. 未知态一拍跨过宽限且死 → 立即 failover ---- */
    link_arb_init(&cfg, -1, 0);
    (void)link_arb_notify(0, 5000);
    CHECK(link_arb_owner() == LINK_ARB_UART, "首拍即过宽限且死 → 立即接管");

    /* ---- 12. UART owner 下 USB 恢复 → 失联: 不产生 failover ---- */
    link_arb_notify(1, 4000);
    link_arb_request(LINK_ARB_UART);
    link_arb_notify(0, 5000);
    CHECK(link_arb_owner() == LINK_ARB_UART, "UART owner 时 USB 死, owner 不变");

    printf("\n%s\n", g_fail ? "结果: FAIL" : "结果: ALL PASS");
    return g_fail ? 1 : 0;
}
