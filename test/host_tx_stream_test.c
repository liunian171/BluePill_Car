/**
 * @file    host_tx_stream_test.c
 * @brief   tx_stream 发送流核心 PC 测试桩（R3，纯逻辑零 HAL）
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/common -o build/pc_test_tx_stream.exe \
 *       test/host_tx_stream_test.c Core/Src/common/tx_stream.c Core/Src/common/ringbuf.c
 *
 * 验证重点（R3 发送路径专项 doc/发送路径专项设计.md §2）:
 *   ① free()=可收 && 队非空 → 预取 ≤TX_SEG_MAX 并 start；OK 才消费
 *   ② start=RETRY/ERR → 段不消费（数据不丢，留待下次 service）
 *   ③ 队满丢弃：dropped 计数可见（"沉默失败必可见"铁律）
 *   ④ service while-drain 多段（USB 宽松侧）与 free 门控推进
 *   ⑤ start 失败计数 txerr
 */
#include <stdio.h>
#include <string.h>
#include "tx_stream.h"
#include "ringbuf.h"

static int g_pass = 0, g_fail = 0;
static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

/* ---- 可控平台桩：记录每次 start 收到的段 + 返回码/空闲可控 ---- */
static int g_free = 1;            /* free() 返回值 */
static int g_start_ret = TX_START_OK;
static unsigned g_start_n = 0;    /* 调用次数 */
static uint8_t g_last_seg[TX_SEG_MAX];
static uint16_t g_last_len = 0;
static uint8_t g_fail_start = 0;  /* >0 时第 1 次 start 返回 g_start_ret（用于精确用例） */

static int stub_start(const uint8_t *seg, uint16_t len, void *ctx)
{
    (void)ctx;
    g_start_n++;
    memcpy(g_last_seg, seg, len);
    g_last_len = len;
    if (g_fail_start) { g_fail_start--; return g_start_ret; }
    return TX_START_OK;
}
static int stub_free(void *ctx) { (void)ctx; return g_free; }

static tx_stream_t g_s;

int main(void)
{
    printf("== tx_stream 发送流核心测试桩 ==\n");

    /* ---- ① 基础：入队即 kick，短数据整体发送 ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_free = 1; g_start_ret = TX_START_OK; g_fail_start = 0; g_start_n = 0;
        const char msg[] = "M0:60RPM\r\n";
        uint16_t acc = tx_stream_send(&g_s, (const uint8_t *)msg, (int)strlen(msg));
        check(acc == strlen(msg), "send 全部接受（队有余量）");
        check(g_start_n == 1 && g_last_len == strlen(msg), "send 后已 kick 一次 start");
        check(memcmp(g_last_seg, msg, strlen(msg)) == 0, "start 收到正确段内容");
        check(tx_stream_pending(&g_s) == 0, "短段单次发完 → 队空");
        check(tx_stream_sent(&g_s) == strlen(msg), "sent 累计正确");
    }

    /* ---- ② 分段：超 TX_SEG_MAX 一字节 → 两个 start，第一段满额，其余留在队 ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_free = 1; g_start_n = 0;
        uint8_t big[TX_SEG_MAX + 5];
        for (int i = 0; i < TX_SEG_MAX + 5; i++) big[i] = (uint8_t)i;
        tx_stream_send(&g_s, big, TX_SEG_MAX + 5);
        /* send 只 kick 一段；service 再推（free=1 会连推直到队空） */
        while (tx_stream_service(&g_s)) {}
        check(g_start_n == 2, "跨 TX_SEG_MAX 分两段发送");
        check(tx_stream_pending(&g_s) == 0, "两段发完队空");
        check(tx_stream_sent(&g_s) == TX_SEG_MAX + 5, "sent 累计 = 总字节");
        check(g_last_len == 5, "第二段为余数");
    }

    /* ---- ③ RETRY：start 返回 RETRY → 段不消费，下次 service 重发 ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_start_ret = TX_START_RETRY; g_fail_start = 1; g_start_n = 0;
        tx_stream_send(&g_s, (const uint8_t *)"A", 1);
        check(tx_stream_pending(&g_s) == 1, "RETRY 后段仍留在队（数据不丢）");
        /* 下次成功重发 */
        g_start_ret = TX_START_OK; g_fail_start = 0;
        (void)tx_stream_service(&g_s);
        check(tx_stream_pending(&g_s) == 0, "重试成功后队清空");
        check(tx_stream_txerr(&g_s) == 0, "RETRY 不计入 txerr（不是失败）");
    }

    /* ---- ④ ERR：start 返回 ERR → 计 txerr，段仍留队（可重发/丢弃由上层定） ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_start_ret = TX_START_ERR; g_fail_start = 1; g_start_n = 0;
        tx_stream_send(&g_s, (const uint8_t *)"B", 1);
        check(tx_stream_pending(&g_s) == 1, "ERR 后段仍留在队");
        check(tx_stream_txerr(&g_s) == 1, "ERR 计 txerr = 1");
    }

    /* ---- ⑤ free()=0（链路忙）→ 不推进，段滞留；free 恢复后推完 ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_free = 0; g_start_n = 0;
        tx_stream_send(&g_s, (const uint8_t *)"CC", 2);
        check(g_start_n == 0 && tx_stream_pending(&g_s) == 2, "free=0 时不 start，段滞留队内");
        g_free = 1;
        (void)tx_stream_service(&g_s);
        check(g_start_n == 1 && tx_stream_pending(&g_s) == 0, "free 恢复后 service 推发");
    }

    /* ---- ⑥ 队满丢弃：超出容量重复入队 → dropped 计数递增 ---- */
    {
        tx_stream_init(&g_s);
        tx_stream_bind(&g_s, &(tx_stream_io_t){ stub_start, stub_free, NULL });
        g_free = 0; /* 锁住不让 service 清空 → 观察队溢出 */
        uint16_t cap = (uint16_t)(RINGBUF_SIZE - 1);
        uint8_t fill[200];
        memset(fill, 0x11, sizeof(fill));
        /* 分两次投机：第一次顶满，第一次的 send 会 kick 但 free=0 不清 → 全滞留 */
        tx_stream_send(&g_s, fill, cap);
        check(tx_stream_pending(&g_s) == cap, "队可存满 cap 字节");
        uint16_t acc2 = tx_stream_send(&g_s, fill, 50);
        check(acc2 == 0, "队满后新字节全部被拒（accepted=0）");
        check(tx_stream_dropped(&g_s) == 50, "dropped 计数 = 被拒字节数（可见）");
    }

    /* ---- ⑦ 未 bind → service 空转不崩溃 ---- */
    {
        tx_stream_init(&g_s);
        (void)tx_stream_service(&g_s);
        (void)tx_stream_send(&g_s, (const uint8_t *)"D", 1);
        check(tx_stream_pending(&g_s) == 1, "未 bind 时不崩溃，数据仅滞留");
    }

    printf("\n== tx_stream 测试桩: %d PASS / %d FAIL ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}