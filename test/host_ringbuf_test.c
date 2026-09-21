/**
 * @file    host_ringbuf_test.c
 * @brief   RingBuffer PC 测试桩 (AGENTS.md §5.3 义务 7)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/common -o build/pc_test_ringbuf.exe \
 *       test/host_ringbuf_test.c Core/Src/common/ringbuf.c
 *
 * 为何补本桩（2026-09-19 节拍拉长专案顺带发现，专案文档 §5）:
 *   实测 USB 突发命令 ≥30 条（150B）丢应答、60 条丢一半（31/60），丢失量 ≈ (N−127)B
 *   —— 与旧容量 127B 定量吻合（容量偏小 + 溢出**静默**）。本次扩容至 240 并补溢出计数，
 *   本桩即锁住这两条语义（容量、可见性），防回归。
 *
 * 验证重点:
 *   ① 空/满判据：容量 = RINGBUF_SIZE − 1（牺牲一格区分空满）
 *   ② 溢出：满后写丢弃**新**数据 + **计数递增**（丢过必须可见）
 *   ③ 溢出计数饱和于 255（诊断用，只需回答量级）
 *   ④ 回绕（wrap-around）跨边界读写不丢序、不乱序
 *   ⑤ num_available 与读计数一致
 *   ⑥ 容量上限约束：RINGBUF_SIZE ≤ 255（uint8_t 索引前提）——静态断言
 */
#include <stdio.h>
#include "ringbuf.h"

/* ⑥ 索引为 uint8_t，容量不得取 256（回绕/满判据会失效） */
#if (RINGBUF_SIZE > 255) || (RINGBUF_SIZE < 2)
#error "RINGBUF_SIZE must be in [2, 255]: head/tail are uint8_t (atomic, lock-free)"
#endif

#define CAP   (RINGBUF_SIZE - 1)   /* 实际可存字节数 */

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static RingBuffer rb;

int main(void)
{
    printf("== ringbuf 测试桩 (RINGBUF_SIZE=%d, 容量=%d) ==\n", RINGBUF_SIZE, CAP);

    /* ---- ① 初始态 ---- */
    ringbuf_init(&rb);
    check(rb.head == rb.tail, "init 后 head == tail（空）");
    check(ringbuf_num_available(&rb) == 0, "init 后可读字节数 = 0");
    check(ringbuf_overflow(&rb) == 0, "init 后溢出计数归零");

    {
        uint8_t b = 0xAA;
        check(ringbuf_read(&rb, &b) == -1 && b == 0xAA, "空读返回 -1 且不改写出参");
    }

    /* ---- ② FIFO 顺序 ---- */
    ringbuf_init(&rb);
    for (uint8_t i = 0; i < 10; i++) ringbuf_write(&rb, (uint8_t)(0x10 + i));
    check(ringbuf_num_available(&rb) == 10, "写 10 字节后可读 = 10");
    {
        int ok = 1;
        for (uint8_t i = 0; i < 10; i++) {
            uint8_t b = 0;
            if (ringbuf_read(&rb, &b) != 0 || b != (uint8_t)(0x10 + i)) ok = 0;
        }
        check(ok, "读出顺序与写入一致（FIFO）");
    }

    /* ---- ③ 容量边界：可存满 CAP 字节 ---- */
    ringbuf_init(&rb);
    for (uint16_t i = 0; i < CAP; i++) ringbuf_write(&rb, (uint8_t)i);
    check(ringbuf_num_available(&rb) == CAP, "可写满容量 240-1=239 字节");
    check(ringbuf_overflow(&rb) == 0, "恰好写满不产生溢出计数");

    /* ---- ④ 溢出：丢弃新数据 + 计数递增 ---- */
    ringbuf_write(&rb, 0xEE);                       /* 第 CAP+1 字节 → 溢出 */
    check(ringbuf_overflow(&rb) == 1, "满后再写 → 溢出计数 = 1");
    check(ringbuf_num_available(&rb) == CAP, "溢出后已有数据未受影响（保护旧数据）");
    {
        uint8_t b = 0;
        ringbuf_read(&rb, &b);
        check(b == 0, "被保护的是**旧**数据（首字节仍为 0）");
    }

    /* ---- ⑤ 溢出计数饱和 255 ---- */
    for (uint16_t i = 0; i < 300; i++) ringbuf_write(&rb, 0x55);
    check(ringbuf_overflow(&rb) == 255, "溢出计数饱和于 255（uint8_t 原子性前提）");

    /* ---- ⑥ 回绕：索引多次跨越边界后读写仍成序 ---- */
    ringbuf_init(&rb);
    {
        int ok = 1;
        uint8_t w = 0, expect = 0;
        for (uint32_t round = 0; round < 50u; round++) {
            for (uint16_t i = 0; i < CAP / 4; i++) ringbuf_write(&rb, w++);
            for (uint16_t i = 0; i < CAP / 4; i++) {
                uint8_t b = 0;
                if (ringbuf_read(&rb, &b) != 0 || b != expect++) ok = 0;
            }
        }
        check(ok, "50 轮写读（索引回绕 ~12 圈）顺序正确");
        check(ringbuf_overflow(&rb) == 0, "收发平衡时无溢出");
        check(ringbuf_num_available(&rb) == 0, "收支平衡后回到空态");
    }

    /* ---- ⑦ 读空后计数归零 ---- */
    ringbuf_init(&rb);
    for (uint16_t i = 0; i < CAP; i++) ringbuf_write(&rb, (uint8_t)i);
    for (uint16_t i = 0; i < CAP; i++) { uint8_t b = 0; ringbuf_read(&rb, &b); }
    check(ringbuf_num_available(&rb) == 0 && rb.head == rb.tail, "读空后回到空态");

    /* ================= R3: peek / commit（分段发送预取） ================= */
    {   /* ⑧ peek 不消费：拷贝到 dst 但 tail 不动 */
        ringbuf_init(&rb);
        for (uint8_t i = 0; i < 10; i++) ringbuf_write(&rb, (uint8_t)(0x20 + i));
        uint8_t tmp[10] = {0};
        uint16_t n = ringbuf_peek(&rb, tmp, 4);
        check(n == 4, "peek 最多拷贝 max 字节");
        check(tmp[0] == 0x20 && tmp[3] == 0x23, "peek 拷贝内容正确（从 tail 起）");
        check(ringbuf_num_available(&rb) == 10, "peek 不推进 tail（未消费）");
    }
    {   /* ⑨ commit 推进 tail：与 peek 配对消费 */
        uint8_t tmp[10] = {0};
        ringbuf_peek(&rb, tmp, 4);
        ringbuf_commit(&rb, 4);
        check(ringbuf_num_available(&rb) == 6, "commit 4 字节后未读余 6");
        uint8_t b = 0;
        ringbuf_read(&rb, &b);
        check(b == 0x24, "commit 后下一字节是原第 5 字节（tail 正确跳进）");
    }
    {   /* ⑩ peek 跨回绕：填充到 head 恰好越过缓存末尾回绕，peek 从 tail 读跨边界成序 */
        ringbuf_init(&rb);
        for (uint16_t i = 0; i < CAP; i++) ringbuf_write(&rb, (uint8_t)i);       /* 满, head=239 */
        for (uint16_t i = 0; i < CAP; i++) { uint8_t b = 0; ringbuf_read(&rb, &b); } /* 空, head=tail=239 */
        for (uint16_t i = 0; i < 10; i++) ringbuf_write(&rb, (uint8_t)(0x50 + i));    /* 跨 239->0, tail=239 */
        uint8_t tmp[16] = {0};
        uint16_t n = ringbuf_peek(&rb, tmp, 16);
        int ok = (n == 10);
        for (uint16_t i = 0; i < n && ok; i++) if (tmp[i] != (uint8_t)(0x50 + i)) ok = 0;
        check(ok, "peek 跨缓冲回绕仍成序（239->0）");
    }
    {   /* ⑪ commit 越界钳制：n>avail 时只推 avail */
        ringbuf_init(&rb);
        for (uint8_t i = 0; i < 5; i++) ringbuf_write(&rb, (uint8_t)(0x40 + i));
        ringbuf_commit(&rb, 999);
        check(ringbuf_num_available(&rb) == 0, "commit 越界钳制到 avail（不越过 head）");
    }

    printf("\n== ringbuf 测试桩: %d PASS / %d FAIL ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
