/**
 * @file    host_speed_loop_test.c
 * @brief   speed_loop 执行组件 PC 测试桩 (AGENTS.md §5.3 义务 7: PC 桩先行)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc -I Core/Inc/driver -I Core/Inc/common \
 *       -o build/pc_test_speed_loop.exe \
 *       test/host_speed_loop_test.c Core/Src/driver/speed_loop.c Core/Src/common/pid.c
 *
 * 验证重点（对应 doc/结构优化顺序分析.md §9.2 C2 契约）:
 *   ① 零硬件依赖：输出/刹停/编码器读全部经函数指针注入 → 本桩能编译即证明
 *   ② 测速：回绕校正 + 反馈符号（与开环阶跃测试同一份实现）
 *   ③ 闭环：前馈直通、输出限幅、0 速必停（含内轮停车）
 *   ④ 停车后重启不丢帧（基准由节拍用真实 now 重建）
 *   ⑤ 错误通道：非法 id / 未初始化 / 非有限值 / 负增益一律返回非 OK
 *   ⑥ 参数与状态读出（遥测/显示共用，不再外泄到组装层 static）
 */
#include <stdio.h>
#include "driver/speed_loop.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static int feq(float a, float b)
{
    float d = a - b;
    if (d < 0) d = -d;
    return d < 0.001f;
}

/* ---- 注入桩：模拟"编码器 / 桥接层" ---- */
static int32_t g_enc[2]        = {0, 0};      /* 假编码器计数 */
static int     g_brake_cnt[2]  = {0, 0};      /* 物理刹停次数 */
static int     g_out_cnt[2]    = {0, 0};      /* 下发次数 */
static float   g_last_out[2]   = {0.0f, 0.0f};/* 最近下发值 */
static int     g_bad_out_id    = 0;           /* 非法 id 下发计数 */

static void    stub_set_rpm(uint8_t id, float rpm)
{
    if (id > 1) { g_bad_out_id++; return; }
    g_out_cnt[id]++; g_last_out[id] = rpm;
}
static void    stub_brake  (uint8_t id) { if (id > 1) { g_bad_out_id++; return; } g_brake_cnt[id]++; }
static int32_t stub_read_enc(uint8_t id) { return g_enc[id]; }

static const speed_loop_io_t IO = {
    .set_rpm  = stub_set_rpm,
    .brake    = stub_brake,
    .read_enc = stub_read_enc,
};

/* 本车标定表（ppr 取 1000 便于手算：dt=0.05s 时 rpm = delta × 1.2） */
static speed_loop_cfg_t mk_cfg(void)
{
    speed_loop_cfg_t c = {
        .ch_count = 2,
        .ch = {
            { .ppr = 1000.0f, .enc_span = 65536, .fb_sign =  1,
              .ff_gain = 1.0f, .out_min = -319.0f, .out_max = 319.0f,
              .kp = 0.0f, .ki = 0.0f, .kd = 0.0f },
            { .ppr = 1000.0f, .enc_span = 65536, .fb_sign = -1,
              .ff_gain = 1.0f, .out_min = -319.0f, .out_max = 319.0f,
              .kp = 0.0f, .ki = 0.0f, .kd = 0.0f },
        }
    };
    return c;
}

int main(void)
{
    printf("=== speed_loop 执行组件 PC 桩 ===\n\n");

    /* ================= 组 1: 未初始化保护 + 初始化校验 ================= */
    printf("---- 组 1: 未初始化保护 / 初始化配置校验 ----\n");
    float tmp;
    speed_loop_state_t st;
    check(speed_loop_is_init() == 0,                       "init 前 is_init == 0");
    check(speed_loop_update(0) == SPEED_LOOP_ERR_NOT_INIT, "init 前 update -> NOT_INIT");
    check(speed_loop_set_target(0, 10.0f) == SPEED_LOOP_ERR_NOT_INIT,
                                                          "init 前 set_target -> NOT_INIT");
    check(speed_loop_measure(0, 0, &tmp) == SPEED_LOOP_ERR_NOT_INIT,
                                                          "init 前 measure -> NOT_INIT");
    check(speed_loop_get_state(0, &st) == SPEED_LOOP_ERR_NOT_INIT,
                                                          "init 前 get_state -> NOT_INIT");

    speed_loop_cfg_t cfg = mk_cfg();
    check(speed_loop_init(NULL, &IO) == SPEED_LOOP_ERR_BAD_ARG,   "init(NULL cfg) -> BAD_ARG");
    check(speed_loop_init(&cfg, NULL) == SPEED_LOOP_ERR_BAD_ARG,  "init(NULL io) -> BAD_ARG");

    speed_loop_io_t bad_io = IO; bad_io.brake = NULL;
    check(speed_loop_init(&cfg, &bad_io) == SPEED_LOOP_ERR_BAD_ARG,
                                                                  "init(io 缺 brake) -> BAD_ARG");
    bad_io = IO; bad_io.read_enc = NULL;
    check(speed_loop_init(&cfg, &bad_io) == SPEED_LOOP_ERR_BAD_ARG,
                                                                  "init(io 缺 read_enc) -> BAD_ARG");

    speed_loop_cfg_t c2 = mk_cfg(); c2.ch_count = 0;
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "ch_count=0 -> BAD_CFG");
    c2 = mk_cfg(); c2.ch_count = SPEED_LOOP_MAX_CH + 1;
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "ch_count 超上限 -> BAD_CFG");
    c2 = mk_cfg(); c2.ch[0].ppr = 0.0f;
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "ppr=0 -> BAD_CFG(除零保护)");
    c2 = mk_cfg(); c2.ch[0].out_min = 400.0f;   /* 400 > out_max 319 → 区间非法 */
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "out_min>=out_max -> BAD_CFG");
    c2 = mk_cfg(); c2.ch[0].kp = -1.0f;
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "kp<0 -> BAD_CFG");
    c2 = mk_cfg(); c2.ch[0].ff_gain = -1.0f;
    check(speed_loop_init(&c2, &IO) == SPEED_LOOP_ERR_BAD_CFG,    "ff_gain<0 -> BAD_CFG");

    cfg = mk_cfg();
    check(speed_loop_init(&cfg, &IO) == SPEED_LOOP_OK,            "合法配置 init -> OK");
    check(speed_loop_is_init() == 1,                              "init 后 is_init == 1");

    check(speed_loop_get_state(0, &st) == SPEED_LOOP_OK &&
          feq(st.target_rpm, 0.0f) && feq(st.out_rpm, 0.0f) && st.running == 0,
          "init 不发车: 目标/输出全 0, running=0");

    /* ================= 组 2: 测速（换算 / 回绕 / 符号 / dt 守卫） ================= */
    printf("\n---- 组 2: 测速 ----\n");
    speed_loop_init(&cfg, &IO);              /* 重置内部状态 */
    g_enc[0] = 0; g_enc[1] = 0;

    check(speed_loop_measure(0, 1000, &tmp) == SPEED_LOOP_ERR_BAD_ARG,
          "首帧只建基准、不产出值 -> BAD_ARG(等价原实现 dt=0 丢弃)");

    g_enc[0] = 100;                          /* delta=100, dt=0.05s, ppr=1000 */
    check(speed_loop_measure(0, 1050, &tmp) == SPEED_LOOP_OK && feq(tmp, 120.0f),
          "delta=100/dt=50ms/ppr=1000 -> +120 RPM");

    /* 16 位回绕：65000 -> 100 应判为 +636 而非 -64900 */
    speed_loop_resync(0, 2000); g_enc[0] = 65000;
    speed_loop_measure(0, 2050, &tmp);       /* 基准落在 65000 */
    g_enc[0] = 100;
    check(speed_loop_measure(0, 2100, &tmp) == SPEED_LOOP_OK && feq(tmp, 763.2f),
          "计数回绕 65000->100 校正为 +636 -> +763.2 RPM（未校正会得 -77880）");

    /* 反向回绕：100 -> 65000 应判为 -636 */
    speed_loop_resync(0, 3000); g_enc[0] = 100;
    speed_loop_measure(0, 3050, &tmp);
    g_enc[0] = 65000;
    check(speed_loop_measure(0, 3100, &tmp) == SPEED_LOOP_OK && feq(tmp, -763.2f),
          "反向回绕 100->65000 校正为 -636 -> -763.2 RPM");

    /* 反馈符号：id1 配置 fb_sign=-1，同样正转应得负 RPM */
    speed_loop_resync(1, 4000); g_enc[1] = 0;
    speed_loop_measure(1, 4050, &tmp);
    g_enc[1] = 100;
    check(speed_loop_measure(1, 4100, &tmp) == SPEED_LOOP_OK && feq(tmp, -120.0f),
          "id1 fb_sign=-1 -> 同向计数得 -120 RPM（镜像驱动侧取反）");

    /* dt 守卫 */
    speed_loop_resync(0, 5000);
    check(speed_loop_measure(0, 5005, &tmp) == SPEED_LOOP_ERR_BAD_ARG,
          "dt=5ms 过小 -> BAD_ARG（不做除法，防噪声放大）");
    check(speed_loop_measure(0, 9000, &tmp) == SPEED_LOOP_ERR_BAD_ARG,
          "dt=4s 过大 -> BAD_ARG（基准失效已重同步）");
    check(speed_loop_measure(0, 9050, &tmp) == SPEED_LOOP_OK,
          "重同步后下一帧即恢复有效");

    /* resync 使基准作废 */
    speed_loop_reset(0);
    check(speed_loop_measure(0, 10000, &tmp) == SPEED_LOOP_ERR_BAD_ARG,
          "reset 后基准作废 -> 需重建");

    check(speed_loop_measure(9, 11000, &tmp) == SPEED_LOOP_ERR_BAD_ARG,
          "id 越界 -> BAD_ARG");

    /* ================= 组 3: 闭环更新（前馈 / 限幅 / 停车语义） ================= */
    printf("\n---- 组 3: 闭环更新 ----\n");
    speed_loop_init(&cfg, &IO);
    g_enc[0] = 0; g_enc[1] = 0;
    speed_loop_resync(0, 20000);
    speed_loop_resync(1, 20000);

    speed_loop_set_target(0, 100.0f);
    speed_loop_set_target(1, 100.0f);
    g_enc[0] = 100; g_enc[1] = 100;          /* 实际 120 / -120 RPM */
    check(speed_loop_update(20050) == SPEED_LOOP_OK, "update -> OK");

    /* 零增益 + ff=1 → 输出 = 目标（纯前馈直通） */
    check(feq(g_last_out[0], 100.0f) && feq(g_last_out[1], 100.0f),
          "前馈直通: 零增益时 out == target (100 RPM)");
    speed_loop_get_state(0, &st);
    check(feq(st.actual_rpm, 120.0f) && st.running == 1 && g_out_cnt[0] == 1,
          "状态: actual=120, running=1, 下发 1 次(经注入桩)");
    check(feq(st.out_rpm, 100.0f), "状态: out_rpm 与实际下发一致");

    /* 输出限幅 */
    speed_loop_set_target(0, 1000.0f);
    g_enc[0] = 200;
    speed_loop_update(20100);
    check(feq(g_last_out[0], 319.0f), "目标 1000 超限 -> 输出钳到 out_max=319");

    /* 停车语义：|目标| < MIN_RUN_RPM */
    speed_loop_set_target(0, 0.0f);
    g_enc[0] = 300;
    speed_loop_update(20150);
    speed_loop_get_state(0, &st);
    check(st.running == 0 && feq(st.out_rpm, 0.0f) && feq(st.actual_rpm, 0.0f),
          "目标 0 -> 停车: running=0, out=0, actual=0");
    check(g_brake_cnt[0] > 0, "停车触发物理刹停(注入桩被调用)");
    check(g_out_cnt[0] == 2, "停车后不再下发新值(下发次数未增加)");

    speed_loop_set_target(1, 0.5f);           /* 内轮停车等价场景: 幅值 < 1 */
    speed_loop_update(20150);
    speed_loop_get_state(1, &st);
    check(st.running == 0, "|目标|=0.5 < MIN_RUN_RPM -> 停车(内轮停车同一语义)");
    check(feq(st.target_rpm, 0.5f), "停车时保留原始意图(遥测如实反映)");

    /* 停车后重启不丢帧：停车期由节拍重建基准 → 重启首帧即有效 */
    speed_loop_set_target(0, 100.0f);
    g_enc[0] = 400;                           /* 停车时基准落在 300 (上一帧) */
    speed_loop_update(20200);
    speed_loop_get_state(0, &st);
    check(st.running == 1 && feq(st.actual_rpm, 120.0f),
          "重启首帧即产出有效测速(120 RPM) —— 停车期基准由节拍用真实 now 维护");

    /* 目标为负（倒车）：带符号域 */
    speed_loop_set_target(0, -100.0f);
    g_enc[0] = 300;
    speed_loop_update(20250);
    speed_loop_get_state(0, &st);
    check(feq(st.target_rpm, -100.0f) && feq(g_last_out[0], -100.0f),
          "负目标 = 倒车: 目标与输出同为负(带符号转速域)");

    /* ================= 组 4: 参数读写 / 状态读出 / 错误通道 ================= */
    printf("\n---- 组 4: 参数与状态 ----\n");
    float kp, ki, kd, ff;

    check(speed_loop_set_gains(0, 0.87f, 0.40f, 0.05f) == SPEED_LOOP_OK, "set_gains -> OK");
    check(speed_loop_get_gains(0, &kp, &ki, &kd) == SPEED_LOOP_OK &&
          feq(kp, 0.87f) && feq(ki, 0.40f) && feq(kd, 0.05f),
          "get_gains 回读一致(0.87/0.40/0.05)");

    check(speed_loop_set_gains(0, -1.0f, 0.0f, 0.0f) == SPEED_LOOP_ERR_BAD_ARG,
          "负增益(正反馈) -> BAD_ARG");
    speed_loop_get_gains(0, &kp, &ki, &kd);
    check(feq(kp, 0.87f), "非法增益被拒后原值不变(不静默写入)");

    check(speed_loop_set_ff_gain(0, 1.25f) == SPEED_LOOP_OK &&
          speed_loop_get_ff_gain(0, &ff) == SPEED_LOOP_OK && feq(ff, 1.25f),
          "ff_gain 在线整定 1.25 回读一致");
    check(speed_loop_set_ff_gain(0, -0.5f) == SPEED_LOOP_ERR_BAD_ARG, "负前馈 -> BAD_ARG");

    check(speed_loop_get_gains(0, NULL, &ki, &kd) == SPEED_LOOP_ERR_BAD_ARG,
          "get_gains(出参 NULL) -> BAD_ARG");
    check(speed_loop_get_state(0, NULL) == SPEED_LOOP_ERR_BAD_ARG,
          "get_state(出参 NULL) -> BAD_ARG");
    check(speed_loop_set_target(2, 10.0f) == SPEED_LOOP_ERR_BAD_ARG,
          "set_target(id 越界) -> BAD_ARG");
    check(speed_loop_stop(7) == SPEED_LOOP_ERR_BAD_ARG, "stop(id 越界) -> BAD_ARG");

    float nan_v = __builtin_nanf("");
    check(speed_loop_set_target(0, nan_v) == SPEED_LOOP_ERR_BAD_ARG,
          "set_target(NaN) -> BAD_ARG(脏值不进积分器)");

    /* stop() 的完整语义 */
    speed_loop_set_target(0, 100.0f);
    int brk_before = g_brake_cnt[0];
    check(speed_loop_stop(0) == SPEED_LOOP_OK && g_brake_cnt[0] == brk_before + 1,
          "stop() 立刻物理刹停");
    speed_loop_get_state(0, &st);
    check(feq(st.target_rpm, 0.0f) && st.running == 0, "stop() 清目标 + running=0");

    check(g_bad_out_id == 0, "全程无非法 id 下发到注入通道");

    printf("\n=== 结果: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
