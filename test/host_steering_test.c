/**
 * @file    host_steering_test.c
 * @brief   steering 执行组件 PC 测试桩 (AGENTS.md §5.3 义务 7: PC 桩先行)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/driver -o build/pc_test_steering.exe \
 *       test/host_steering_test.c Core/Src/driver/steering.c
 *
 * 验证重点（对应 doc/舵机代码结构对照分析.md §4 钳位规则）:
 *   ① 器件/机构两道钳位各管各的域：本组件管"输出域行程 [-115,-65]"
 *   ② 限位=配置态、钳位=执行态：set_limit_* 是配置，set/nudge 每次都强制钳位
 *   ③ 钳位在组件内部强制执行 → 绕过调用方也无效
 *   ④ 错误通道：非法值"钳位保底 + 返回非 OK"，绝不静默
 *   ⑤ 域换算：输出域绝对角 → 物理角（+安装偏置 135）
 *   ⑥ 输出经函数指针注入 → 本桩能编译即证明组件零硬件依赖
 */
#include <stdio.h>
#include "driver/steering.h"

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

/* ---- 输出桩：记录每次下发（模拟"桥接层 → 舵机"）---- */
static int   g_out_cnt   = 0;
static int   g_out_id    = -1;
static float g_out_angle = 0.0f;
static int   g_out_bad_id = 0;

static void stub_output(uint8_t id, float phys_angle)
{
    g_out_cnt++;
    g_out_id = (int)id;
    g_out_angle = phys_angle;
    if (id != 0) g_out_bad_id++;
}

/* 本项目实车标定值（SS2 手动探边，见 doc/舵机转向设计文档.md §2） */
static const steering_cfg_t CFG = {
    .lim_min     = -115.0f,
    .lim_max     = -65.0f,
    .center      = -90.0f,
    .phys_offset = 135.0f,
    .lim_abs     = 135.0f,
    .servo_id    = 0,
};

int main(void)
{
    printf("=== steering 执行组件 PC 测试桩 ===\n");

    /* ---- 组 1: 初始化与错误通道 ---- */
    check(steering_init(0, stub_output) == STEERING_ERR_BAD_ARG, "init(cfg=NULL) -> BAD_ARG");
    check(steering_init(&CFG, 0) == STEERING_ERR_BAD_ARG,        "init(out=NULL) -> BAD_ARG");
    check(steering_get() == 0.0f,                                "未初始化时 get() 不产生垃圾值");

    steering_cfg_t bad = CFG; bad.lim_abs = 0.0f;
    check(steering_init(&bad, stub_output) == STEERING_ERR_BAD_CFG, "init(lim_abs=0) -> BAD_CFG");
    bad = CFG; bad.lim_min = -60.0f;   /* min > max */
    check(steering_init(&bad, stub_output) == STEERING_ERR_BAD_CFG, "init(min>max) -> BAD_CFG");
    bad = CFG; bad.lim_max = 200.0f;   /* 超 lim_abs */
    check(steering_init(&bad, stub_output) == STEERING_ERR_BAD_CFG, "init(max>lim_abs) -> BAD_CFG");
    check(!steering_is_init(), "非法配置后组件仍未初始化");

    /* ---- 组 2: 正常初始化 + 上电归位 ---- */
    g_out_cnt = 0;
    check(steering_init(&CFG, stub_output) == STEERING_OK, "init(合法配置) -> OK");
    check(steering_is_init(),                             "is_init() == 1");
    check(g_out_cnt == 1,                                 "init 后自动下发一次(上电归位, 安全铁律)");
    check(feq(steering_get(), -90.0f),                    "归位角 = 直行位 -90 (输出域)");
    check(feq(g_out_angle, 45.0f),                        "域换算: -90 + 135 = 物理角 45.0");
    check(g_out_id == 0 && g_out_bad_id == 0,             "servo_id 正确传递");

    /* ---- 组 3: 移动程钳位(组件层, 输出域 [-115,-65]) ---- */
    g_out_cnt = 0;
    check(steering_set(0.0f) == STEERING_OK,  "set(0) 不报错(越界属正常钳位语义)");
    check(feq(steering_get(), -65.0f),        "set(0) -> 钳到上限 -65");
    check(feq(g_out_angle, 70.0f),            "物理角 = -65 + 135 = 70.0");
    check(g_out_cnt == 1,                     "每次 set 下发一次");

    steering_set(-200.0f);
    check(feq(steering_get(), -115.0f),       "set(-200) -> 钳到下限 -115");
    check(feq(g_out_angle, 20.0f),            "物理角 = -115 + 135 = 20.0");

    steering_set(-95.0f);
    check(feq(steering_get(), -95.0f) && feq(g_out_angle, 40.0f), "set(-95) 在程内 -> 物理角 40.0");

    /* ---- 组 4: 微步与回中 ---- */
    steering_nudge(1.0f);
    check(feq(steering_get(), -94.0f),  "nudge(+1) -> -94");
    steering_nudge(-3.0f);
    check(feq(steering_get(), -97.0f),  "nudge(-3) -> -97");
    steering_center();
    check(feq(steering_get(), -90.0f) && feq(g_out_angle, 45.0f), "center() -> 直行位 45.0");

    /* ---- 组 5: 限位=配置态（收窄后立即生效 + 归一化 + 非法报告） ---- */
    check(steering_set_limit_max(-70.0f) == STEERING_OK, "set_limit_max(-70) -> OK");
    steering_set(-65.0f);
    check(feq(steering_get(), -70.0f),   "收窄上限后 set(-65) -> 钳到新上限 -70");

    check(steering_set_limit_max(-200.0f) == STEERING_ERR_BAD_ARG, "set_limit_max(-200) 超限 -> BAD_ARG(已保底)");
    check(feq(steering_get_lim_min(), -135.0f) && feq(steering_get_lim_max(), -115.0f),
          "超限配置被钳并归一化 -> [-135, -115]");

    check(steering_set_limit_min(50.0f) == STEERING_OK, "set_limit_min(50) -> OK");
    check(feq(steering_get_lim_min(), -115.0f) && feq(steering_get_lim_max(), 50.0f),
          "min>max 自动交换 -> [-115, 50]");

    check(steering_set_center(10.0f) == STEERING_OK,  "set_center(10) 在程内 -> OK");
    check(steering_set_center(100.0f) == STEERING_ERR_BAD_ARG, "set_center(100) 超上限 -> BAD_ARG");
    check(feq(steering_get(), -115.0f),               "限位收窄后当前角被拉回区间(钳位在组件内部)");

    /* ---- 组 6: 未初始化保护（重新起一个"未初始化"状态无法构造，
     *            故用"组件在 init 前调用"的语义已验证于组 1 ---- */
    check(g_out_bad_id == 0, "全程 servo_id 均为配置值");

    printf("\n=== 结果: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
