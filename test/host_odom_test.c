/**
 * @file    host_odom_test.c
 * @brief   odom 感知组件 PC 测试桩 (AGENTS.md §5.3 义务 7: PC 桩先行)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/driver -o build/pc_test_odom.exe \
 *       test/host_odom_test.c Core/Src/driver/odom.c -lm
 *
 * 验证重点（对应 odom.h 坐标域声明 与 2026-09-14 上位机需求）:
 *   ① 差速运动学：直线 ΔX=Δs/ΔY=0；原地转 ΔX=ΔY=0/Δθ 正确
 *   ② 回绕校正：16 位计数翻转不产生巨幅假位移
 *   ③ 符号镜像：sign 注入后倒车 = 负增量（与 speed_loop fb_sign 同一硬件事实）
 *   ④ IMU yaw 差分回绕（-180/180 穿越不产生 ±360 跳变）
 *   ⑤ 中点积分：前进中转弯，ΔX/ΔY 与手算一致（精度 1e-2）
 *   ⑥ dt 守卫：0ms / 超界 → ERR + 重同步（丢帧不污染）
 *   ⑦ 错误通道：未 init / NULL / 非法 cfg 拒绝，绝不静默
 */
#include <stdio.h>
#include <math.h>
#include "driver/odom.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static int feq(float a, float b, float tol)
{
    float d = a - b;
    if (d < 0) d = -d;
    return d <= tol;
}

/* ---- 编码器桩：可编程计数 ---- */
static int32_t g_enc_stub[2] = {0, 0};
static float   g_yaw_stub = 0.0f;         /* ° */

static int32_t stub_enc(uint8_t id) { return g_enc_stub[id]; }
static void stub_yaw(float *out)    { *out = g_yaw_stub; }

/* 本车实车标定值（README 权威: PPR=1466 实测 / 轮周长 205mm / 轮距 ⚠️ 待标定占位） */
static odom_cfg_t base_cfg(void)
{
    odom_cfg_t c;
    c.ppr[0] = 1466.0f; c.ppr[1] = 1466.0f;
    c.wheel_circ_mm = 205.0f;
    c.wheel_track_mm = 170.0f;            /* 仅编码器差分模式用, 占位值 */
    c.enc_span = 65536;
    c.sign[0] = 1; c.sign[1] = -1;        /* 镜像 g_spd_cfg.fb_sign */
    c.yaw_sign = 1;                       /* 默认同向; 反向装引用例单独覆盖 */
    return c;
}

int main(void)
{
    odom_delta_t d;

    /* ---- ⑦ 错误通道 ---- */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        check(odom_update(100, &d) == ODOM_ERR_NOT_INIT, "未 init 调 update → NOT_INIT");
        check(odom_init(NULL, &io) == ODOM_ERR_BAD_ARG, "cfg=NULL → BAD_ARG");
        check(odom_init(&c, NULL) == ODOM_ERR_BAD_ARG, "io=NULL → BAD_ARG");
        c.ppr[0] = -1;
        check(odom_init(&c, &io) == ODOM_ERR_BAD_CFG, "ppr<=0 → BAD_CFG");
        c = base_cfg(); c.sign[1] = 0;
        check(odom_init(&c, &io) == ODOM_ERR_BAD_CFG, "sign=0 → BAD_CFG");
        c = base_cfg(); c.wheel_circ_mm = 0;
        check(odom_init(&c, &io) == ODOM_ERR_BAD_CFG, "circ<=0 → BAD_CFG");
        c = base_cfg(); c.yaw_sign = 0;
        check(odom_init(&c, &io) == ODOM_ERR_BAD_CFG, "yaw_sign 非±1 → BAD_CFG");
        /* 编码器差分模式必须有轮距 */
        c = base_cfg(); c.wheel_track_mm = 0;
        odom_io_t io2 = { .read_enc = stub_enc, .get_yaw = NULL };
        check(odom_init(&c, &io2) == ODOM_ERR_BAD_CFG, "差分模式 track<=0 → BAD_CFG");
    }

    /* ---- ① 直线前进（IMU yaw 模式）----
     * 左轮 +200 计数、右轮 -200 计数(×sign-1 = +200) → 双轮等速前进
     * 位移 = 200/1466×205 = 27.967mm → ΔX=27.967, ΔY=0, Δθ=0 */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        g_enc_stub[0] = 1000; g_enc_stub[1] = 1000; g_yaw_stub = 0.0f;
        check(odom_init(&c, &io) == ODOM_OK, "init OK");
        check(odom_resync(1000) == ODOM_OK, "resync OK");
        g_enc_stub[0] = 1200; g_enc_stub[1] = 800;   /* 左+200, 右-200(sign-1→+200) */
        check(odom_update(1050, &d) == ODOM_OK, "直线帧 OK");
        check(feq(d.dx_mm, 27.967f, 0.01f), "直线 ΔX = 27.967mm");
        check(feq(d.dy_mm, 0.0f, 0.01f), "直线 ΔY = 0");
        check(feq(d.dtheta_deg, 0.0f, 0.01f), "直线 Δθ = 0");
        check(d.dt_ms == 50, "dt = 50ms");
    }

    /* ---- ③ 符号镜像：倒车 = 负增量（若 sign 注入丢失, 此处必 FAIL） ---- */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        g_enc_stub[0] = 500; g_enc_stub[1] = 500;
        odom_init(&c, &io); odom_resync(0);
        g_enc_stub[0] = 400; g_enc_stub[1] = 600;    /* 左-100, 右+100(sign-1→-100) */
        odom_update(50, &d);
        check(feq(d.dx_mm, -13.984f, 0.01f), "倒车 ΔX = -13.984mm");
    }

    /* ---- ④ IMU yaw 差分 + 回绕（179 → -179 应为 +2°, 而非 -358°） ---- */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        g_enc_stub[0] = 0; g_enc_stub[1] = 0; g_yaw_stub = 179.0f;
        odom_init(&c, &io); odom_resync(0);
        g_yaw_stub = -179.0f;
        odom_update(50, &d);
        check(feq(d.dtheta_deg, 2.0f, 0.001f), "yaw 回绕: Δθ = +2°");
    }

    /* ---- ④b yaw_sign = -1（本车 MPU6050 安装: yaw 顺时针为正, 2026-09-14 真机实测）----
     * IMU yaw +30°(顺时针) → 车体 Δθ 应为 -30°(顺时针为负) */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        c.yaw_sign = -1;
        g_enc_stub[0] = 0; g_enc_stub[1] = 0; g_yaw_stub = 10.0f;
        odom_init(&c, &io); odom_resync(0);
        g_yaw_stub = 40.0f;
        odom_update(50, &d);
        check(feq(d.dtheta_deg, -30.0f, 0.001f), "yaw_sign=-1: yaw+30° → Δθ = -30°");
    }

    /* ---- ② 编码器回绕：计数从 65530 → 10（真实增量 +16, 校正避免 -65520） ---- */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        g_enc_stub[0] = 65530; g_enc_stub[1] = 0; g_yaw_stub = 0.0f;
        odom_init(&c, &io); odom_resync(0);
        g_enc_stub[0] = 10; g_enc_stub[1] = 0;       /* 左 +16, 右 0 */
        odom_update(50, &d);
        /* 单侧位移 = 16/1466×205 = 2.238mm, ds = (2.238+0)/2 = 1.119 */
        check(feq(d.dx_mm, 1.119f, 0.01f), "编码器回绕: ΔX = 1.119mm");
    }

    /* ---- ⑤ 中点积分：边走边转 90° ----
     * 双轮各走 ~50mm → Δs≈50mm, Δθ=+90°(yaw 驱动)
     * → ΔX = 50·cos45° = 35.355, ΔY = 50·sin45° = 35.355 */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        int32_t cnt = (int32_t)(50.0f / 205.0f * 1466.0f);   /* 每轮 50mm ≈ 357 */
        g_enc_stub[0] = 0; g_enc_stub[1] = 0; g_yaw_stub = 0.0f;
        odom_init(&c, &io); odom_resync(0);
        g_enc_stub[0] = cnt; g_enc_stub[1] = -cnt;
        g_yaw_stub = 90.0f;
        odom_update(50, &d);
        check(feq(d.dx_mm, 35.355f, 0.2f), "中点积分 ΔX = 35.36mm");
        check(feq(d.dy_mm, 35.355f, 0.2f), "中点积分 ΔY = 35.36mm");
        check(feq(d.dtheta_deg, 90.0f, 0.001f), "Δθ = 90°");
        float x, y, th;
        odom_get_pose(&x, &y, &th);
        check(feq(th, 90.0f, 0.001f), "累计 θ = 90°");
    }

    /* ---- ⑥ dt 守卫：0ms / 超界 → ERR 且重同步（下帧恢复正常） ---- */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = stub_yaw };
        g_enc_stub[0] = 0; g_enc_stub[1] = 0; g_yaw_stub = 0.0f;
        odom_init(&c, &io); odom_resync(1000);
        check(odom_update(1000, &d) == ODOM_ERR_BAD_ARG, "dt=0 → BAD_ARG");
        check(odom_update(5000, &d) == ODOM_ERR_BAD_ARG, "dt=4000 超界 → BAD_ARG");
        g_enc_stub[0] = 1466; g_enc_stub[1] = 0;      /* 重同步后一整圈: 205mm */
        check(odom_update(5050, &d) == ODOM_OK, "超界后重同步恢复 OK");
        check(feq(d.dx_mm, 102.5f, 0.2f), "重同步后首帧 ΔX = 102.5mm");
    }

    /* ---- 编码器差分回退模式（io.get_yaw = NULL）----
     * 右轮 +100mm 等效位移(dmm[1]=+13.984), 左轮 0 → dr>dl = 左转(CCW)
     * Δθ = 13.984/170 rad = 4.713°, Δs = 6.992 → ΔX ≈ 6.99mm */
    {
        odom_cfg_t c = base_cfg();
        odom_io_t  io = { .read_enc = stub_enc, .get_yaw = NULL };
        g_enc_stub[0] = 0; g_enc_stub[1] = 0;
        check(odom_init(&c, &io) == ODOM_OK, "差分模式 init OK");
        odom_resync(0);
        g_enc_stub[1] = -100;                          /* sign-1 → dr=+13.984mm */
        odom_update(50, &d);
        check(feq(d.dtheta_deg, 4.713f, 0.05f), "差分 Δθ = 4.713° (右轮正=左转)");
        check(feq(d.dx_mm, 6.99f, 0.05f), "差分模式 ΔX = 6.99mm");
    }

    printf("\n== odom 测试桩: %d PASS / %d FAIL ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
