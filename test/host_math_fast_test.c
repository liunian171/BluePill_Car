/**
 * @file    host_math_fast_test.c
 * @brief   math_fast（紧凑 sinf/cosf）PC 测试桩 (AGENTS.md §5.3 义务 7)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc -o build/pc_test_math_fast.exe \
 *       test/host_math_fast_test.c Core/Src/common/math_fast.c -lm
 *
 * 为何必须验证精度（2026-09-19 Flash 瘦身批次）:
 *   为省 ~3.8KB Flash，用自实现 sinf/cosf 替换 newlib 全量实现（后者链入
 *   __kernel_rem_pio2f/two_over_pi 等 4.1KB 范围规约链）。**替代算法必须自证
 *   够用**——本桩把自实现与宿主 libm 的**双精度**参考逐点比对（用 sin()/cos()
 *   而非 sinf()/cosf()，避免与自实现符号冲突），锁定精度等级。
 *
 * 验证重点:
 *   ① 本车调用域（|x| ≤ 2π，含半角 ±45°、Δθ ±180°）：误差 < 1e-6
 *   ② 扩展域（|x| ≤ 8π）与象限边界点：仍 < 1e-5（float 量级）
 *   ③ 大角（|x| 达 1e4 rad）：规约仍可用（误差放宽到 1e-3，本车不用此域）
 *   ④ sin²+cos² = 1（同一 x 内自洽性，检验象限换算不错位）
 *   ⑤ 单调性/奇偶性：sin(-x) = -sin(x)、cos(-x) = cos(x)
 */
#include <stdio.h>
#include <math.h>
#include "common/math_fast.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static double fabs_d(double v) { return (v < 0.0) ? -v : v; }

/* 在 [lo, hi] 上以 step 密度逐点比对，返回最大绝对误差 */
static double sweep(double lo, double hi, double step, double *worst_x)
{
    double maxerr = 0.0;
    *worst_x = 0.0;
    for (double x = lo; x <= hi; x += step) {
        float  xf = (float)x;
        double es = fabs_d((double)sinf(xf) - sin(x));
        double ec = fabs_d((double)cosf(xf) - cos(x));
        double e  = (es > ec) ? es : ec;
        if (e > maxerr) { maxerr = e; *worst_x = x; }
    }
    return maxerr;
}

int main(void)
{
    double wx = 0.0, e;

    printf("== math_fast 测试桩（自实现 sinf/cosf vs 宿主 libm 双精度参考）==\n");

    /* ---- ① 本车调用域：±2π ---- */
    e = sweep(-6.2831853072, 6.2831853072, 0.0005, &wx);
    printf("      调用域 |x|<=2π: 最大误差 %.3e (at x=%.6f)\n", e, wx);
    check(e < 1e-6, "① 本车调用域 |x|<=2π 误差 < 1e-6");

    /* ---- ② 扩展域：±8π ---- */
    e = sweep(-25.132741229, 25.132741229, 0.001, &wx);
    printf("      扩展域 |x|<=8π: 最大误差 %.3e (at x=%.6f)\n", e, wx);
    check(e < 1e-5, "② 扩展域 |x|<=8π 误差 < 1e-5");

    /* ---- ③ 象限边界点（π/2 的整数倍及邻域）---- */
    {
        const double kPio2 = 1.5707963267948966;
        int ok = 1;
        double worst = 0.0;
        for (int k = -12; k <= 12; k++) {
            double xs[3] = { k * kPio2, k * kPio2 - 1e-6, k * kPio2 + 1e-6 };
            for (int j = 0; j < 3; j++) {
                float  xf = (float)xs[j];
                double es = fabs_d((double)sinf(xf) - sin(xs[j]));
                double ec = fabs_d((double)cosf(xf) - cos(xs[j]));
                double m  = (es > ec) ? es : ec;
                if (m > worst) worst = m;
                if (m > 1e-5) ok = 0;
            }
        }
        printf("      象限边界点: 最大误差 %.3e\n", worst);
        check(ok, "③ π/2 整数倍 ±1e-6 邻域误差 < 1e-5（规约符号/象限正确）");
    }

    /* ---- ④ sin²+cos² = 1（自洽）---- */
    {
        int ok = 1;
        double worst = 0.0;
        for (double x = -6.2831853072; x <= 6.2831853072; x += 0.001) {
            float xf = (float)x;
            float s = sinf(xf), c = cosf(xf);
            double d = fabs_d((double)s * s + (double)c * c - 1.0);
            if (d > worst) worst = d;
            if (d > 1e-5) ok = 0;
        }
        printf("      sin²+cos²-1: 最大偏差 %.3e\n", worst);
        check(ok, "④ sin²+cos² ≈ 1（象限换算自洽）");
    }

    /* ---- ⑤ 奇偶性 ---- */
    {
        int ok = 1;
        for (double x = 0.001; x <= 6.2831853072; x += 0.01) {
            float xf = (float)x;
            if (fabs_d((double)sinf(-xf) + (double)sinf(xf)) > 1e-6) ok = 0;
            if (fabs_d((double)cosf(-xf) - (double)cosf(xf)) > 1e-6) ok = 0;
        }
        check(ok, "⑤ sin 奇函数 / cos 偶函数成立");
    }

    /* ---- ⑥ 大角域（本车不用，登记可用性）---- */
    e = sweep(-10000.0, 10000.0, 3.7, &wx);
    printf("      大角域 |x|<=1e4: 最大误差 %.3e (at x=%.4f)\n", e, wx);
    check(e < 1e-3, "⑥ 大角 |x|<=1e4 rad 误差 < 1e-3（float 规约精度退化符合预期）");

    /* ---- ⑦ 关键实际用例：odom 中点积分 / IMU 半角 ---- */
    {
        int ok = 1;
        /* Δθ/2 ∈ ±90°、半角 ∈ ±45° 的度数域 */
        for (float deg = -90.0f; deg <= 90.0f; deg += 0.25f) {
            float rad = deg * 0.01745329252f;
            if (fabs_d((double)cosf(rad) - cos((double)rad)) > 1e-6) ok = 0;
            if (fabs_d((double)sinf(rad) - sin((double)rad)) > 1e-6) ok = 0;
        }
        check(ok, "⑦ odom/IMU 实际角度域（±90° 步进 0.25°）误差 < 1e-6");
    }

    printf("\n== math_fast 测试桩: %d PASS / %d FAIL ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
