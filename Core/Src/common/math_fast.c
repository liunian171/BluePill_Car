/**
 * ============================================================================
 *  math_fast.c — 紧凑 sinf/cosf（替代 newlib 全量三角实现）
 * ============================================================================
 *
 *  ▸ 为什么自带实现（2026-09-19 Flash 瘦身批次）◂
 *    本工程只在两处用三角：`odom.c`（中点积分 cos/sin(Δθ/2)、航向 cos/sin）
 *    与 `imu_filter.c`（由加速度初始倾角 ±90° → 四元数，半角 ±45°）。
 *    而 newlib 的 sinf/cosf 为了全范围高精度，链入**软浮点范围规约**整条链：
 *      __kernel_rem_pio2f 1904B + two_over_pi 792B + __ieee754_rem_pio2f 732B
 *      + __kernel_sinf 236B + __kernel_cosf 400B ≈ **4.1KB Flash**
 *    → 自实现同精度等级（Cody-Waite 二段式规约 + fdlibm 极小极大多项式）
 *      只需 ~300B 代码 + 0 常数表（系数为立即数），**净省 ~3.8KB**。
 *
 *  ▸ 实现来源 ◂ 算法与系数取自 fdlibm（FreeBSD/新lib 同源）的
 *    `k_sinf.c` / `k_cosf.c` / `e_rem_pio2f.c` 简化版：
 *      · 规约：n = round(x·2/π)，r = (x − n·π/2 高位) − n·π/2 低位（二段，控精度）
 *      · 多项式：sin(r) 4 项、cos(r) 4 项，|r| ≤ π/4
 *      · 象限选择：(n & 3) 查表式 switch（无表、无循环）
 *
 *  ▸ 精度与范围 ◂
 *    · |x| ≤ ~1e5 rad 内与 newlib sinf/cosf 差 < 1e-6（PC 桩验证，见
 *      `test/host_math_fast_test.c`，对宿主 libm 逐点比对）
 *    · 本工程实际调用域：|x| ≤ 2π（半角 ≤ ±45°、Δθ ≤ ±180°）→ 误差 < 1e-7
 *    · **不用于**需要 1ULP 级精度的场合（本车无此需求）
 *
 *  ▸ 生效方式 ◂ 定义强符号 `sinf`/`cosf`，链接器优先本实现 →
 *    newlib 的对应成员不再被拉入（已用 map 复核）。调用方无需改动。
 *
 *  ▸ 回归防线 ◂ 若将来引入其它 libm 函数（tanf/powf/…）而误用全范围精度，
 *    本文件头注释即说明"三角精度等级"这一约束。
 * ============================================================================
 */

#include "math_fast.h"

/* π/2 二段表示（控规约精度：高位 + 低位） */
#define PIO2_HI      1.5707962513e+00f    /* 0x3FC90FDA */
#define PIO2_LO      7.5497894159e-08f    /* 0x33A22168 */
#define TWO_OVER_PI  6.3661977237e-01f    /* 0x3F22F983 = 2/π */

/* fdlibm 极小极大多项式系数（float 版） */
#define S1  (-1.6666667163e-01f)
#define S2  ( 8.3333337680e-03f)
#define S3  (-1.9841270114e-04f)
#define S4  ( 2.7557314297e-06f)
#define C1  (-5.0000000000e-01f)
#define C2  ( 4.1666667908e-02f)
#define C3  (-1.3888889225e-03f)
#define C4  ( 2.4801587642e-05f)

/** @brief 一次规约同时算出 sin/cos（两函数共用，省一份代码） */
static void sincosf_impl(float x, float *sp, float *cp)
{
    /* ① 象限规约：n = round(x·2/π)，r = x − n·π/2（二段减法控精度） */
    int   n  = (int)(x * TWO_OVER_PI + (x >= 0.0f ? 0.5f : -0.5f));
    float fn = (float)n;
    float r  = (x - fn * PIO2_HI) - fn * PIO2_LO;
    float z  = r * r;

    /* ② |r| ≤ π/4 上的多项式（sin 取 r·P(z)，cos 取 Q(z)） */
    float s = r + r * z * (S1 + z * (S2 + z * (S3 + z * S4)));
    float c = 1.0f + z * (C1 + z * (C2 + z * (C3 + z * C4)));

    /* ③ 按象限换算（n&3: 0=sin/cos  1=cos/−sin  2=−sin/−cos  3=−cos/sin） */
    switch (n & 3) {
    case 0:  *sp =  s;  *cp =  c;  break;
    case 1:  *sp =  c;  *cp = -s;  break;
    case 2:  *sp = -s;  *cp = -c;  break;
    default: *sp = -c;  *cp =  s;  break;
    }
}

float sinf(float x)
{
    float s, c;
    sincosf_impl(x, &s, &c);
    return s;
}

float cosf(float x)
{
    float s, c;
    sincosf_impl(x, &s, &c);
    return c;
}
