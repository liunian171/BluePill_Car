/**
 * ============================================================================
 *  fmt.c — 浮点 → 整数格式化助手实现（归拢说明见 fmt.h）
 * ============================================================================
 *  实现**原样搬移**自:
 *    · app/cmd_exec.c    spd_to_int / dec1
 *    · app/app_control.c fmt_f2
 *    · app/app_display.c disp_spd_to_int（= spd_to_int）
 *  零行为变更; 逐字节语义与搬迁前一致。
 * ============================================================================
 */

#include "common/fmt.h"
#include <stdio.h>    /* snprintf (fmt_f2) */

int fmt_spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

int fmt_dec1(float v)
{
    if (v < 0) v = -v;
    int d = (int)((v - (float)(int)v) * 10.0f + 0.5f);
    return (d > 9) ? 9 : d;
}

int fmt_f2(char *out, int n, float v)
{
    int neg = (v < 0);
    float a = neg ? -v : v;
    int ip = (int)a;
    int fp = (int)((a - (float)ip) * 100.0f + 0.5f);
    if (fp >= 100) { ip += 1; fp -= 100; }
    return snprintf(out, n, "%s%d.%02d", neg ? "-" : "", ip, fp);
}