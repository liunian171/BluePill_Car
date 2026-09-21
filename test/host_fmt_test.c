/**
 * @file    host_fmt_test.c
 * @brief   fmt（浮点 → 整数格式化助手）PC 测试桩 (R4 卫生批 / AGENTS.md §5.3 义务 7)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc -o build/pc_test_fmt.exe \
 *       test/host_fmt_test.c Core/Src/common/fmt.c
 *
 * 为何必须验证（R4 卫生批）:
 *   cmd_exec(spd_to_int/dec1)、app_control(fmt_f2)、app_display(disp_spd_to_int)
 *   三处重复实现统一收拢到 common/fmt.h → fmt_spd_to_int / fmt_dec1 / fmt_f2。
 *   铁律 = 零行为变更 → 本桩对三个函数逐点断言边界/进位/饱和, 锁定语义等价。
 */
#include <stdio.h>
#include <string.h>
#include "common/fmt.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

int main(void)
{
    printf("== fmt 浮点格式化助手测试桩 ==\n");

    /* ---- fmt_spd_to_int: 带符号四舍五入 (原 spd_to_int) ---- */
    check(fmt_spd_to_int(0.0f) == 0,            "spd 零 -> 0");
    check(fmt_spd_to_int(1.4f) == 1,            "spd +1.4 -> 1");
    check(fmt_spd_to_int(1.5f) == 2,            "spd +1.5 -> 2 (四舍五入)");
    check(fmt_spd_to_int(2.5f) == 3,            "spd +2.5 -> 3");
    check(fmt_spd_to_int(-1.4f) == -1,          "spd -1.4 -> -1");
    check(fmt_spd_to_int(-1.5f) == -2,          "spd -1.5 -> -2 (负向四舍五入)");
    check(fmt_spd_to_int(-2.5f) == -3,          "spd -2.5 -> -3");
    check(fmt_spd_to_int(123.49f) == 123,       "spd 123.49 -> 123");
    check(fmt_spd_to_int(-0.4f) == 0,           "spd -0.4 -> 0");
    check(fmt_spd_to_int(-0.6f) == -1,          "spd -0.6 -> -1");

    /* ---- fmt_dec1: 十分位进位 + 饱和 9 (原 dec1, 负数先取绝对值) ---- */
    check(fmt_dec1(3.4f) == 4,                  "dec1 3.4 -> 4");
    check(fmt_dec1(3.44f) == 4,                 "dec1 3.44 -> 4");
    check(fmt_dec1(3.46f) == 5,                 "dec1 3.46 -> 5 (进位)");
    check(fmt_dec1(3.999f) == 9,                "dec1 3.999 -> 9 (饱和)");
    check(fmt_dec1(-3.4f) == 4,                 "dec1 -3.4 -> 4 (绝对值)");
    check(fmt_dec1(-3.46f) == 5,                "dec1 -3.46 -> 5 (绝对值)");
    check(fmt_dec1(-3.999f) == 9,               "dec1 -3.999 -> 9");
    check(fmt_dec1(0.0f) == 0,                  "dec1 0 -> 0");
    check(fmt_dec1(-0.05f) == 1,                "dec1 -0.05 -> 1 (取绝对后 0.05*10+0.5=1.0)");
    check(fmt_dec1(127.9f) == 9,                "dec1 127.9 -> 9");
    /* 边界: 进位恰好达 10 -> 饱和 9 (v - (int)v)*10+0.5 上限 .9*10+.5=9.5<10, 实际不会触发,
       但仍需防 (int) 截断后整数部分进位导致的 .99 情形的饱和 */
    check(fmt_dec1(-9.99f) == 9,                "dec1 -9.99 -> 9");

    /* ---- fmt_f2: 两位小数 + 进位 + 负号 + 返回值 (原 app_control fmt_f2) ---- */
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), 1.621f);
        check(strcmp(b, "1.62") == 0 && n == 4, "f2 1.621 -> 1.62, ret=4");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), 1.625f);
        check(strcmp(b, "1.63") == 0 && n == 4, "f2 1.625 -> 1.63 (四舍五入到两位)");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), 1.994f);
        check(strcmp(b, "1.99") == 0 && n == 4, "f2 1.994 -> 1.99");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), 1.9959f);
        check(strcmp(b, "2.00") == 0 && n == 4, "f2 1.9959 -> 2.00 (整数进位)");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), -3.14f);
        check(strcmp(b, "-3.14") == 0 && n == 5, "f2 -3.14 -> -3.14, ret=5");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), -1.996f);
        check(strcmp(b, "-2.00") == 0 && n == 5, "f2 -1.996 -> -2.00 (负整数进位)");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), 0.0f);
        check(strcmp(b, "0.00") == 0 && n == 4, "f2 0 -> 0.00");
    }
    {
        char b[20];
        int n = fmt_f2(b, sizeof(b), -0.5f);
        check(strcmp(b, "-0.50") == 0 && n == 5, "f2 -0.5 -> -0.50");
    }
    {
        char b[6];
        int n = fmt_f2(b, sizeof(b), 123.456f);
        check(strcmp(b, "123.4") == 0 && n == 6, "f2 小缓冲(n=6)截断到 '123.4', ret=6 (snprintf 返回值)");
    }

    printf("\n== fmt 测试桩: %d PASS / %d FAIL ==\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}