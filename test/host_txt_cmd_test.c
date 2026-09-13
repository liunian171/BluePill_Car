/**
 * @file    host_txt_cmd_test.c
 * @brief   txt_cmd_parse PC 测试桩 (AGENTS.md §5.3: 逻辑与硬件分离 + UNIT_TEST)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/driver -o build/pc_test_txt_cmd.exe \
 *       test/host_txt_cmd_test.c Core/Src/driver/txt_cmd.c
 *
 * 回归背景: newlib-nano 下 sscanf %f 静默失败, "M0 60" 无响应 —
 * 首批用例锁定 MOTOR 小数/负数解析, 防止同类回归。
 */
#include <stdio.h>
#include "driver/txt_cmd.h"

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
    return d < 0.0001f;
}

int main(void)
{
    TxtCmd tc;

    printf("=== txt_cmd_parse PC 测试桩 ===\n");

    /* ---- 回归组: 本次事故场景 (sscanf %%f 失效) ---- */
    check(txt_cmd_parse("M0 60", &tc) && tc.type == TXTCMD_MOTOR &&
          tc.i0 == 0 && feq(tc.f0, 60.0f),                 "M0 60 -> MOTOR id0=60 (回归)");
    check(txt_cmd_parse("M1 -100", &tc) && tc.type == TXTCMD_MOTOR &&
          tc.i0 == 1 && feq(tc.f0, -100.0f),               "M1 -100 负数 (回归)");
    check(txt_cmd_parse("M0 12.5", &tc) && feq(tc.f0, 12.5f), "M0 12.5 小数 (回归)");
    check(txt_cmd_parse("M0  60", &tc) && feq(tc.f0, 60.0f),  "M0  60 双空格");
    check(txt_cmd_parse("M0 +60", &tc) && feq(tc.f0, 60.0f),  "M0 +60 显式正号");

    /* ---- MOTOR 拒绝路径 ---- */
    check(!txt_cmd_parse("M0", &tc),                          "M0 无参数 -> 拒绝");
    check(!txt_cmd_parse("M0 x", &tc),                        "M0 x 非数字 -> 拒绝");
    check(!txt_cmd_parse("M0 6.0.5", &tc),                    "M0 6.0.5 非法小数 -> 拒绝");
    check(!txt_cmd_parse("M2 60", &tc),                       "M2 非法id -> 拒绝");
    check(!txt_cmd_parse("M060", &tc),                        "M060 缺空格 -> 拒绝");

    /* ---- 精确匹配命令 ---- */
    check(txt_cmd_parse("PING", &tc) && tc.type == TXTCMD_PING, "PING");
    check(!txt_cmd_parse("ping", &tc),                        "ping 小写 -> 拒绝 (与原 strcmp 一致)");
    check(txt_cmd_parse("STOP", &tc) && tc.type == TXTCMD_STOP, "STOP");
    check(txt_cmd_parse("B0", &tc) && tc.type == TXTCMD_BRAKE && tc.i0 == 0, "B0");
    check(txt_cmd_parse("B1", &tc) && tc.type == TXTCMD_BRAKE && tc.i0 == 1, "B1");
    check(!txt_cmd_parse("B2", &tc),                          "B2 非法id -> 拒绝");

    /* ---- 双电机同步 (MS) ---- */
    check(txt_cmd_parse("MS 60", &tc) && tc.type == TXTCMD_MOTOR_BOTH && feq(tc.f0, 60.0f), "MS 60 双电机");
    check(txt_cmd_parse("MS -50", &tc) && tc.type == TXTCMD_MOTOR_BOTH && feq(tc.f0, -50.0f), "MS -50 反转");
    check(!txt_cmd_parse("MS", &tc),                          "MS 无参数 -> 拒绝");
    check(!txt_cmd_parse("MS x", &tc),                        "MS x 非数字 -> 拒绝");
    check(txt_cmd_parse("M0 60", &tc) && tc.type == TXTCMD_MOTOR, "M0 60 不受 MS 影响");

    /* ---- PID ---- */
    check(txt_cmd_parse("P0 24 13 20", &tc) && tc.type == TXTCMD_PID_SET &&
          tc.i0 == 0 && tc.i1 == 24 && tc.i2 == 13 && tc.i3 == 20, "P0 24 13 20");
    check(txt_cmd_parse("P1 1 2 3", &tc) && tc.i0 == 1,       "P1 1 2 3");
    check(!txt_cmd_parse("P2 1 2 3", &tc),                    "P2 非法id -> 拒绝");
    check(!txt_cmd_parse("P0 1 2", &tc),                      "P0 缺参数 -> 拒绝");
    check(txt_cmd_parse("24 13 20", &tc) && tc.type == TXTCMD_PID_SET &&
          tc.i0 == -1,                                       "24 13 20 -> 双电机 (i0=-1)");
    check(txt_cmd_parse("S", &tc) && tc.type == TXTCMD_PID_SWAP, "S -> 交换");
    check(!txt_cmd_parse("SX", &tc),                          "SX -> 拒绝 (收紧原 S 前缀行为)");

    /* ---- 巡线参数 ---- */
    check(txt_cmd_parse("L 0", &tc) && tc.type == TXTCMD_LINE_EN && tc.i0 == 0, "L 0");
    check(txt_cmd_parse("L 1", &tc) && tc.i0 == 1,            "L 1");
    check(txt_cmd_parse("LA 1", &tc) && tc.type == TXTCMD_LINE_AUTO &&
          tc.i0 == 1,                                        "LA 1 (LA 优先于 L)");
    check(!txt_cmd_parse("LA", &tc),                          "LA 无参数 -> 拒绝");

    check(txt_cmd_parse("GK 6", &tc) && tc.type == TXTCMD_GK && feq(tc.f0, 6.0f), "GK 6");
    check(txt_cmd_parse("GK 6.5", &tc) && feq(tc.f0, 6.5f),   "GK 6.5 小数");
    check(txt_cmd_parse("GD 2", &tc) && tc.type == TXTCMD_GD, "GD 2");
    check(txt_cmd_parse("GS 30", &tc) && tc.type == TXTCMD_GS && feq(tc.f0, 30.0f), "GS 30");
    check(!txt_cmd_parse("GX 1", &tc),                        "GX 未知子命令 -> 拒绝");
    check(txt_cmd_parse("GI", &tc) && tc.type == TXTCMD_GI,   "GI");
    check(txt_cmd_parse("GC 690", &tc) && tc.type == TXTCMD_GC && tc.i0 == 690, "GC 690");
    check(!txt_cmd_parse("GC 0", &tc),                        "GC 0 -> 拒绝 (需>0, 与原一致)");
    check(txt_cmd_parse("GT 1060", &tc) && tc.type == TXTCMD_GT && tc.i0 == 1060, "GT 1060");

    /* ---- 编码器 PPR ---- */
    check(txt_cmd_parse("E0 1466", &tc) && tc.type == TXTCMD_PPR &&
          tc.i0 == 0 && tc.i1 == 1466,                       "E0 1466");
    check(txt_cmd_parse("E1 1500", &tc) && tc.i0 == 1,        "E1 1500");
    check(!txt_cmd_parse("E2 100", &tc),                      "E2 非法id -> 拒绝");

    /* ---- 舵机转向 (SS1, 居中域) ---- */
    check(txt_cmd_parse("SV 30", &tc) && tc.type == TXTCMD_SERVO && feq(tc.f0, 30.0f), "SV 30 右转");
    check(txt_cmd_parse("SV -30", &tc) && tc.type == TXTCMD_SERVO && feq(tc.f0, -30.0f), "SV -30 左转");
    check(txt_cmd_parse("SV 12.5", &tc) && feq(tc.f0, 12.5f), "SV 12.5 小数");
    check(txt_cmd_parse("SV+", &tc) && tc.type == TXTCMD_SERVO_NUDGE && tc.i0 == 1,  "SV+ 微步右");
    check(txt_cmd_parse("SV-", &tc) && tc.type == TXTCMD_SERVO_NUDGE && tc.i0 == -1, "SV- 微步左");
    check(!txt_cmd_parse("SV", &tc),                          "SV 无参数 -> 拒绝");
    check(!txt_cmd_parse("SV x", &tc),                        "SV x 非数字 -> 拒绝");
    check(txt_cmd_parse("SL -40", &tc) && tc.type == TXTCMD_SERVO_LIM &&
          tc.i0 == 0 && feq(tc.f0, -40.0f),                  "SL -40 左限位");
    check(txt_cmd_parse("SR 40", &tc) && tc.i0 == 1 && feq(tc.f0, 40.0f), "SR 40 右限位");
    check(txt_cmd_parse("SC -2", &tc) && tc.i0 == 2 && feq(tc.f0, -2.0f), "SC -2 中位修正");
    check(!txt_cmd_parse("SL", &tc),                          "SL 无参数 -> 拒绝");
    check(txt_cmd_parse("S", &tc) && tc.type == TXTCMD_PID_SWAP, "S 仍为 PID 交换 (与 SV/S+ 不冲突)");
    check(txt_cmd_parse("STOP", &tc) && tc.type == TXTCMD_STOP, "STOP 仍优先匹配");
    check(!txt_cmd_parse("SX 1", &tc),                        "SX 仍拒绝");

    /* ---- 调参工具链 (STEP 开环阶跃 / TEL 遥测) ---- */
    check(txt_cmd_parse("STEP 0 300 2000", &tc) && tc.type == TXTCMD_STEP &&
          tc.i0 == 0 && tc.i1 == 300 && tc.i2 == 2000,        "STEP 0 300 2000");
    check(txt_cmd_parse("STEP 1 -500 1500", &tc) && tc.i0 == 1 && tc.i1 == -500 &&
          tc.i2 == 1500,                                      "STEP 1 -500 1500 负千分比");
    check(!txt_cmd_parse("STEP 2 300 2000", &tc),             "STEP 非法id -> 拒绝");
    check(!txt_cmd_parse("STEP 0 300", &tc),                  "STEP 缺参数 -> 拒绝");
    check(!txt_cmd_parse("STEP 0 300 x", &tc),                "STEP 非数字 -> 拒绝");
    check(!txt_cmd_parse("STEPX 0 1 2", &tc),                 "STEPX 仍拒绝");
    check(txt_cmd_parse("TEL 1", &tc) && tc.type == TXTCMD_TEL && tc.i0 == 1, "TEL 1");
    check(txt_cmd_parse("TEL 0", &tc) && tc.type == TXTCMD_TEL && tc.i0 == 0, "TEL 0");
    check(!txt_cmd_parse("TEL", &tc),                         "TEL 无参数 -> 拒绝");
    check(!txt_cmd_parse("TELA 1", &tc),                      "TELA 仍拒绝");

    /* ---- 兜底 ---- */
    check(!txt_cmd_parse("", &tc),                            "空串 -> 拒绝");
    check(!txt_cmd_parse("HELLO", &tc),                       "HELLO -> 拒绝");

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
