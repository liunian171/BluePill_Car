/**
 * @file    txt_cmd.h
 * @brief   文本命令解析 — 纯逻辑模块, 不依赖 HAL/stdio (PC 测试桩可编译)
 *
 * 背景 (2026-09-09 调试): newlib-nano (nano.specs) 下 sscanf "%f" 需要
 * -u _scanf_float 否则静默失败, 导致 "M0 60" 等命令无响应。
 * 本模块用手写解析替代 scanf %f, 同时把命令识别逻辑从 main.c 剥离,
 * 使其可在 PC 上用 gcc 直接做单元测试 (AGENTS.md §5.3 测试桩规范)。
 *
 * 支持的命令 (与原 main.c 分发链语义一致):
 *   PING              心跳
 *   STOP              双轮急停
 *   M0 <rpm> / M1 <rpm>   电机转速 (支持负数/小数/双空格/显式+号)
 *   MS <rpm>          双电机同步转速 (M0=M1)
 *   B0 / B1           单轮刹车
 *   P<id> <kp100> <ki100> <kd100>   单电机 PID (id 0/1)
 *   <kp100> <ki100> <kd100>         双电机 PID (纯 3 整数)
 *   S                 交换 M0/M1 PID 参数
 *   L 0/1             巡线使能
 *   LA 0/1            巡线自动启动
 *   GK/GD/GS <float>  巡线 Kp/Kd/速度
 *   GI                灰度反转
 *   GC/GT <int>       直行/转弯编码器计数标定 (>0)
 *   E<id> <ppr>       编码器 PPR (id 0/1, ppr>0)
 *   SV <deg>          舵机转向角 (输出域绝对角, 直行 = -90)
 *   SV+ / SV-         舵机微步 (±1°)
 *   SL/SR/SC <deg>    左限/右限/直行位标定 (RAM 生效)
 *   STEP <id> <u> <t> 开环阶跃测试 (id 0/1, u=千分比±1000, t=时长ms 100~5000;
 *                     数值范围由 main 分发层校验, 语法 + id 域在此校验)
 *   TEL 0/1           闭环遥测开关 (10Hz CSV)
 */

#ifndef __TXT_CMD_H__
#define __TXT_CMD_H__

typedef enum {
    TXTCMD_NONE = 0,
    TXTCMD_PING,        /* 无参数 */
    TXTCMD_STOP,        /* 无参数 */
    TXTCMD_MOTOR,       /* i0=id, f0=rpm (带符号) */
    TXTCMD_MOTOR_BOTH,  /* f0=rpm (带符号), M0/M1 同步 */
    TXTCMD_BRAKE,       /* i0=id */
    TXTCMD_PID_SET,     /* i0=id(-1=双电机), i1=kp100, i2=ki100, i3=kd100 */
    TXTCMD_PID_SWAP,    /* 无参数 */
    TXTCMD_LINE_EN,     /* i0=0/1 */
    TXTCMD_LINE_AUTO,   /* i0=0/1 */
    TXTCMD_GK,          /* f0 */
    TXTCMD_GD,          /* f0 */
    TXTCMD_GS,          /* f0 */
    TXTCMD_GI,          /* 无参数 */
    TXTCMD_GC,          /* i0>0 */
    TXTCMD_GT,          /* i0>0 */
    TXTCMD_PPR,         /* i0=id, i1=ppr>0 */

    /* ---- 舵机转向 (输出域绝对角: 直行 = -90, clamp [-115, -65]) ----
     * 语义修订 2026-09-09: 命令 = 输出域绝对角, SC 不叠加 (详 舵机转向设计文档 §3) */
    TXTCMD_SERVO,       /* f0=角度(°) 输出域绝对角 */
    TXTCMD_SERVO_NUDGE, /* i0=+1/-1 (微步) */
    TXTCMD_SERVO_LIM,   /* i0: 0=SL左限 1=SR右限 2=SC中位修正, f0=值 */

    /* ---- 调参工具链 (可开关, 默认关) ---- */
    TXTCMD_STEP,        /* i0=id(0/1), i1=rate_0E3(±1000), i2=时长ms, i3=遥测分频 — 开环阶跃测试 */
    TXTCMD_TEL,         /* i0=0/1 — 闭环遥测开关 (10Hz CSV) */
    TXTCMD_FF,          /* i0=前馈系数×100 (0~500, 辨识值 1/K≈100) — 速度环前馈在线整定 */
    TXTCMD_REC,         /* i0=0/1 — 机内记录开关 (20Hz 写 RAM, 对抗无线丢行) */
    TXTCMD_DUMP,        /* 无参数 — 重放机内记录 (与 TEL 同格式, 可重复执行补全) */
} TxtCmdType;

typedef struct {
    TxtCmdType type;
    int   i0, i1, i2, i3;
    float f0;
} TxtCmd;

/**
 * @brief  解析一行文本命令 (不含行尾, \r 应由调用方过滤)
 * @param  s   命令字符串
 * @param  out 输出解析结果 (识别成功时填充)
 * @return 1=识别成功, 0=不识别 (out.type == TXTCMD_NONE)
 */
int txt_cmd_parse(const char *s, TxtCmd *out);

#endif /* __TXT_CMD_H__ */
