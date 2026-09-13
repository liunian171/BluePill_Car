/**
 * @file    txt_cmd.c
 * @brief   文本命令解析 — 纯逻辑实现 (见 txt_cmd.h)
 *
 * 设计约束:
 *  - 不用 sscanf %f (newlib-nano 缺 _scanf_float 会静默失败, 本次事故根因)
 *  - 手写 float 解析 (符号/整数/小数, 不支持指数, 命令场景足够)
 *  - 整数用 strtol (newlib-nano 原生支持, PC 同源可测)
 */
#include "driver/txt_cmd.h"
#include <string.h>
#include <stdlib.h>

/* 跳过空白 */
static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/* 解析整数 (strtol, 自动跳过前导空白); 成功推进 *p 并返回 1 */
static int parse_int(const char **p, int *out)
{
    const char *s = skip_ws(*p);
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s) return 0;
    *out = (int)v;
    *p = end;
    return 1;
}

/* 解析浮点: [+-]digits[.digits]; 成功推进 *p 并返回 1 (不用 sscanf %f!) */
static int parse_float(const char **p, float *out)
{
    const char *s = skip_ws(*p);
    int neg = 0;
    if (*s == '+') s++;
    else if (*s == '-') { neg = 1; s++; }

    long ip = 0;
    const char *d0 = s;
    while (*s >= '0' && *s <= '9') { ip = ip * 10 + (*s - '0'); s++; }
    if (s == d0) return 0;            /* 无整数部分 */
    float v = (float)ip;

    if (*s == '.') {
        s++;
        float frac = 0.0f, scale = 0.1f;
        const char *d1 = s;
        while (*s >= '0' && *s <= '9') { frac += (*s - '0') * scale; scale *= 0.1f; s++; }
        if (s == d1) return 0;        /* "60.x" 小数点后无数字 */
        v += frac;                    /* 小数部分并入结果 */
    }
    if (*s == '.') return 0;          /* "6.0.5" 多个小数点 -> 拒绝 */
    *out = neg ? -v : v;
    *p = s;
    return 1;
}

int txt_cmd_parse(const char *s, TxtCmd *o)
{
    memset(o, 0, sizeof(*o));
    o->type = TXTCMD_NONE;
    if (s == NULL || *s == '\0') return 0;
    const char *p = s;

    /* ---- 精确匹配命令 (大小写敏感, 与原 strcmp 行为一致) ---- */
    if (strcmp(p, "PING") == 0) { o->type = TXTCMD_PING;     return 1; }
    if (strcmp(p, "STOP") == 0) { o->type = TXTCMD_STOP;     return 1; }
    if (strcmp(p, "S")    == 0) { o->type = TXTCMD_PID_SWAP; return 1; }
    if (strcmp(p, "GI")   == 0) { o->type = TXTCMD_GI;       return 1; }

    /* ---- SV <deg> / SV+ / SV- 与 SL/SR/SC <deg> (舵机转向, 输出域绝对角) ---- */
    if (p[0] == 'S' && (p[1] == 'V' || p[1] == 'L' || p[1] == 'R' || p[1] == 'C')) {
        const char *q = p + 2;
        float v;
        if (p[1] == 'V' && q[0] == '+' && q[1] == '\0') {
            o->type = TXTCMD_SERVO_NUDGE; o->i0 = 1;  return 1;
        }
        if (p[1] == 'V' && q[0] == '-' && q[1] == '\0') {
            o->type = TXTCMD_SERVO_NUDGE; o->i0 = -1; return 1;
        }
        if (!parse_float(&q, &v)) return 0;
        if (p[1] == 'V') {
            o->type = TXTCMD_SERVO;
            o->f0 = v;
        } else {
            o->type = TXTCMD_SERVO_LIM;
            o->i0 = (p[1] == 'L') ? 0 : (p[1] == 'R') ? 1 : 2;
            o->f0 = v;
        }
        return 1;
    }

    /* ---- B0 / B1 (前缀匹配, 与原行为一致) ---- */
    if (p[0] == 'B' && (p[1] == '0' || p[1] == '1')) {
        o->type = TXTCMD_BRAKE;
        o->i0 = p[1] - '0';
        return 1;
    }

    /* ---- STEP <id> <rate_0E3> <dur_ms> (调参: 开环阶跃测试) ----
     * 数值范围校验在 main 分发层 (需 ack 错误码), 此处只做语法 + id 域 */
    if (strncmp(p, "STEP", 4) == 0) {
        const char *q = p + 4;
        int id, u, t;
        if (!parse_int(&q, &id) || id < 0 || id > 1) return 0;
        if (!parse_int(&q, &u) || !parse_int(&q, &t)) return 0;
        o->type = TXTCMD_STEP;
        o->i0 = id; o->i1 = u; o->i2 = t;
        return 1;
    }

    /* ---- TEL 0/1 (调参: 闭环遥测开关) ---- */
    if (strncmp(p, "TEL", 3) == 0) {
        const char *q = p + 3;
        int en;
        if (!parse_int(&q, &en)) return 0;
        o->type = TXTCMD_TEL;
        o->i0 = en;
        return 1;
    }

    /* ---- MS <rpm> (双电机同步, 必须先于 M0/M1 判定? 无冲突, 位置任意) ---- */
    if (p[0] == 'M' && p[1] == 'S' && p[2] == ' ') {
        const char *q = p + 2;
        float v;
        if (!parse_float(&q, &v)) return 0;
        o->type = TXTCMD_MOTOR_BOTH;
        o->f0 = v;
        return 1;
    }

    /* ---- M0/M1 <rpm> (必须空格 + 数字) ---- */
    if (p[0] == 'M' && (p[1] == '0' || p[1] == '1') && p[2] == ' ') {
        const char *q = p + 2;
        float v;
        if (!parse_float(&q, &v)) return 0;
        o->type = TXTCMD_MOTOR;
        o->i0 = p[1] - '0';
        o->f0 = v;
        return 1;
    }

    /* ---- LA 0/1 (必须先于 L 判定) ---- */
    if (p[0] == 'L' && p[1] == 'A') {
        const char *q = p + 2;
        int en;
        if (!parse_int(&q, &en)) return 0;
        o->type = TXTCMD_LINE_AUTO;
        o->i0 = en;
        return 1;
    }

    /* ---- L 0/1 ---- */
    if (p[0] == 'L') {
        const char *q = p + 1;
        int en;
        if (!parse_int(&q, &en)) return 0;
        o->type = TXTCMD_LINE_EN;
        o->i0 = en;
        return 1;
    }

    /* ---- GK/GD/GS <float> 与 GC/GT <int> ---- */
    if (p[0] == 'G') {
        const char *q = p + 2;
        if (p[1] == 'K' || p[1] == 'D' || p[1] == 'S') {
            float v;
            if (!parse_float(&q, &v)) return 0;
            o->type = (p[1] == 'K') ? TXTCMD_GK :
                      (p[1] == 'D') ? TXTCMD_GD : TXTCMD_GS;
            o->f0 = v;
            return 1;
        }
        if (p[1] == 'C' || p[1] == 'T') {
            int c;
            if (!parse_int(&q, &c) || c <= 0) return 0;
            o->type = (p[1] == 'C') ? TXTCMD_GC : TXTCMD_GT;
            o->i0 = c;
            return 1;
        }
        return 0;
    }

    /* ---- P<id> <kp100> <ki100> <kd100> ---- */
    if (p[0] == 'P') {
        const char *q = p + 1;
        int id, a, c, d;
        if (!parse_int(&q, &id) || id < 0 || id > 1) return 0;
        if (!parse_int(&q, &a) || !parse_int(&q, &c) || !parse_int(&q, &d)) return 0;
        o->type = TXTCMD_PID_SET;
        o->i0 = id; o->i1 = a; o->i2 = c; o->i3 = d;
        return 1;
    }

    /* ---- E<id> <ppr> ---- */
    if (p[0] == 'E') {
        const char *q = p + 1;
        int id, ppr;
        if (!parse_int(&q, &id) || id < 0 || id > 1) return 0;
        if (!parse_int(&q, &ppr) || ppr <= 0) return 0;
        o->type = TXTCMD_PPR;
        o->i0 = id; o->i1 = ppr;
        return 1;
    }

    /* ---- 纯 3 整数 = 双电机 PID (兜底, 与原分支顺序一致放最后) ---- */
    {
        const char *q = p;
        int a, c, d;
        if (parse_int(&q, &a) && parse_int(&q, &c) && parse_int(&q, &d)) {
            o->type = TXTCMD_PID_SET;
            o->i0 = -1; o->i1 = a; o->i2 = c; o->i3 = d;
            return 1;
        }
    }

    return 0;
}
