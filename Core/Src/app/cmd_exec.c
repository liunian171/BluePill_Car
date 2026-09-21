/**
 * ============================================================================
 *  cmd_exec.c — 命令执行器（组装层 · 纯逻辑 · 全函数指针注入）
 * ============================================================================
 *
 *  层位: 组装层命令执行模块。零 HAL、零组件 include（全部经 cmd_exec_io_t
 *        函数指针注入），保 PC 桩可编译。
 *
 *  搬迁自: app_link.c —— exec_text_cmd(32 文本 case) + 消费循环内 7 个二进制
 *          if-else 分支 + 出口助手(ack/tx_raw/cmd_note/dec1/spd_to_int)。
 *          app_link 保留: 双 ringbuf 消费 + 指挥权门控(BUSY) + 帧重同步。
 *
 *  语义承诺: 与搬迁前逐行为等价 —— 应答字符串逐字节一致、命令调用序不变、
 *          C4 override_manual 调用点不变（各手法命令执行前调用一次）。
 *
 *  ▸ 注入面（R2 用户拍板全函数指针注入）◂
 *    组件(speed/line/steer)/宿主(app_control)/桥(imu/mtr)/仲裁(link_arbiter)
 *    的所有调用经 io 函数指针完成；本文件不 include 任何组件/桥头。
 * ============================================================================
 */

#include "app/cmd_exec.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- 出口助手（原 app_link.c ack/tx_raw/cmd_note/line_diag/dec1/spd_to_int 迁入）---- */

/* 带符号浮点 → 四舍五入 int (与 app_control 同实现, 归拢待卫生批 R4) */
static int spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static int dec1(float v)
{
    if (v < 0) v = -v;
    int d = (int)((v - (float)(int)v) * 10.0f + 0.5f);
    return (d > 9) ? 9 : d;
}

/* 文本回应: 发送 + 记录到 OLED 页4 */
static void ack(const cmd_exec_io_t *io, const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        io->send(buf, n, io->ctx);
        io->note_resp(buf, io->ctx);
    }
}

/* 页6 命令记录 */
static void cmd_note(const cmd_exec_io_t *io, const char *fmt, ...)
{
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    io->note_cmd(buf, io->ctx);
}

/* 手动指令统一接管权出口 (C4: 组件调用 → io 注入) */
static void override_manual(const cmd_exec_io_t *io)
{
    io->override_manual(io->ctx);
}

/* ====================================================================== */
/*  文本命令执行（原 app_link.c exec_text_cmd 整体搬迁; 直调改 io 注入）     */
/* ====================================================================== */

void cmd_exec_text(const cmd_exec_io_t *io, const TxtCmd *tc, uint32_t now)
{
    switch (tc->type) {
    case TXTCMD_PING:
        cmd_note(io, "PING->PONG");
        ack(io, "PONG\r\n");
        break;
    case TXTCMD_STOP:
        for (int i = 0; i < 2; i++) (void)io->spd_stop((uint8_t)i, io->ctx);
        /* 急停语义 = 彻底停: 必须同步关巡线+自动启动,
         * 否则 50ms 后状态机重写目标, 车会"复活"(实测踩坑) */
        io->line_enable(0, io->ctx);
        io->line_set_auto(0, io->ctx);
        (void)io->steer_center(io->ctx);   /* 前轮回直行位 (实测 -90, 非 0) */
        /* 调参工具链随急停关闭 (阶跃测试中途=安全终止, 遥测/记录回默认关) */
        io->ctl_stop_all(io->ctx);
        cmd_note(io, "STOP ALL");
        ack(io, "STOP OK LINE OFF\r\n");
        break;
    case TXTCMD_MOTOR: {
        int id = tc->i0;
        float v = tc->f0;
        override_manual(io);   /* 手动指令优先: 退出巡线接管, 否则 50ms 后被覆盖 */
        (void)io->spd_set_target((uint8_t)id, v, io->ctx);  /* 带符号目标 */
        cmd_note(io, "M%d=%dRPM", id, (int)((v >= 0) ? v : -v));
        ack(io, "M%d:%dRPM\r\n", id, (int)(v + ((v < 0) ? -0.5f : 0.5f)));
        break;
    }
    case TXTCMD_MOTOR_BOTH: {
        float v = tc->f0;
        override_manual(io);   /* 手动指令优先 */
        (void)io->spd_set_target(0, v, io->ctx);
        (void)io->spd_set_target(1, v, io->ctx);
        cmd_note(io, "MS=%dRPM", (int)((v >= 0) ? v : -v));
        ack(io, "MS:%dRPM\r\n", (int)(v + ((v < 0) ? -0.5f : 0.5f)));
        break;
    }
    case TXTCMD_BRAKE: {
        int id = tc->i0;
        override_manual(io);   /* 手动指令优先 */
        (void)io->spd_stop((uint8_t)id, io->ctx);
        cmd_note(io, "BRK%d", id);
        ack(io, "M%d:BRAKE\r\n", id);
        break;
    }
    case TXTCMD_PID_SET: {
        int lo = (tc->i0 < 0) ? 0 : tc->i0, hi = (tc->i0 < 0) ? 1 : tc->i0;
        for (int i = lo; i <= hi; i++) {
            (void)io->spd_set_gains((uint8_t)i, tc->i1*0.01f, tc->i2*0.01f, tc->i3*0.01f, io->ctx);
        }
        if (tc->i0 < 0) ack(io, "PID ALL:%d %d %d\r\n", tc->i1, tc->i2, tc->i3);
        else            ack(io, "PID%d:%d %d %d\r\n", tc->i0, tc->i1, tc->i2, tc->i3);
        cmd_note(io, "PID SET");
        break;
    }
    case TXTCMD_PID_SWAP: {
        /* 增益真值从组件读（不再维护 ×100 镜像数组） */
        float kp[2], ki[2], kd[2];
        (void)io->spd_get_gains(0, &kp[0], &ki[0], &kd[0], io->ctx);
        (void)io->spd_get_gains(1, &kp[1], &ki[1], &kd[1], io->ctx);
        (void)io->spd_set_gains(0, kp[1], ki[1], kd[1], io->ctx);
        (void)io->spd_set_gains(1, kp[0], ki[0], kd[0], io->ctx);
        cmd_note(io, "PID SWAP");
        ack(io, "SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
            spd_to_int(kp[1]*100.0f), spd_to_int(ki[1]*100.0f), spd_to_int(kd[1]*100.0f),
            spd_to_int(kp[0]*100.0f), spd_to_int(ki[0]*100.0f), spd_to_int(kd[0]*100.0f));
        break;
    }
    case TXTCMD_LINE_EN: {
        int en = tc->i0 ? 1 : 0;
        io->line_enable((uint8_t)en, io->ctx);
        if (!en) { (void)io->spd_set_target(0, 0.0f, io->ctx);
                   (void)io->spd_set_target(1, 0.0f, io->ctx); }
        cmd_note(io, "LINE %s", en ? "ON" : "OFF");
        ack(io, "LINE:%s\r\n", en ? "ON" : "OFF");
        break;
    }
    case TXTCMD_LINE_AUTO:
        io->line_set_auto((uint8_t)(tc->i0 ? 1 : 0), io->ctx);
        cmd_note(io, "AUTO %d", tc->i0 ? 1 : 0);
        ack(io, "LA:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_GK:
        io->line_set_kp(tc->f0, io->ctx);
        cmd_note(io, "GK=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack(io, "GK:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GD:
        io->line_set_kd(tc->f0, io->ctx);
        cmd_note(io, "GD=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack(io, "GD:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GS:
        io->line_set_speed(tc->f0, io->ctx);
        cmd_note(io, "GS=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack(io, "GS:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GI:
        io->line_invert(io->ctx);
        cmd_note(io, "GI");
        ack(io, "GI:%d\r\n", io->line_inverted(io->ctx));
        break;
    case TXTCMD_GC:
        io->line_set_straight_cnt(tc->i0, io->ctx);
        cmd_note(io, "GC=%d", tc->i0);
        ack(io, "GC:%d\r\n", tc->i0);
        break;
    case TXTCMD_GT:
        io->line_set_turn_cnt(tc->i0, io->ctx);
        cmd_note(io, "GT=%d", tc->i0);
        ack(io, "GT:%d\r\n", tc->i0);
        break;
    case TXTCMD_PPR:
        io->ppr_set((uint8_t)tc->i0, (uint16_t)tc->i1, io->ctx);
        cmd_note(io, "PPR%d=%d", tc->i0, tc->i1);
        ack(io, "PPR%d:%d\r\n", tc->i0, tc->i1);
        break;
    case TXTCMD_SERVO: {
        (void)io->steer_set(tc->f0, io->ctx);
        float a = io->steer_get(io->ctx);   /* 回显实际生效角(钳位后) */
        cmd_note(io, "SV=%d.%d", (int)a, dec1(a));
        ack(io, "SV:%d.%d\r\n", (int)a, dec1(a));
        break;
    }
    case TXTCMD_SERVO_NUDGE: {
        (void)io->steer_nudge((float)tc->i0, io->ctx);
        float a = io->steer_get(io->ctx);
        cmd_note(io, "SV=%d.%d", (int)a, dec1(a));
        ack(io, "SV:%d.%d\r\n", (int)a, dec1(a));
        break;
    }
    case TXTCMD_SERVO_LIM: {
        /* 限位=配置态 (下限<上限自动交换、限幅 ±lim_abs、直行位拉回区间内) */
        steering_ret_t r;
        if      (tc->i0 == 0) r = io->steer_set_limit_min(tc->f0, io->ctx);
        else if (tc->i0 == 1) r = io->steer_set_limit_max(tc->f0, io->ctx);
        else                  r = io->steer_set_center(tc->f0, io->ctx);
        /* 非法值: 组件已钳位保底, 此处回显请求值并标记 BAD (义务 6) */
        cmd_note(io, "S%c=%d.%d%s", "LRC"[tc->i0], (int)tc->f0, dec1(tc->f0),
                 (r == STEERING_OK) ? "" : "!");
        ack(io, "S%c:%d.%d%s\r\n", "LRC"[tc->i0], (int)tc->f0, dec1(tc->f0),
            (r == STEERING_OK) ? "" : " BAD");
        break;
    }
    case TXTCMD_STEP: {
        bridge_ret_t sr = io->ctl_step_start(
            (uint8_t)tc->i0, (int16_t)tc->i1,
            (uint32_t)tc->i2, (uint8_t)tc->i3, now, io->ctx);
        if (sr == BRIDGE_OK) {
            cmd_note(io, "STEP%d %d", tc->i0, tc->i1);
            ack(io, "STEP GO %d %d %d %d\r\n", tc->i0, tc->i1, tc->i2, tc->i3);
        } else if (sr == BRIDGE_ERR_BUSY) {
            ack(io, "STEP BUSY\r\n"); cmd_note(io, "STEP BUSY");
        } else {
            ack(io, "STEP BAD\r\n"); cmd_note(io, "STEP BAD");
        }
        break;
    }
    case TXTCMD_TEL:
        io->ctl_tel_set((uint8_t)tc->i0, io->ctx);
        cmd_note(io, "TEL %d", tc->i0 ? 1 : 0);
        ack(io, "TEL:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_FF:
        if (tc->i0 < 0 || tc->i0 > 500) { ack(io, "FF BAD\r\n"); cmd_note(io, "FF BAD"); break; }
        for (int i = 0; i < 2; i++) (void)io->spd_set_ff_gain((uint8_t)i, tc->i0 / 100.0f, io->ctx);
        cmd_note(io, "FF %d", tc->i0);
        ack(io, "FF:%d.%02d\r\n", tc->i0 / 100, tc->i0 % 100);
        break;
    case TXTCMD_REC:
        io->ctl_rec_set((uint8_t)tc->i0, io->ctx);
        cmd_note(io, "REC %d", tc->i0 ? 1 : 0);
        ack(io, "REC:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_DUMP:
        /* 重放输出/延时/收尾在宿主 app_control 内 (ctl_rec_dump) */
        io->ctl_rec_dump(io->ctx);
        break;
    case TXTCMD_ODOM:
        io->ctl_odom_set((uint8_t)tc->i0, io->ctx);
        cmd_note(io, "ODOM %d", tc->i0 ? 1 : 0);
        ack(io, "ODOM:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_WD:
        /* 范围 0~60000ms, 0=关 (校验在 app_control); 使能时刷新喂狗时刻 */
        if (io->ctl_wd_set((uint32_t)tc->i0, io->ctx) != BRIDGE_OK) {
            ack(io, "WD BAD\r\n"); cmd_note(io, "WD BAD");
        } else {
            io->wd_feed(io->ctx);   /* 使能即刷新喂狗基准 */
            cmd_note(io, "WD %dms", tc->i0);
            ack(io, "WD:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_ITEL:
        if (io->ctl_itel_set((uint8_t)tc->i0, io->ctx) != BRIDGE_OK) {
            ack(io, "ITEL BAD\r\n"); cmd_note(io, "ITEL BAD");
        } else {
            cmd_note(io, "ITEL %d", tc->i0);
            ack(io, "ITEL:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_IGAIN:
        /* Mahony 增益 ×100 (RAM 生效); 允许 0 0 = 纯陀螺积分 (E2) */
        if (tc->i0 < 0 || tc->i1 < 0) { ack(io, "IGAIN BAD\r\n"); cmd_note(io, "IGAIN BAD"); break; }
        (void)io->imu_set_mahony_gains(0, tc->i0 / 100.0f, tc->i1 / 100.0f, io->ctx);
        cmd_note(io, "IGAIN %d %d", tc->i0, tc->i1);
        ack(io, "IGAIN:%d %d\r\n", tc->i0, tc->i1);
        break;
    case TXTCMD_IDRIFT:
        if (tc->i0 < 0 || tc->i0 > 1) { ack(io, "IDRIFT BAD\r\n"); cmd_note(io, "IDRIFT BAD"); break; }
        (void)io->imu_set_drift_enable(0, (uint8_t)tc->i0, io->ctx);
        cmd_note(io, "IDRIFT %d", tc->i0);
        ack(io, "IDRIFT:%d\r\n", tc->i0);
        break;
    case TXTCMD_IRATE:
        if (io->ctl_imu_rate_set((uint16_t)tc->i0, io->ctx) != BRIDGE_OK) {
            ack(io, "IRATE BAD\r\n"); cmd_note(io, "IRATE BAD");
        } else {
            cmd_note(io, "IRATE %d", tc->i0);
            ack(io, "IRATE:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_ICAL:
        /* 前置: 车体静止水平; 之后 50 拍(默认5s)校准, 完成后 yaw 归零 */
        if (io->imu_recalibrate(0, io->ctx) != BRIDGE_OK) { ack(io, "ICAL BAD\r\n"); cmd_note(io, "ICAL BAD"); break; }
        cmd_note(io, "ICAL GO");
        ack(io, "ICAL:GO\r\n");
        break;
    case TXTCMD_LINK:
        /* [P2 双链路] 查询/切换 — 任意链路可发 (安全例外, 设计文档 §2.2);
         * 让出/接管/抢回三语义合一: 目标=对方即切换, 目标=自己即无操作 */
        if (tc->i0 < 0) {
            cmd_note(io, "OWNER:%s", (io->arb_owner(io->ctx) == LINK_ARB_USB) ? "USB" : "UART");
            ack(io, "OWNER:%s\r\n", (io->arb_owner(io->ctx) == LINK_ARB_USB) ? "USB" : "UART");
        } else if (io->uart_enabled && io->usb_enabled) {
            LinkArbRet r = io->arb_request((tc->i0 == 1) ? LINK_ARB_UART : LINK_ARB_USB, io->ctx);
            if (r == LINK_ARB_OK) {
                cmd_note(io, "LINK:%s", tc->i0 == 1 ? "UART" : "USB");
                ack(io, "LINK:%s OK\r\n", (tc->i0 == 1) ? "UART" : "USB");
            } else {   /* ERR_SESSION — 会话锁 */
                cmd_note(io, "BUSY SESSION");
                ack(io, "BUSY:SESSION\r\n");
            }
        } else {
            /* [P3 开关收口] 单链路模式: 目标链路编译期不存在 → 明确回 LINK:OFF
             * (既不当成功也不无声无操作, 上位机必须能分辨"切不了") */
            uint8_t fb = io->usb_enabled ? APP_LINK_USB : APP_LINK_UART;
            if (((tc->i0 == 1) ? APP_LINK_UART : APP_LINK_USB) == fb) {
                ack(io, "LINK:%s OK\r\n", tc->i0 == 1 ? "UART" : "USB");
            } else {
                cmd_note(io, "LINK:OFF");
                ack(io, "LINK:OFF\r\n");
            }
        }
        break;
    default:
        break;
    }
    (void)now;
}

/* ====================================================================== */
/*  二进制帧命令执行（原 app_link.c 消费循环内 7 分支 + BUSY 门控; 搬迁  */
/*  仅执行已放行命令; BUSY 门控保留在 app_link）                            */
/* ====================================================================== */

void cmd_exec_bin(const cmd_exec_io_t *io, uint8_t cmd, const uint8_t *data, uint8_t len, uint32_t now)
{
    (void)now;
    /* 与搬迁前逐分支等价（len = flen-2; ff 尾已由 app_link 截除） */
    if      (cmd == 0x01 && len >= 5 && data[0] < 2) { uint8_t id = data[0]; float v; memcpy(&v,&data[1],4); override_manual(io); (void)io->spd_set_target(id, v, io->ctx); cmd_note(io, "M%d=%dRPM", id, (int)((v>0)?v:-v)); ack(io, "M%d:%dRPM\r\n",id,spd_to_int(v)); }
    else if (cmd == 0x02 && len >= 5 && data[0] < 2) { uint8_t id = data[0]; float v; memcpy(&v,&data[1],4); override_manual(io); (void)io->mtr_set_speed_mps(id,v, io->ctx); cmd_note(io, "M%d=%dcm/s", id, (int)(v*100)); ack(io, "M%d:%dcm/s\r\n",id,spd_to_int(v*100.0f)); }
    else if (cmd == 0x03 && len >= 1 && data[0] < 2) { uint8_t id = data[0]; override_manual(io); (void)io->spd_stop(id, io->ctx); cmd_note(io, "BRK%d", id); ack(io, "M%d:BRAKE\r\n",id); }
    else if (cmd == 0x10 && len >= 5 && data[0] < 1) { float v; memcpy(&v,&data[1],4); (void)io->steer_set(v, io->ctx); float a = io->steer_get(io->ctx); cmd_note(io, "SV=%d.%d", (int)a, dec1(a)); ack(io, "SV:%d.%d\r\n", (int)a, dec1(a)); }
    else if (cmd == 0xE0 && len >= 13 && data[0] < 2) { uint8_t id = data[0]; float kp,ki,kd; memcpy(&kp,&data[1],4); memcpy(&ki,&data[5],4); memcpy(&kd,&data[9],4); (void)io->spd_set_gains(id,kp,ki,kd, io->ctx); cmd_note(io, "PID%d", id); ack(io, "OK\r\n"); }
    else if (cmd == 0x20 && len >= 5 && data[0] < 2) { uint8_t id = data[0]; float v; memcpy(&v,&data[1],4); if (v >= -100.0f && v <= 100.0f) { override_manual(io); (void)io->mtr_set_rate_0E3(id, (int16_t)(v * 10.0f), io->ctx); cmd_note(io, "DUTY%d", id); ack(io, "D%d:%d.%d\r\n", id, (int)v, dec1(v)); } else { cmd_note(io, "DUTY BAD"); ack(io, "?\r\n"); } }
    else if (cmd == 0xF0) { cmd_note(io, "PING->PONG"); ack(io, "PONG\r\n"); }
    else { cmd_note(io, "ERR:%02X", cmd); ack(io, "?\r\n"); }
}