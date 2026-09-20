/**
 * ============================================================================
 *  app_link.c — 命令链宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（时间基准传入; 发送/页记录/接管权经 io 注入）。
 *  搬迁自: main.c 双 ringbuf 消费循环 + 指挥权门控 + 39 个命令分支（文本 32 case
 *          + 二进制 7 分支）+ ack/tx_raw/cmd_note/line_diag 出口 + UART rx 记账。
 *  语义承诺: 与搬迁前逐行为等价（分片预算/门控例外/命令级交织/帧超时重同步不变）。
 *
 *  ▸ P1-8 (2026-09-20 随本批落地) ◂
 *    USB 收包经 usbd_cdc_if_register_rx_sink(app_link_usb_rx_isr) 注入——
 *    usbd_cdc_if.c 不再 extern 直写组装层 ringbuf, 反向依赖消除。
 *
 *  ▸ C4 收敛 (2026-09-20 随本批落地) ◂
 *    11 处散落的 line_follower_enable(0) 收敛为 io.override_manual 注入点——
 *    调用点语义不变（各分支执行前退出巡线）, 但组件依赖解除, PC 桩可验证。
 * ============================================================================
 */

#include "app/app_link.h"
#include "driver/link_arbiter.h"
#include "driver/txt_cmd.h"
#include "driver/speed_loop.h"
#include "driver/motor_bridge.h"
#include "driver/steering.h"
#include "driver/line_follower.h"
#include "driver/encoder.h"
#include "driver/imu_bridge.h"
#include "app/app_control.h"
#include "common/ringbuf.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- 私有状态 ---- */

static app_link_cfg_t s_cfg;
static app_link_io_t  s_io;
static uint8_t        s_inited = 0;

/* 双 ringbuf (原 main.c g_ringbuf_uart/usb 收编; ISR 写/任务读) */
static RingBuffer s_rb_uart;
static RingBuffer s_rb_usb;

/* 按链路独立的解析状态 (原 main.c 主循环局部静态) */
static uint8_t  s_txt[APP_LINK_COUNT][20];
static uint8_t  s_txt_len[APP_LINK_COUNT];
static uint8_t  s_frame[APP_LINK_COUNT][32];
static uint8_t  s_f_len[APP_LINK_COUNT];
static uint8_t  s_in_frame[APP_LINK_COUNT];
static uint32_t s_t_frame[APP_LINK_COUNT];

/* UART rx 记账 (页7 "U:<秒>" 数据源) */
static uint32_t s_uart_last_rx = 0;
static uint8_t  s_uart_seen    = 0;
static uint32_t s_now          = 0;   /* 最近一次 task 的 now (silence 查询用) */

/* ---- 出口助手 (原 main.c ack/tx_raw/cmd_note/line_diag/dec1 迁入) ---- */

/* 带符号浮点 → 四舍五入 int (与 app_control/main.c 同实现, 归拢待卫生批) */
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

/* 裸发送: 不记录 OLED 应答页 (寻迹诊断/遥测等高频事件用) */
static void tx_raw(const char *buf, int n)
{
    s_io.sink.send(buf, n, s_io.sink.ctx);
}

/* 文本回应: 发送 + 记录到 OLED 页4 */
static void ack(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        tx_raw(buf, n);
        s_io.page_note.note_resp(buf, s_io.page_note.ctx);
    }
}

/* 页6 命令记录 */
static void cmd_note(const char *fmt, ...)
{
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s_io.page_note.note_cmd(buf, s_io.page_note.ctx);
}

/* 寻迹诊断代理: 转发 line_follower 事件到串口 (高频, 不占应答页) */
static void line_diag(const char *msg)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", msg);
    if (n > 0) tx_raw(buf, n);
}

/* 手动指令统一接管权出口 (C4: 组件调用 → io 注入) */
static void override_manual(void)
{
    s_io.override_manual(s_io.ctx);
}

/* ---- 生命周期 ---- */

bridge_ret_t app_link_init(const app_link_cfg_t *cfg, const app_link_io_t *io)
{
    if (cfg == NULL || io == NULL) return BRIDGE_ERR_BAD_ARG;
    if (cfg->byte_budget == 0 || cfg->frame_timeout_ms == 0) return BRIDGE_ERR_BAD_CFG;
    if (io->sink.send == NULL || io->page_note.note_cmd == NULL ||
        io->page_note.note_resp == NULL || io->set_active == NULL ||
        io->owner_link == NULL || io->override_manual == NULL ||
        io->wd_feed == NULL || io->session_active == NULL)
        return BRIDGE_ERR_BAD_ARG;
    s_cfg = *cfg;
    s_io  = *io;
    ringbuf_init(&s_rb_uart);
    ringbuf_init(&s_rb_usb);
    memset(s_txt_len, 0, sizeof(s_txt_len));
    memset(s_in_frame, 0, sizeof(s_in_frame));
    memset(s_f_len, 0, sizeof(s_f_len));
    /* 寻迹诊断事件回调在此注册 (命令链外泄通道, 原 main.c init 段职责) */
    line_follower_set_event_cb(line_diag);
    s_inited = 1;
    return BRIDGE_OK;
}

/* ---- ISR 注入（只写 ringbuf, 不解析） ---- */

void app_link_uart_rx_isr(uint8_t byte) { ringbuf_write(&s_rb_uart, byte); }
void app_link_usb_rx_isr(uint8_t byte)  { ringbuf_write(&s_rb_usb, byte); }

/* ---- 只读查询 ---- */

uint16_t app_link_rx_overflow(void)
{
    /* [P3] 禁用链路的 ringbuf 无人消费, 溢出计数不计入 (与 P2 行为一致) */
    uint16_t ovf = (uint16_t)ringbuf_overflow(&s_rb_uart);
    if (s_cfg.usb_enabled)
        ovf = (uint16_t)(ovf + ringbuf_overflow(&s_rb_usb));
    return ovf;
}

uint32_t app_link_uart_silence_s(void)
{
    if (!s_uart_seen) return 0xFFFFFFFFu;   /* 从未收到 (页7 显 "-") */
    return (s_now - s_uart_last_rx) / 1000;
}

/* ---- 命令执行（原 main.c 文本 switch 整体搬迁; 仅接管权出口改注入） ---- */

static void exec_text_cmd(const TxtCmd *tc)
{
    switch (tc->type) {
    case TXTCMD_PING:
        cmd_note("PING->PONG");
        ack("PONG\r\n");
        break;
    case TXTCMD_STOP:
        for (int i = 0; i < 2; i++) speed_loop_stop(i);
        /* 急停语义 = 彻底停: 必须同步关巡线+自动启动,
         * 否则 50ms 后状态机重写目标, 车会"复活"(实测踩坑) */
        line_follower_enable(0);
        line_follower_set_auto(0);
        steering_center();  /* 前轮回直行位 (实测 -90, 非 0) */
        /* 调参工具链随急停关闭 (阶跃测试中途=安全终止, 遥测/记录回默认关)
         * [app_control 拆分] 状态收编后经 stop_all 一口关闭 (看门狗保持武装) */
        app_control_stop_all();
        cmd_note("STOP ALL");
        ack("STOP OK LINE OFF\r\n");
        break;
    case TXTCMD_MOTOR: {
        int id = tc->i0;
        float v = tc->f0;
        override_manual();  /* 手动指令优先: 退出巡线接管, 否则 50ms 后被覆盖 */
        speed_loop_set_target(id, v);   /* 带符号目标: 方向由符号统一表达 */
        cmd_note("M%d=%dRPM", id, (int)((v >= 0) ? v : -v));
        ack("M%d:%dRPM\r\n", id, (int)(v + ((v < 0) ? -0.5f : 0.5f)));
        break;
    }
    case TXTCMD_MOTOR_BOTH: {
        float v = tc->f0;
        override_manual();  /* 手动指令优先 */
        speed_loop_set_target(0, v);
        speed_loop_set_target(1, v);
        cmd_note("MS=%dRPM", (int)((v >= 0) ? v : -v));
        ack("MS:%dRPM\r\n", (int)(v + ((v < 0) ? -0.5f : 0.5f)));
        break;
    }
    case TXTCMD_BRAKE: {
        int id = tc->i0;
        override_manual();  /* 手动指令优先 */
        speed_loop_stop(id);
        cmd_note("BRK%d", id);
        ack("M%d:BRAKE\r\n", id);
        break;
    }
    case TXTCMD_PID_SET: {
        int lo = (tc->i0 < 0) ? 0 : tc->i0, hi = (tc->i0 < 0) ? 1 : tc->i0;
        for (int i = lo; i <= hi; i++) {
            speed_loop_set_gains(i, tc->i1*0.01f, tc->i2*0.01f, tc->i3*0.01f);
        }
        if (tc->i0 < 0) ack("PID ALL:%d %d %d\r\n", tc->i1, tc->i2, tc->i3);
        else            ack("PID%d:%d %d %d\r\n", tc->i0, tc->i1, tc->i2, tc->i3);
        cmd_note("PID SET");
        break;
    }
    case TXTCMD_PID_SWAP: {
        /* 增益真值从组件读（不再维护 ×100 镜像数组 → 消除"二进制改参后镜像失效"的漂移） */
        float kp[2], ki[2], kd[2];
        speed_loop_get_gains(0, &kp[0], &ki[0], &kd[0]);
        speed_loop_get_gains(1, &kp[1], &ki[1], &kd[1]);
        speed_loop_set_gains(0, kp[1], ki[1], kd[1]);
        speed_loop_set_gains(1, kp[0], ki[0], kd[0]);
        cmd_note("PID SWAP");
        ack("SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
            spd_to_int(kp[1]*100.0f), spd_to_int(ki[1]*100.0f), spd_to_int(kd[1]*100.0f),
            spd_to_int(kp[0]*100.0f), spd_to_int(ki[0]*100.0f), spd_to_int(kd[0]*100.0f));
        break;
    }
    case TXTCMD_LINE_EN: {
        int en = tc->i0 ? 1 : 0;
        line_follower_enable(en);
        if (!en) { speed_loop_set_target(0, 0.0f); speed_loop_set_target(1, 0.0f); }
        cmd_note("LINE %s", en ? "ON" : "OFF");
        ack("LINE:%s\r\n", en ? "ON" : "OFF");
        break;
    }
    case TXTCMD_LINE_AUTO:
        line_follower_set_auto(tc->i0 ? 1 : 0);
        cmd_note("AUTO %d", tc->i0 ? 1 : 0);
        ack("LA:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_GK:
        line_follower_set_kp(tc->f0);
        cmd_note("GK=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack("GK:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GD:
        line_follower_set_kd(tc->f0);
        cmd_note("GD=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack("GD:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GS:
        line_follower_set_speed(tc->f0);
        cmd_note("GS=%d.%d", (int)tc->f0, dec1(tc->f0));
        ack("GS:%d.%d\r\n", (int)tc->f0, dec1(tc->f0));
        break;
    case TXTCMD_GI:
        line_follower_invert();
        cmd_note("GI");
        ack("GI:%d\r\n", line_follower_inverted());
        break;
    case TXTCMD_GC:
        line_follower_set_straight_cnt(tc->i0);
        cmd_note("GC=%d", tc->i0);
        ack("GC:%d\r\n", tc->i0);
        break;
    case TXTCMD_GT:
        line_follower_set_turn_cnt(tc->i0);
        cmd_note("GT=%d", tc->i0);
        ack("GT:%d\r\n", tc->i0);
        break;
    case TXTCMD_PPR:
        /* PPR 在线改: 经 app_tx 无关, 编码器句柄属组装层 — 由 io.enc_ppr_set 注入?
         * 现状: 直接调 main.c 绑定不可达 → 经 speed_loop 无此口; 保留 main 职责:
         * 该命令搬迁进 app_link 需句柄, 故经 io 扩展位 (见 app_link_io_t.ppr_set) */
        s_io.ppr_set((uint8_t)tc->i0, (uint16_t)tc->i1, s_io.ctx);
        cmd_note("PPR%d=%d", tc->i0, tc->i1);
        ack("PPR%d:%d\r\n", tc->i0, tc->i1);
        break;
    case TXTCMD_SERVO: {
        steering_set(tc->f0);
        float a = steering_get();            /* 回显实际生效角(钳位后) */
        cmd_note("SV=%d.%d", (int)a, dec1(a));
        ack("SV:%d.%d\r\n", (int)a, dec1(a));
        break;
    }
    case TXTCMD_SERVO_NUDGE: {
        steering_nudge((float)tc->i0);
        float a = steering_get();
        cmd_note("SV=%d.%d", (int)a, dec1(a));
        ack("SV:%d.%d\r\n", (int)a, dec1(a));
        break;
    }
    case TXTCMD_SERVO_LIM: {
        /* 限位=配置态: 由 steering 组件唯一持有并归一化
         * (下限<上限自动交换、限幅到 ±lim_abs、直行位拉回区间内) */
        steering_ret_t r;
        if      (tc->i0 == 0) r = steering_set_limit_min(tc->f0);
        else if (tc->i0 == 1) r = steering_set_limit_max(tc->f0);
        else                  r = steering_set_center(tc->f0);
        /* 非法值: 组件已钳位保底, 此处回显请求值并标记 BAD (义务 6: 不静默) */
        cmd_note("S%c=%d.%d%s", "LRC"[tc->i0], (int)tc->f0, dec1(tc->f0),
                 (r == STEERING_OK) ? "" : "!");
        ack("S%c:%d.%d%s\r\n", "LRC"[tc->i0], (int)tc->f0, dec1(tc->f0),
            (r == STEERING_OK) ? "" : " BAD");
        break;
    }
    case TXTCMD_STEP: {
        /* [app_control 拆分] 校验/激活/安全前置全部在模块内 */
        bridge_ret_t sr = app_control_step_start(
            (uint8_t)tc->i0, (int16_t)tc->i1,
            (uint32_t)tc->i2, (uint8_t)tc->i3, s_now);
        if (sr == BRIDGE_OK) {
            cmd_note("STEP%d %d", tc->i0, tc->i1);
            ack("STEP GO %d %d %d %d\r\n", tc->i0, tc->i1, tc->i2, tc->i3);
        } else if (sr == BRIDGE_ERR_BUSY) {
            ack("STEP BUSY\r\n"); cmd_note("STEP BUSY");
        } else {
            ack("STEP BAD\r\n"); cmd_note("STEP BAD");
        }
        break;
    }
    case TXTCMD_TEL:
        app_control_tel_set((uint8_t)tc->i0);
        cmd_note("TEL %d", tc->i0 ? 1 : 0);
        ack("TEL:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_FF:
        if (tc->i0 < 0 || tc->i0 > 500) { ack("FF BAD\r\n"); cmd_note("FF BAD"); break; }
        for (int i = 0; i < 2; i++) speed_loop_set_ff_gain(i, tc->i0 / 100.0f);
        cmd_note("FF %d", tc->i0);
        ack("FF:%d.%02d\r\n", tc->i0 / 100, tc->i0 % 100);
        break;
    case TXTCMD_REC:
        app_control_rec_set((uint8_t)tc->i0);
        cmd_note("REC %d", tc->i0 ? 1 : 0);
        ack("REC:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_DUMP:
        /* [app_control 拆分] 重放输出/延时/收尾全部在模块内 */
        app_control_rec_dump();
        break;
    case TXTCMD_ODOM:
        app_control_odom_set((uint8_t)tc->i0);
        cmd_note("ODOM %d", tc->i0 ? 1 : 0);
        ack("ODOM:%d\r\n", tc->i0 ? 1 : 0);
        break;
    case TXTCMD_WD:
        /* 范围 0~60000ms, 0=关 (校验在 app_control); 使能时刷新喂狗时刻 */
        if (app_control_wd_set((uint32_t)tc->i0) != BRIDGE_OK) {
            ack("WD BAD\r\n"); cmd_note("WD BAD");
        } else {
            s_io.wd_feed(s_io.ctx);   /* 使能即刷新喂狗基准 (等价原直写) */
            cmd_note("WD %dms", tc->i0);
            ack("WD:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_ITEL:
        if (app_control_itel_set((uint8_t)tc->i0) != BRIDGE_OK) {
            ack("ITEL BAD\r\n"); cmd_note("ITEL BAD");
        } else {
            cmd_note("ITEL %d", tc->i0);
            ack("ITEL:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_IGAIN:
        /* Mahony 增益 ×100 (RAM 生效, 断电失); 允许 0 0 = 纯陀螺积分 (E2 实验) */
        if (tc->i0 < 0 || tc->i1 < 0) { ack("IGAIN BAD\r\n"); cmd_note("IGAIN BAD"); break; }
        (void)imu_bridge_set_mahony_gains(0, tc->i0 / 100.0f, tc->i1 / 100.0f);
        cmd_note("IGAIN %d %d", tc->i0, tc->i1);
        ack("IGAIN:%d %d\r\n", tc->i0, tc->i1);
        break;
    case TXTCMD_IDRIFT:
        if (tc->i0 < 0 || tc->i0 > 1) { ack("IDRIFT BAD\r\n"); cmd_note("IDRIFT BAD"); break; }
        (void)imu_bridge_set_drift_enable(0, (uint8_t)tc->i0);
        cmd_note("IDRIFT %d", tc->i0);
        ack("IDRIFT:%d\r\n", tc->i0);
        break;
    case TXTCMD_IRATE:
        if (app_control_imu_rate_set((uint16_t)tc->i0) != BRIDGE_OK) {
            ack("IRATE BAD\r\n"); cmd_note("IRATE BAD");
        } else {
            cmd_note("IRATE %d", tc->i0);
            ack("IRATE:%d\r\n", tc->i0);
        }
        break;
    case TXTCMD_ICAL:
        /* 前置: 车体静止水平; 之后 50 拍(默认5s)校准, 完成后 yaw 归零 */
        if (imu_bridge_recalibrate(0) != BRIDGE_OK) { ack("ICAL BAD\r\n"); cmd_note("ICAL BAD"); break; }
        cmd_note("ICAL GO");
        ack("ICAL:GO\r\n");
        break;
    case TXTCMD_LINK:
        /* [P2 双链路仲裁] 查询/切换 — 任意链路可发 (安全例外, 设计文档 §2.2);
         * 让出/接管/抢回三语义合一: 目标=对方即切换, 目标=自己即无操作 */
        if (tc->i0 < 0) {
            cmd_note("OWNER:%s", link_arb_owner() == LINK_ARB_USB ? "USB" : "UART");
            ack("OWNER:%s\r\n", (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
        } else if (s_cfg.uart_enabled && s_cfg.usb_enabled) {
            LinkArbRet r = link_arb_request(tc->i0 == 1 ? LINK_ARB_UART : LINK_ARB_USB);
            if (r == LINK_ARB_OK) {
                cmd_note("LINK:%s", tc->i0 == 1 ? "UART" : "USB");
                ack("LINK:%s OK\r\n", (tc->i0 == 1) ? "UART" : "USB");
            } else {       /* ERR_SESSION — 会话锁 */
                cmd_note("BUSY SESSION");
                ack("BUSY:SESSION\r\n");
            }
        } else {
            /* [P3 开关收口] 单链路模式: 目标链路编译期不存在 → 明确回 LINK:OFF
             * (既不当成功也不无声无操作, 上位机必须能分辨"切不了") */
            uint8_t fb = s_cfg.usb_enabled ? APP_LINK_USB : APP_LINK_UART;
            if (((tc->i0 == 1) ? APP_LINK_UART : APP_LINK_USB) == fb) {
                ack("LINK:%s OK\r\n", tc->i0 == 1 ? "UART" : "USB");
            } else {
                cmd_note("LINK:OFF");
                ack("LINK:OFF\r\n");
            }
        }
        break;
    default:
        break;
    }
}

/* ---- 消费任务 ---- */

void app_link_task(uint32_t now)
{
    if (!s_inited) return;
    s_now = now;

    static uint8_t b;   /* 单字节暂存 (原主循环局部) */

    /* [P1 双链路] 命令级交织消费: 每链路每轮最多 byte_budget 字节 (时间片封顶);
     * 执行完一条命令再回来读下一条, 双链路按行/帧粒度交错, 谁也不饿死谁 */
    for (uint8_t li = 0; li < APP_LINK_COUNT; li++) {
        if (li == APP_LINK_UART   && !s_cfg.uart_enabled) continue;   /* [P3] 下线不消费 */
        if (li == APP_LINK_USB    && !s_cfg.usb_enabled)  continue;
        uint8_t budget = s_cfg.byte_budget;
        RingBuffer *rb = (li == APP_LINK_UART) ? &s_rb_uart : &s_rb_usb;
        while (budget-- && ringbuf_read(rb, &b) == 0) {
            s_io.set_active(li, s_io.ctx);   /* 应答路由: 谁的命令回谁 */
            if (li == APP_LINK_UART) { s_uart_last_rx = now; s_uart_seen = 1; }
            if (li == s_io.owner_link(s_io.ctx)) {   /* [P2] WD 只盯 owner 链路 */
                s_io.wd_feed(s_io.ctx);              /* 看门狗: owner 字节算"活"+解除闩锁 */
            }
            if (b == 0xAA && !s_in_frame[li]) { s_f_len[li] = 0; s_in_frame[li] = 1; s_t_frame[li] = now; }
            if (s_in_frame[li]) {
                if (s_f_len[li] >= sizeof(s_frame[li])) { s_in_frame[li] = 0; continue; }
                s_frame[li][s_f_len[li]++] = b;
                s_t_frame[li] = now;
                if (s_f_len[li] >= 2 && s_frame[li][s_f_len[li]-2] == 0xFF && s_frame[li][s_f_len[li]-1] == 0xFF) {
                    uint8_t cmd = s_frame[li][1], flen = s_f_len[li] - 2;
                    /* [P2 仲裁门控] 二进制帧: 非 owner 仅放行心跳 0xF0, 其余回 BUSY 拒绝 */
                    if (li != s_io.owner_link(s_io.ctx) && cmd != 0xF0) {
                        cmd_note("BUSY");
                        ack("BUSY:%s\r\n", (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
                    }
                    else if      (cmd == 0x01 && flen >= 7 && s_frame[li][2] < 2) { uint8_t id = s_frame[li][2]; float v; memcpy(&v,&s_frame[li][3],4); override_manual(); speed_loop_set_target(id, v); cmd_note("M%d=%dRPM", id, (int)((v>0)?v:-v)); ack("M%d:%dRPM\r\n",id,spd_to_int(v)); }
                    else if (cmd == 0x02 && flen >= 7 && s_frame[li][2] < 2) { uint8_t id = s_frame[li][2]; float v; memcpy(&v,&s_frame[li][3],4); override_manual(); (void)motor_bridge_set_speed_mps(id,v); cmd_note("M%d=%dcm/s", id, (int)(v*100)); ack("M%d:%dcm/s\r\n",id,spd_to_int(v*100.0f)); }
                    else if (cmd == 0x03 && flen >= 3 && s_frame[li][2] < 2) { uint8_t id = s_frame[li][2]; override_manual(); speed_loop_stop(id); cmd_note("BRK%d", id); ack("M%d:BRAKE\r\n",id); }
                    else if (cmd == 0x10 && flen >= 7 && s_frame[li][2] < 1) { float v; memcpy(&v,&s_frame[li][3],4); steering_set(v); float a = steering_get(); cmd_note("SV=%d.%d", (int)a, dec1(a)); ack("SV:%d.%d\r\n", (int)a, dec1(a)); }
                    else if (cmd == 0xE0 && flen >= 15 && s_frame[li][2] < 2) { uint8_t id = s_frame[li][2]; float kp,ki,kd; memcpy(&kp,&s_frame[li][3],4); memcpy(&ki,&s_frame[li][7],4); memcpy(&kd,&s_frame[li][11],4); speed_loop_set_gains(id,kp,ki,kd); cmd_note("PID%d", id); ack("OK\r\n"); }
                    else if (cmd == 0x20 && flen >= 7 && s_frame[li][2] < 2) { uint8_t id = s_frame[li][2]; float v; memcpy(&v,&s_frame[li][3],4); if (v >= -100.0f && v <= 100.0f) { override_manual(); (void)motor_bridge_set_rate_0E3(id, (int16_t)(v * 10.0f)); cmd_note("DUTY%d", id); ack("D%d:%d.%d\r\n", id, (int)v, dec1(v)); } else { cmd_note("DUTY BAD"); ack("?\r\n"); } }
                    else if (cmd == 0xF0) { cmd_note("PING->PONG"); ack("PONG\r\n"); }
                    else { cmd_note("ERR:%02X", cmd); ack("?\r\n"); }
                    s_in_frame[li] = 0; s_f_len[li] = 0;
                }
                continue;
            }
            if (b == '\r') continue;  /* 兼容手机APP "\r\n" 行尾 */
            if (b == '\n') s_txt[li][s_txt_len[li]] = 0;
            else if (s_txt_len[li] < 19) { s_txt[li][s_txt_len[li]++] = b; continue; }
            if (s_txt_len[li] > 0) {
                /* 文本命令解析已模块化到 txt_cmd.c (PC 测试桩覆盖),
                 * 手写数值解析替代 sscanf %f (newlib-nano 缺 _scanf_float) */
                TxtCmd tc;
                if (txt_cmd_parse((char *)s_txt[li], &tc)) {
                    /* [P2 仲裁门控] 非 owner 链路仅放行 安全例外命令 (STOP/PING/LINK);
                     * 其余回 BUSY:<owner> 拒绝 */
                    if (li != s_io.owner_link(s_io.ctx) &&
                        tc.type != TXTCMD_PING && tc.type != TXTCMD_STOP &&
                        tc.type != TXTCMD_LINK) {
                        cmd_note("BUSY");
                        ack("BUSY:%s\r\n",
                            (link_arb_owner() == LINK_ARB_USB) ? "USB" : "UART");
                    }
                    else {
                        exec_text_cmd(&tc);
                    }
                } else {
                    /* 识别失败的文本回 '?', 无线调试时确认"收到了但没看懂" */
                    cmd_note("ERR TXT");
                    ack("?\r\n");
                }
            }
            s_txt_len[li] = 0;
          }
        }

        /* 无线丢包保护(按链路): 二进制帧不完整超时丢弃, 重新同步
         * (无线链路可能吞掉帧尾 0xFF 0xFF, 不复位会卡住后续解析) */
        for (uint8_t li = 0; li < APP_LINK_COUNT; li++)
            if (s_in_frame[li] && (now - s_t_frame[li]) > s_cfg.frame_timeout_ms) {
                s_in_frame[li] = 0; s_f_len[li] = 0;
            }
}
