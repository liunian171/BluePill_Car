/**
 * ============================================================================
 *  app_control.c — 50ms 控制节拍宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（判活/编码器/发送/页记录/延时全部 io 注入,
 *        保 PC 桩可编译）。
 *  搬迁自: main.c 50ms 控制块（判活喂入/WD/STEP/闭环搬运/odom 组帧/TEL/REC）
 *          + IMU 独立节拍块（update_filter/ITEL）+ 调参命令的执行状态机。
 *  语义承诺: 与搬迁前逐行为等价（分频相位/闩锁/独占执行链/STOP 联动不变）。
 * ============================================================================
 */

#include "app/app_control.h"
#include "driver/link_arbiter.h"
#include "driver/line_follower.h"
#include "driver/speed_loop.h"
#include "driver/steering.h"
#include "driver/odom.h"
#include "driver/encoder.h"
#include "driver/imu_bridge.h"
#include "driver/motor_bridge.h"
#include <string.h>
#include <stdio.h>

/* ---- 协议号（自 main.c PV 区迁入, 仅本模块使用; 权威 = README §3.8.1） ---- */
#define ODOM_FRAME_CMD   0x51
#define ATT_FRAME_CMD    0x52

/* ---- 私有状态（原 main.c 调参/工具链/WD 6 组跨块共享标志收编于此） ---- */
static app_control_cfg_t s_cfg;
static app_control_io_t  s_io;
static uint8_t           s_inited = 0;

/* 阶跃测试（激活时独占执行链） */
static uint8_t  s_step_active = 0;
static uint8_t  s_step_m      = 0;
static int16_t  s_step_rate   = 0;
static uint8_t  s_step_div    = 1;
static uint8_t  s_step_cnt    = 0;
static uint32_t s_step_t0     = 0;
static uint32_t s_step_end    = 0;
static uint8_t  s_step_first  = 1;

/* 闭环遥测 / 机内记录 */
static uint8_t  s_tel_on  = 0;
static uint8_t  s_tel_div = 0;
static uint8_t  s_rec_on  = 0;
static uint16_t s_rec_cnt = 0;
#define REC_MAX_CAP 128                    /* 与原 main.c REC_MAX 一致 (cfg.rec_max 封顶) */
static uint32_t s_rec_tick[REC_MAX_CAP];
static int16_t  s_rec_t0[REC_MAX_CAP], s_rec_r0[REC_MAX_CAP];
static int16_t  s_rec_t1[REC_MAX_CAP], s_rec_r1[REC_MAX_CAP];

/* IMU 更新 / ITEL */
static uint16_t s_imu_period_ms = 100;
static uint8_t  s_itel_mode     = 0;       /* 0关/1=CSV/2=VOFA+ FireWater */
static uint8_t  s_itel_div      = 0;
static uint32_t t_imu           = 0;

/* 里程计上报 / 看门狗 */
static uint8_t  s_odom_on   = 0;
static uint8_t  s_att_div   = 0;
static uint32_t s_wd_ms     = 0;
static uint8_t  s_wd_fired  = 0;

/* 决策组件意图缓冲（line_follower 写 → 本模块推给 speed_loop; 原 g_lf_tgt/dir） */
static float   s_lf_tgt[2] = {0.0f, 0.0f};
static int8_t  s_lf_dir[2] = {0, 0};

/* 节拍基准 */
static uint32_t t_ctrl = 0;

/* ---- 内部助手 ---- */

/* 带符号浮点 → 四舍五入 int（遥测/记录用; 与 main.c 的 spd_to_int 同实现,
 * 统一归拢到 common/tool 留待后续卫生批） */
static int spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* float → "-1.63"（禁 %f 整数拆分; VOFA+ FireWater 用, 自 main.c fmt_f2 迁入） */
static int fmt_f2(char *out, int n, float v)
{
    int neg = (v < 0);
    float a = neg ? -v : v;
    int ip = (int)a;
    int fp = (int)((a - (float)ip) * 100.0f + 0.5f);
    if (fp >= 100) { ip += 1; fp -= 100; }
    return snprintf(out, n, "%s%d.%02d", neg ? "-" : "", ip, fp);
}

/* ---- 生命周期 ---- */

bridge_ret_t app_control_init(const app_control_cfg_t *cfg, const app_control_io_t *io)
{
    if (cfg == NULL || io == NULL) return BRIDGE_ERR_BAD_ARG;
    if (cfg->ctrl_period_ms == 0)  return BRIDGE_ERR_BAD_CFG;
    if (io->usb_alive == NULL || io->last_rx_tick == NULL || io->enc_count == NULL ||
        io->send_line == NULL || io->note_cmd == NULL || io->resp == NULL ||
        io->delay_ms == NULL)
        return BRIDGE_ERR_BAD_ARG;
    s_cfg = *cfg;
    s_io  = *io;
    s_imu_period_ms = cfg->imu_period_ms;
    s_wd_ms = cfg->wd_default_off ? 0u : s_wd_ms;
    s_inited = 1;
    return BRIDGE_OK;
}

void app_control_task(uint32_t now)
{
    if (!s_inited) return;

    if (now - t_ctrl >= s_cfg.ctrl_period_ms) {
        t_ctrl = now;

        /* ═══ [P2] USB 判活喂入（双链路并存才喂; P3 单链路不喂伪判活） ═══ */
        if (s_cfg.both_links) {
            (void)link_arb_notify((int8_t)(s_io.usb_alive(s_io.ctx) ? 1 : -1), now);
        }

        /* ═══ 命令看门狗: 超时无下行字节 → 急停一次 (闩锁防刷屏) ═══ */
        if (s_wd_ms > 0 && !s_wd_fired &&
            (now - s_io.last_rx_tick(s_io.ctx)) > s_wd_ms) {
            for (int i = 0; i < 2; i++) speed_loop_stop(i);
            line_follower_enable(0);
            line_follower_set_auto(0);
            steering_center();
            s_wd_fired = 1;
            s_io.note_cmd("WD TIMEOUT", s_io.ctx);
            s_io.send_line("WD TIMEOUT STOP\r\n", 17, s_io.ctx);
        }

        if (s_step_active) {
            /* ═══ 阶跃测试: 激活时独占执行链 (测速复用 speed_loop_measure) ═══ */
            if (now < s_step_end) {
                (void)motor_bridge_set_rate_0E3(s_step_m, s_step_rate);
            } else {
                s_step_active = 0;
                (void)motor_bridge_brake(s_step_m);
                speed_loop_reset(s_step_m);     /* 释放 PID + 测速基准作废 */
                s_io.resp("STEP END\r\n", s_io.ctx);
            }
            float step_rpm;
            if (s_step_first) {
                speed_loop_resync(s_step_m, now);   /* 建基准并丢弃该帧 */
                s_step_first = 0;
            } else if (speed_loop_measure(s_step_m, now, &step_rpm) == SPEED_LOOP_OK) {
                if ((++s_step_cnt % s_step_div) == 0) {   /* 分频: 弱链路防丢行 */
                    char tb[36];
                    int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d\r\n",
                                      (unsigned long)(now - s_step_t0),
                                      (int)s_step_rate, spd_to_int(step_rpm * 10.0f));
                    if (tn > 0) s_io.send_line(tb, tn, s_io.ctx);
                }
            }
        } else {
            /* ═══ 寻迹控制: 巡线写意图 → 启用时推给执行组件 ═══ */
            line_follower_update(now, s_lf_tgt, s_lf_dir,
                                 s_io.enc_count(0, s_io.ctx),
                                 s_io.enc_count(1, s_io.ctx));
            if (line_follower_enabled()) {
                for (uint8_t m = 0; m < 2; m++)
                    speed_loop_set_target(m, s_lf_tgt[m] * (float)s_lf_dir[m]);
            }
            /* ═══ 速度环: 测速 → PID+前馈 → 限幅 → 下发 ═══ */
            speed_loop_update(now);

            /* ═══ 里程计: 恒解算保持新鲜; 开启时组帧 ODOM 20Hz + ATT 10Hz ═══ */
            odom_delta_t od;
            if (odom_update(now, &od) == ODOM_OK && s_odom_on) {
                uint8_t ob[20];
                float   th = od.dtheta_deg;
                ob[0] = 0xAA; ob[1] = ODOM_FRAME_CMD;
                memcpy(&ob[2],  &od.dx_mm, 4);
                memcpy(&ob[6],  &od.dy_mm, 4);
                memcpy(&ob[10], &th,       4);
                memcpy(&ob[14], &now,      4);
                ob[18] = 0xFF; ob[19] = 0xFF;
                s_io.send_line((const char *)ob, 20, s_io.ctx);
                if ((++s_att_div & 1) == 0) {
                    uint8_t ab[20];
                    float r = imu_bridge_get_roll(0);
                    float p = imu_bridge_get_pitch(0);
                    float y = imu_bridge_get_yaw(0);
                    ab[0] = 0xAA; ab[1] = ATT_FRAME_CMD;
                    memcpy(&ab[2],  &r,   4);
                    memcpy(&ab[6],  &p,   4);
                    memcpy(&ab[10], &y,   4);
                    memcpy(&ab[14], &now, 4);
                    ab[18] = 0xFF; ab[19] = 0xFF;
                    s_io.send_line((const char *)ab, 20, s_io.ctx);
                }
            }

            /* ═══ 闭环遥测 (10Hz CSV: "tick,tgt0,rpm0,tgt1,rpm1") ═══ */
            if (s_tel_on && (++s_tel_div & 1) == 0) {
                speed_loop_state_t s0, s1;
                speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
                char tb[44];
                int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d,%d,%d\r\n",
                                  (unsigned long)now,
                                  (int)s0.target_rpm, spd_to_int(s0.actual_rpm),
                                  (int)s1.target_rpm, spd_to_int(s1.actual_rpm));
                if (tn > 0) s_io.send_line(tb, tn, s_io.ctx);
            }

            /* ═══ 机内记录 (20Hz 写 RAM, DUMP 重放; 对抗无线丢行) ═══ */
            if (s_rec_on && s_rec_cnt < s_cfg.rec_max && s_rec_cnt < REC_MAX_CAP) {
                speed_loop_state_t s0, s1;
                speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
                s_rec_tick[s_rec_cnt] = now;
                s_rec_t0[s_rec_cnt] = (int16_t)s0.target_rpm;
                s_rec_r0[s_rec_cnt] = (int16_t)spd_to_int(s0.actual_rpm);
                s_rec_t1[s_rec_cnt] = (int16_t)s1.target_rpm;
                s_rec_r1[s_rec_cnt] = (int16_t)spd_to_int(s1.actual_rpm);
                s_rec_cnt++;
            }
        }
    }

    /* ═══ IMU 感知更新 — 独立节拍 (IRATE 可调; ITEL 随更新 2 分频发射) ═══ */
    if (now - t_imu >= s_imu_period_ms) {
        t_imu = now;
        (void)imu_bridge_update_filter(0, now);
        if (s_itel_mode > 0 && (++s_itel_div & 1) == 0) {
            int16_t gx, gy, gz;
            float drx, dry, drz, gs = imu_bridge_gyro_scale(0);   /* LSB/(°/s) */
            float r = imu_bridge_get_roll(0), p = imu_bridge_get_pitch(0), y = imu_bridge_get_yaw(0);
            float d10 = 10.0f, d100 = 100.0f;
            if (imu_bridge_read_gyro_raw(0, &gx, &gy, &gz) != BRIDGE_OK) { gx = gy = gz = 0; }
            (void)imu_bridge_get_drift(0, &drx, &dry, &drz);
            float gxf = gx / gs, gyf = gy / gs, gzf = gz / gs;
            /* CSV 行缓冲 96B: 8×10 + 7 逗号 + CRLF + NUL; VOFA 分通道 16B
             * (fmt_f2 理论上界 14 字节 "-2147483647.xx") — 消除截断可能 */
            char tb[96];
            int tn;
            if (s_itel_mode == 2) {
                char c1[16], c2[16], c3[16], c4[16], c5[16], c6[16], c7[16];
                fmt_f2(c1, (int)sizeof(c1), gxf); fmt_f2(c2, (int)sizeof(c2), gyf);
                fmt_f2(c3, (int)sizeof(c3), gzf); fmt_f2(c4, (int)sizeof(c4), r);
                fmt_f2(c5, (int)sizeof(c5), p);   fmt_f2(c6, (int)sizeof(c6), y);
                fmt_f2(c7, (int)sizeof(c7), drz);
                tn = snprintf(tb, sizeof(tb), "%s,%s,%s,%s,%s,%s,%s\r\n",
                              c1, c2, c3, c4, c5, c6, c7);
            } else {
                tn = snprintf(tb, sizeof(tb), "%lu,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                              (unsigned long)now,
                              (int)(gxf * d10), (int)(gyf * d10), (int)(gzf * d10),
                              (int)((r < 0) ? (r * d10 - 0.5f) : (r * d10 + 0.5f)),
                              (int)((p < 0) ? (p * d10 - 0.5f) : (p * d10 + 0.5f)),
                              (int)((y < 0) ? (y * d10 - 0.5f) : (y * d10 + 0.5f)),
                              (int)imu_bridge_stable(0),
                              (int)((drz < 0) ? (drz * d100 - 0.5f) : (drz * d100 + 0.5f)));
            }
            if (tn > 0) s_io.send_line(tb, tn, s_io.ctx);
        }
    }
}

/* ---- 调参工具链命令入口 ---- */

bridge_ret_t app_control_step_start(uint8_t motor_id, int16_t rate_0E3,
                                    uint32_t dur_ms, uint8_t tel_div, uint32_t now)
{
    if (!s_inited) return BRIDGE_ERR_NOT_INIT;
    if (s_step_active) return BRIDGE_ERR_BUSY;
    if (motor_id > 1 || rate_0E3 < -1000 || rate_0E3 > 1000 ||
        dur_ms < 100 || dur_ms > 5000 || tel_div < 1 || tel_div > 4)
        return BRIDGE_ERR_BAD_ARG;
    /* 台架安全前置: 关巡线+关自动, 双轮清目标+PID释放+物理刹停 */
    line_follower_enable(0);
    line_follower_set_auto(0);
    for (int i = 0; i < 2; i++) speed_loop_stop(i);
    s_step_m      = motor_id;
    s_step_rate   = rate_0E3;
    s_step_div    = tel_div;
    s_step_cnt    = 0;
    s_step_t0     = now;
    s_step_end    = now + dur_ms;
    s_step_first  = 1;
    s_step_active = 1;
    return BRIDGE_OK;
}

void app_control_tel_set(uint8_t on)  { s_tel_on = on ? 1 : 0; }
void app_control_rec_set(uint8_t on)  { s_rec_on = on ? 1 : 0; if (s_rec_on) s_rec_cnt = 0; }

void app_control_rec_dump(void)
{
    /* 重放机内记录: 首行报条数, 末行报结束 — PC 端按条数校验补全
     * [已知限制] 逐行 20ms 匀速发送 (~1.3s 阻塞主循环), 换 DMA 后可移除延时 */
    s_rec_on = 0;
    char hb[20];
    int hn = snprintf(hb, sizeof(hb), "DUMP %u\r\n", (unsigned)s_rec_cnt);
    if (hn > 0) s_io.resp(hb, s_io.ctx);
    for (uint16_t i = 0; i < s_rec_cnt; i++) {
        char tb[44];
        int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d,%d,%d\r\n",
                          (unsigned long)s_rec_tick[i],
                          s_rec_t0[i], s_rec_r0[i], s_rec_t1[i], s_rec_r1[i]);
        if (tn > 0) s_io.send_line(tb, tn, s_io.ctx);
        s_io.delay_ms(20, s_io.ctx);
    }
    s_io.resp("DUMP END\r\n", s_io.ctx);
    s_io.note_cmd("DUMP", s_io.ctx);
}

bridge_ret_t app_control_itel_set(uint8_t mode)
{
    if (!s_inited) return BRIDGE_ERR_NOT_INIT;
    if (mode > 2) return BRIDGE_ERR_BAD_ARG;
    s_itel_mode = mode;
    s_itel_div = 0;   /* 开/关都对齐分频相位 */
    return BRIDGE_OK;
}

bridge_ret_t app_control_imu_rate_set(uint16_t ms)
{
    if (!s_inited) return BRIDGE_ERR_NOT_INIT;
    if (ms < 20 || ms > 1000) return BRIDGE_ERR_BAD_ARG;
    s_imu_period_ms = ms;
    return BRIDGE_OK;
}

void app_control_odom_set(uint8_t on) { s_odom_on = on ? 1 : 0; }

bridge_ret_t app_control_wd_set(uint32_t timeout_ms)
{
    if (!s_inited) return BRIDGE_ERR_NOT_INIT;
    if (timeout_ms > 60000) return BRIDGE_ERR_BAD_ARG;
    s_wd_ms = timeout_ms;
    s_wd_fired = 0;   /* 使能/改参即重新武装 (喂狗时刻由消费侧持续刷新) */
    return BRIDGE_OK;
}

void app_control_wd_rearm(void)
{
    s_wd_fired = 0;   /* 断链恢复 = 收到任意 owner 字节自动解除闩锁, 重新武装 */
}

/* ---- 查询 / 安全 ---- */

uint8_t app_control_session_active(void)
{
    return (s_step_active || s_tel_on || s_rec_on || s_odom_on || s_itel_mode) ? 1u : 0u;
}

void app_control_stop_all(void)
{
    /* STOP/WD 共用: 工具链全部关闭 (闩锁/位姿等保持既有语义) */
    s_step_active = 0;
    s_tel_on = 0; s_rec_on = 0;
    s_odom_on = 0; s_itel_mode = 0;
}
