/**
 * ============================================================================
 *  执行组件 — speed_loop 实现（速度环：意图 → 器件级目标，纯 C）
 * ============================================================================
 *
 *  数据流（每 50ms，节拍由组装层给）：
 *    main.c 命令 / line_follower ──set_target(带符号 RPM)──▶ [本组件]
 *                                                            │ ① 测速：编码器计数 → 带符号 RPM
 *                                                            │ ② PID(增量式) + 前馈
 *                                                            │ ③ 输出限幅
 *                                                            ▼
 *                                        speed_loop_io_t.set_rpm(id, 带符号 RPM)
 *                                                            ▼
 *                                        motor_bridge → Motor → TB6612Protocol → pwm
 *
 *  设计要点：
 *    ① 零 HAL / 零 HAL_GetTick / 只依赖 pid.h（纯数学）→ PC 桩可编译（义务 7）
 *    ② 输出、刹停、编码器读全部走函数指针注入 → 本组件不含桥接层头，
 *       执行栈向上不反向依赖（§1.2）
 *    ③ 闭环与开环（阶跃测试）共用 speed_loop_measure() → 回绕校正/反馈符号只有一份，
 *       杜绝"测速逻辑两处实现"分叉（原 main.c 靠注释"与闭环同源"维持，属隐患）
 *    ④ 停车语义集中在本组件：|目标| < MIN_RUN_RPM → 释放 PID + 物理刹停，
 *       调用方无法绕过（照搬原 main.c 的"0 速必停"实测结论）
 * ============================================================================
 */

#include "speed_loop.h"
#include "pid.h"

/* ---- 实现内常量（实现细节，不进契约）---- */
#define DT_MIN_S        0.01f    /* dt 下限：过小则转速换算噪声爆炸（50ms 节拍下不会触发） */
#define DT_MAX_S        2.0f     /* dt 上限：超大说明长时间未跑本环，基准已失效需重同步 */

/* ---- 组件内部状态（单实例、通道数组化；执行栈多通道与桥接层同构）---- */
static speed_loop_cfg_t    g_cfg;
static speed_loop_io_t     g_io;
static PID_Handle          g_pid[SPEED_LOOP_MAX_CH];
static speed_loop_state_t  g_st[SPEED_LOOP_MAX_CH];
static int32_t             g_prev_enc[SPEED_LOOP_MAX_CH];
static uint32_t            g_prev_tick[SPEED_LOOP_MAX_CH];
static uint8_t             g_basis_ok[SPEED_LOOP_MAX_CH];   /* 测速基准是否已建立 */
static float               g_dt[SPEED_LOOP_MAX_CH];         /* 最近一次有效 dt（供 PID 复用） */
static uint8_t             g_inited;

/* ---- 内部工具 ---- */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 有限值判定（拒绝 NaN / ±Inf，避免脏值进积分器后不可逆） */
static uint8_t is_finite_f(float v)
{
    return (v == v) && (v <= 1.0e30f) && (v >= -1.0e30f);
}

static uint8_t ch_ok(uint8_t id)
{
    return (g_inited && id < g_cfg.ch_count) ? 1 : 0;
}

/* 把通道拉回"停车态"：目标 0、PID 释放、器件物理刹停、状态归零、测速基准作废。
 *
 * 基准作废（而非就地刷新）的原因：本函数也会被"命令上下文"调用（BRK / 接管 / 急停），
 * 那里拿不到可靠的 now —— 伪造成上一帧时刻会让下一次测速用错误的 dt 算出假转速。
 * 改为作废后，由节拍的 update() 用真实 now 重建基准（见 update 停车分支），
 * 效果等价于原 main.c 中 dir==0 分支的"退出停车从零起步"，且少丢一帧。 */
static void channel_park(uint8_t id)
{
    pid_reset(&g_pid[id]);
    if (g_io.brake) g_io.brake(id);
    g_st[id].target_rpm = 0.0f;
    g_st[id].actual_rpm = 0.0f;
    g_st[id].out_rpm    = 0.0f;
    g_st[id].running    = 0;
    g_basis_ok[id]      = 0;
}

/* ---- 对外接口 ---- */

speed_loop_ret_t speed_loop_init(const speed_loop_cfg_t *cfg, const speed_loop_io_t *io)
{
    if (cfg == 0 || io == 0)                          return SPEED_LOOP_ERR_BAD_ARG;
    if (io->set_rpm == 0 || io->brake == 0 ||
        io->read_enc == 0)                            return SPEED_LOOP_ERR_BAD_ARG;
    if (cfg->ch_count == 0 ||
        cfg->ch_count > SPEED_LOOP_MAX_CH)            return SPEED_LOOP_ERR_BAD_CFG;

    for (uint8_t i = 0; i < cfg->ch_count; i++) {
        const speed_loop_ch_cfg_t *c = &cfg->ch[i];
        if (c->ppr <= 0.0f || !is_finite_f(c->ppr))   return SPEED_LOOP_ERR_BAD_CFG;
        if (c->enc_span < 0)                          return SPEED_LOOP_ERR_BAD_CFG;
        if (!is_finite_f(c->out_min) || !is_finite_f(c->out_max)) return SPEED_LOOP_ERR_BAD_CFG;
        if (c->out_min >= c->out_max)                 return SPEED_LOOP_ERR_BAD_CFG;
        if (c->ff_gain < 0.0f || !is_finite_f(c->ff_gain))        return SPEED_LOOP_ERR_BAD_CFG;
        if (c->kp < 0.0f || c->ki < 0.0f || c->kd < 0.0f)         return SPEED_LOOP_ERR_BAD_CFG;
        if (!is_finite_f(c->kp) || !is_finite_f(c->ki) || !is_finite_f(c->kd))
                                                      return SPEED_LOOP_ERR_BAD_CFG;
    }

    g_cfg = *cfg;
    g_io  = *io;

    for (uint8_t i = 0; i < g_cfg.ch_count; i++) {
        PID_Params_t p;
        p.kp = g_cfg.ch[i].kp;  p.ki = g_cfg.ch[i].ki;  p.kd = g_cfg.ch[i].kd;
        p.out_min = g_cfg.ch[i].out_min;
        p.out_max = g_cfg.ch[i].out_max;
        p.integral_limit = 0.0f;            /* 0 = 由 pid_init 取输出幅值一半（增量式不依赖 ∫） */
        pid_init(&g_pid[i], PID_MODE_INCREMENTAL, &p);

        g_st[i].target_rpm = 0.0f;
        g_st[i].actual_rpm = 0.0f;
        g_st[i].out_rpm    = 0.0f;
        g_st[i].running    = 0;
        g_prev_enc[i]      = 0;
        g_prev_tick[i]     = 0;
        g_basis_ok[i]      = 0;             /* 首次 measure 只建基准，不产出值 */
        g_dt[i]            = 0.0f;
    }

    g_inited = 1;
    return SPEED_LOOP_OK;                   /* 初始化不发车：目标全 0，等第一条意图 */
}

speed_loop_ret_t speed_loop_measure(uint8_t id, uint32_t now_ms, float *rpm_out)
{
    if (!g_inited) return SPEED_LOOP_ERR_NOT_INIT;
    if (id >= g_cfg.ch_count) return SPEED_LOOP_ERR_BAD_ARG;

    int32_t enc = g_io.read_enc(id);

    /* 基准未建立：只建立基准并退出（等价于原实现"首帧 dt=0 丢弃"） */
    if (!g_basis_ok[id]) {
        g_prev_enc[id]  = enc;
        g_prev_tick[id] = now_ms;
        g_basis_ok[id]  = 1;
        return SPEED_LOOP_ERR_BAD_ARG;
    }

    float dt = (float)(now_ms - g_prev_tick[id]) * 0.001f;
    if (dt < DT_MIN_S || dt > DT_MAX_S) {
        g_prev_enc[id]  = enc;              /* 间隔异常：丢弃本次并重同步，不做除法 */
        g_prev_tick[id] = now_ms;
        return SPEED_LOOP_ERR_BAD_ARG;
    }

    int32_t delta = enc - g_prev_enc[id];
    if (g_cfg.ch[id].enc_span > 0) {        /* 编码器计数回绕校正（16 位定时器 = 65536） */
        int32_t half = g_cfg.ch[id].enc_span / 2;
        if (delta >  half) delta -= g_cfg.ch[id].enc_span;
        if (delta < -half) delta += g_cfg.ch[id].enc_span;
    }

    g_prev_enc[id]  = enc;
    g_prev_tick[id] = now_ms;
    g_dt[id]        = dt;

    if (rpm_out) {
        *rpm_out = (float)delta * 60.0f / (dt * g_cfg.ch[id].ppr)
                   * (float)g_cfg.ch[id].fb_sign;
    }
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_resync(uint8_t id, uint32_t now_ms)
{
    if (!g_inited) return SPEED_LOOP_ERR_NOT_INIT;
    if (id >= g_cfg.ch_count) return SPEED_LOOP_ERR_BAD_ARG;

    g_prev_enc[id]  = g_io.read_enc(id);
    g_prev_tick[id] = now_ms;
    g_basis_ok[id]  = 1;
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_update(uint32_t now_ms)
{
    if (!g_inited) return SPEED_LOOP_ERR_NOT_INIT;

    for (uint8_t m = 0; m < g_cfg.ch_count; m++) {
        const speed_loop_ch_cfg_t *c = &g_cfg.ch[m];
        float tgt = g_st[m].target_rpm;

        /* 停车语义：|目标| < MIN_RUN_RPM → 释放 PID + 物理刹停（"0 速必停"） */
        if (tgt < SPEED_LOOP_MIN_RUN_RPM && tgt > -SPEED_LOOP_MIN_RUN_RPM) {
            channel_park(m);
            speed_loop_resync(m, now_ms);   /* 持续停车期保持基准新鲜 → 重启时首帧即有效 */
            g_st[m].target_rpm = tgt;       /* 保留 |tgt|<1 的原始意图（遥测可读，非 0 也如实反映） */
            continue;
        }

        float rpm;
        if (speed_loop_measure(m, now_ms, &rpm) != SPEED_LOOP_OK) continue;  /* 无有效样本 */

        g_st[m].actual_rpm = rpm;

        /* 带符号前馈：本环工作域 = 带符号 RPM，方向由符号统一表达 */
        float ff  = tgt * c->ff_gain;
        float out = pid_update(&g_pid[m], tgt, rpm, g_dt[m]) + ff;
        out = clampf(out, c->out_min, c->out_max);

        if (g_io.set_rpm) g_io.set_rpm(m, out);

        g_st[m].out_rpm = out;
        g_st[m].running = 1;
    }
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_set_target(uint8_t id, float rpm_signed)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    if (!is_finite_f(rpm_signed)) return SPEED_LOOP_ERR_BAD_ARG;

    /* 目标不做上限钳位：超出输出上限时由 PID 输出限幅自然饱和（与原实现一致）。
     * |目标| < MIN_RUN_RPM 时立即进入停车态（释放 PID + 物理刹停），不等下一个节拍。 */
    g_st[id].target_rpm = rpm_signed;
    if (rpm_signed < SPEED_LOOP_MIN_RUN_RPM && rpm_signed > -SPEED_LOOP_MIN_RUN_RPM) {
        channel_park(id);
        g_st[id].target_rpm = rpm_signed;   /* channel_park 会清目标，这里还原原始意图 */
    }
    return SPEED_LOOP_OK;
}
/* 2026-09-20 P1-6: speed_loop_get_target 判死删除（§7 零调用; 目标读出走 get_state）。 */

speed_loop_ret_t speed_loop_stop(uint8_t id)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    channel_park(id);
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_reset(uint8_t id)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    pid_reset(&g_pid[id]);
    g_basis_ok[id] = 0;                      /* 下次测速重建基准 */
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_set_gains(uint8_t id, float kp, float ki, float kd)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    /* 负增益 = 正反馈，直接拒绝（不静默钳位，避免"设了但没生效"） */
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) return SPEED_LOOP_ERR_BAD_ARG;
    if (!is_finite_f(kp) || !is_finite_f(ki) || !is_finite_f(kd)) return SPEED_LOOP_ERR_BAD_ARG;

    g_cfg.ch[id].kp = kp;
    g_cfg.ch[id].ki = ki;
    g_cfg.ch[id].kd = kd;
    pid_set_gains(&g_pid[id], kp, ki, kd);
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_get_gains(uint8_t id, float *kp, float *ki, float *kd)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    if (kp == 0 || ki == 0 || kd == 0) return SPEED_LOOP_ERR_BAD_ARG;
    *kp = g_cfg.ch[id].kp;
    *ki = g_cfg.ch[id].ki;
    *kd = g_cfg.ch[id].kd;
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_set_ff_gain(uint8_t id, float gain)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    if (gain < 0.0f || !is_finite_f(gain)) return SPEED_LOOP_ERR_BAD_ARG;
    g_cfg.ch[id].ff_gain = gain;
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_get_ff_gain(uint8_t id, float *gain)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    if (gain == 0) return SPEED_LOOP_ERR_BAD_ARG;
    *gain = g_cfg.ch[id].ff_gain;
    return SPEED_LOOP_OK;
}

speed_loop_ret_t speed_loop_get_state(uint8_t id, speed_loop_state_t *out)
{
    if (!ch_ok(id)) return g_inited ? SPEED_LOOP_ERR_BAD_ARG : SPEED_LOOP_ERR_NOT_INIT;
    if (out == 0) return SPEED_LOOP_ERR_BAD_ARG;
    *out = g_st[id];
    return SPEED_LOOP_OK;
}

uint8_t speed_loop_is_init(void)
{
    return g_inited;
}
