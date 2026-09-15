/**
 * ============================================================================
 *  感知组件实现 — attitude（姿态）
 * ============================================================================
 *
 *  C3（2026-09-15）：原 imu_bridge.cpp 的"胖桥"业务段 1:1 下沉至此
 *  （初始零偏校准状态机 / 静止检测 / 门限漂移跟踪 / dt 钳位 / Mahony 调度），
 *  算法数学在 common/imu_filter.c。搬迁纪律：与原实现逐行等价（含 IMU-1 门限
 *  修复），真机以 ITEL 曲线等价比对背书（同 C2 搬迁方法）。
 *
 *  与原实现的两处有意结构差异（行为零差异）：
 *    1) 配置数值（校准拍数/阈值/门限/速率）从编译期宏/字面量 → cfg 注入
 *    2) 校准状态/漂移/滤波状态从文件级静态数组 → 通道结构体（本就 per-id）
 * ============================================================================
 */

#include "attitude.h"
#include "imu_filter.h"
#include <math.h>

/* ---- 默认配置（沿用原固件字面量；组件不隐藏默认——NULL 成员取此值）---- */
#define ATT_DEF_CAL_SAMPLES   50u
#define ATT_DEF_ACCEL_TOL     0.05f
#define ATT_DEF_GYRO_GATE     1.5f
#define ATT_DEF_RATE_SLOW     0.02f
#define ATT_DEF_RATE_FAST     0.05f

/* ---- 每通道状态（静态池，禁堆）---- */
typedef struct {
    const attitude_cfg_t *cfg;
    const attitude_io_t  *io;
    imu_filter_t          filter;

    /* 启动零偏校准状态机 */
    int32_t   gb_sum[3];      /* raw 陀螺累加 */
    uint16_t  cal_cnt;        /* < cal_samples 采集中；完成拍后 → 完成态（沿用原">N 即完成"语义） */

    /* 漂移补偿（°/s）+ 跟踪状态 */
    float     drift[3];
    uint32_t  stable_since;   /* 0 = 非连续静止 */
    uint8_t   drift_en;       /* 默认 1 = 原有行为 */
    uint8_t   is_stable;      /* 最近一拍静止标志（诊断出） */

    uint32_t  last_tick;      /* dt 基准 */
} attitude_ch_t;

static attitude_ch_t g_ch[ATTITUDE_MAX_CH];
static uint8_t       g_inited[ATTITUDE_MAX_CH] = {0};

/* 完成态判定（沿用原"cal_cnt > cal_samples 即完成"语义） */
static uint8_t cal_done(const attitude_ch_t *c)
{
    return (c->cal_cnt > c->cfg->cal_samples) ? 1u : 0u;
}

attitude_ret_t attitude_init(uint8_t id, const attitude_cfg_t *cfg, const attitude_io_t *io)
{
    if (id >= ATTITUDE_MAX_CH) return ATTITUDE_ERR_BAD_ARG;
    if (cfg == NULL || io == NULL) return ATTITUDE_ERR_BAD_ARG;
    if (io->read_accel_raw == NULL || io->read_gyro_raw == NULL) return ATTITUDE_ERR_BAD_ARG;

    /* 就地取默认：NULL/0 成员取组件默认值（显式声明，不靠调用方记忆）。
     * def 静态存储使 c->cfg 指针在 init 后持续有效。 */
    static attitude_cfg_t def[ATTITUDE_MAX_CH];
    def[id].cal_samples     = cfg->cal_samples      ? cfg->cal_samples      : ATT_DEF_CAL_SAMPLES;
    def[id].accel_tol_g     = (cfg->accel_tol_g     > 0.0f) ? cfg->accel_tol_g     : ATT_DEF_ACCEL_TOL;
    def[id].gyro_gate_dps   = (cfg->gyro_gate_dps   > 0.0f) ? cfg->gyro_gate_dps   : ATT_DEF_GYRO_GATE;
    def[id].drift_rate_slow = (cfg->drift_rate_slow > 0.0f) ? cfg->drift_rate_slow : ATT_DEF_RATE_SLOW;
    def[id].drift_rate_fast = (cfg->drift_rate_fast > 0.0f) ? cfg->drift_rate_fast : ATT_DEF_RATE_FAST;

    attitude_ch_t *c = &g_ch[id];
    c->cfg = &def[id];
    c->io  = io;

    imu_filter_init(&c->filter);
    for (int i = 0; i < 3; i++) { c->gb_sum[i] = 0; c->drift[i] = 0.0f; }
    c->cal_cnt      = 0;
    c->stable_since = 0;
    c->drift_en     = 1;      /* 默认开 = 原有行为 */
    c->is_stable    = 0;
    c->last_tick    = 0;

    g_inited[id] = 1;
    return ATTITUDE_OK;
}

attitude_ret_t attitude_set_scales(uint8_t id, float accel_lsb_per_g, float gyro_lsb_per_dps)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return ATTITUDE_ERR_NOT_INIT;
    if (accel_lsb_per_g <= 0.0f || gyro_lsb_per_dps <= 0.0f) return ATTITUDE_ERR_BAD_ARG;
    imu_filter_set_accel_scale(&g_ch[id].filter, accel_lsb_per_g);
    imu_filter_set_gyro_scale(&g_ch[id].filter, gyro_lsb_per_dps);
    return ATTITUDE_OK;
}

/* ---- 状态出：安全默认值 ---- */

uint8_t attitude_ready(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH) ? g_inited[id] : 0u;
}

float attitude_get_roll(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH && g_inited[id]) ? g_ch[id].filter.roll : 0.0f;
}

float attitude_get_pitch(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH && g_inited[id]) ? g_ch[id].filter.pitch : 0.0f;
}

float attitude_get_yaw(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH && g_inited[id]) ? g_ch[id].filter.yaw : 0.0f;
}

uint8_t attitude_cal_progress(uint8_t id)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return 0;   /* 安全侧：0 = 未完成 */
    const attitude_ch_t *c = &g_ch[id];
    if (cal_done(c)) return 100;
    /* 校准中按拍数给百分比（auto-start 判据 prog>=100 语义与原实现一致） */
    uint16_t p = (uint16_t)((uint32_t)c->cal_cnt * 100u / c->cfg->cal_samples);
    return (p > 100u) ? 100u : (uint8_t)p;
}

uint8_t attitude_stable(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH && g_inited[id]) ? g_ch[id].is_stable : 0u;
}

attitude_ret_t attitude_get_drift(uint8_t id, float *dx, float *dy, float *dz)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return ATTITUDE_ERR_NOT_INIT;
    if (dx == NULL || dy == NULL || dz == NULL) return ATTITUDE_ERR_BAD_ARG;
    *dx = g_ch[id].drift[0];
    *dy = g_ch[id].drift[1];
    *dz = g_ch[id].drift[2];
    return ATTITUDE_OK;
}

/* ---- 运行时调参 ---- */

attitude_ret_t attitude_set_mahony_gains(uint8_t id, float kp, float ki)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return ATTITUDE_ERR_NOT_INIT;
    imu_filter_set_gains(&g_ch[id].filter, kp, ki);
    return ATTITUDE_OK;
}

attitude_ret_t attitude_set_drift_enable(uint8_t id, uint8_t en)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return ATTITUDE_ERR_NOT_INIT;
    attitude_ch_t *c = &g_ch[id];
    c->drift_en = en ? 1u : 0u;
    if (!c->drift_en) {
        /* 关闭即清零：下一拍起完全无补偿（E1 实验语义） */
        for (int i = 0; i < 3; i++) c->drift[i] = 0.0f;
    }
    return ATTITUDE_OK;
}

uint8_t attitude_drift_enabled(uint8_t id)
{
    return (id < ATTITUDE_MAX_CH && g_inited[id]) ? g_ch[id].drift_en : 0u;
}

attitude_ret_t attitude_recalibrate(uint8_t id)
{
    if (id >= ATTITUDE_MAX_CH || !g_inited[id]) return ATTITUDE_ERR_NOT_INIT;
    attitude_ch_t *c = &g_ch[id];
    for (int i = 0; i < 3; i++) { c->gb_sum[i] = 0; c->drift[i] = 0.0f; }
    c->cal_cnt      = 0;
    c->stable_since = 0;
    return ATTITUDE_OK;
}

/* ==========================================================================
 *  节拍更新：读 raw → 校准状态机 → 静止检测 → 门限漂移跟踪 → Mahony
 *  （原 imu_bridge_update_filter 主体 1:1，关键机理注释保留）
 * ========================================================================== */

attitude_ret_t attitude_update(uint8_t id, uint32_t now_ms)
{
    if (id >= ATTITUDE_MAX_CH) return ATTITUDE_ERR_BAD_ARG;
    if (!g_inited[id]) return ATTITUDE_ERR_NOT_INIT;

    attitude_ch_t *c = &g_ch[id];

    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    if (c->io->read_accel_raw(id, &ax_r, &ay_r, &az_r) != ATTITUDE_OK ||
        c->io->read_gyro_raw(id, &gx_r, &gy_r, &gz_r) != ATTITUDE_OK)
        return ATTITUDE_ERR_IO;

    float af[3], gf[3];
    imu_filter_raw_to_accel(&c->filter, ax_r, ay_r, az_r, &af[0], &af[1], &af[2]);
    imu_filter_raw_to_gyro(&c->filter, gx_r, gy_r, gz_r, &gf[0], &gf[1], &gf[2]);

    /* ═══ 阶段①：初始静态零偏校准（前 cal_samples 拍取均值）═══ */
    if (c->cal_cnt < c->cfg->cal_samples) {
        c->gb_sum[0] += gx_r; c->gb_sum[1] += gy_r; c->gb_sum[2] += gz_r;
        c->cal_cnt++;
        c->last_tick = now_ms;
        return ATTITUDE_OK;
    } else if (c->cal_cnt == c->cfg->cal_samples) {
        int16_t avg[3];
        avg[0] = (int16_t)(c->gb_sum[0] / c->cfg->cal_samples);
        avg[1] = (int16_t)(c->gb_sum[1] / c->cfg->cal_samples);
        avg[2] = (int16_t)(c->gb_sum[2] / c->cfg->cal_samples);
        imu_filter_calibrate_gyro_bias(&c->filter, &avg[0], &avg[1], &avg[2], 1);
        c->cal_cnt++;   /* → 完成态 */

        /* 用加速度计初始化四元数 */
        imu_filter_init_from_accel(&c->filter, af[0], af[1], af[2]);
        c->last_tick = now_ms;
        return ATTITUDE_OK;
    }

    /* ═══ dt 计算 + 钳位（长中断后用小 dt 渐进恢复）═══ */
    float dt = (float)(now_ms - c->last_tick) * 0.001f;
    c->last_tick = now_ms;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 1.0f)   dt = 0.01f;

    /* ═══ 阶段②：静止检测 + 门限漂移跟踪 ═══
     * Mahony 的 Kp 修正的是姿态误差，不能替代陀螺零偏的直接测量；
     * 这里用静止时的陀螺输出慢速跟踪实际零偏漂移。 */
    float amag = sqrtf(af[0] * af[0] + af[1] * af[1] + af[2] * af[2]);
    int is_stable = (fabsf(amag - 1.0f) < c->cfg->accel_tol_g);
    c->is_stable = (uint8_t)(is_stable ? 1u : 0u);   /* 诊断出（ITEL stable 列） */

    /* IMU-1 yaw 失真修复（2026-09-15 真机 E1/E1b 实锤）：绕竖直轴慢转时
     * |a|≈1g → 旋转角速度被漂移跟踪当零偏吸收（真机：90° 只跟 -41.9°，
     * 停转以 driftz=-5.5°/s 回落到 0；IDRIFT 0 对照 -85.1° 满跟零回落）。
     * 修法 = 跟踪进入条件加陀螺模值门限：三轴 |gf| 均低于门限才允许跟踪。
     * 代价：<门限的极慢旋转（90°/60s@1.5°/s）仍会被吸收，可接受。 */
    if (!c->drift_en) {
        for (int i = 0; i < 3; i++) c->drift[i] = 0.0f;   /* 关闭即清零：完全无补偿 */
    }

    int track_ok = 0;
    if (is_stable && c->drift_en) {
        float gmax = fabsf(gf[0]);
        if (fabsf(gf[1]) > gmax) gmax = fabsf(gf[1]);
        if (fabsf(gf[2]) > gmax) gmax = fabsf(gf[2]);
        track_ok = (gmax < c->cfg->gyro_gate_dps);
    }

    if (track_ok) {
        if (c->stable_since == 0) c->stable_since = now_ms;
        uint32_t stable_ms = now_ms - c->stable_since;

        if (stable_ms > 1000) {
            /* 静止后快速跟踪零偏，1~5秒用慢速率，之后用快速率 */
            float rate = (stable_ms > 5000) ? c->cfg->drift_rate_fast : c->cfg->drift_rate_slow;
            c->drift[0] += (gf[0] - c->drift[0]) * rate;
            c->drift[1] += (gf[1] - c->drift[1]) * rate;
            c->drift[2] += (gf[2] - c->drift[2]) * rate;
        }
    } else {
        c->stable_since = 0;
    }

    /* 减去漂移补偿量 */
    gf[0] -= c->drift[0];
    gf[1] -= c->drift[1];
    gf[2] -= c->drift[2];

    /* ═══ 阶段③：Mahony 融合 ═══ */
    imu_filter_mahony(&c->filter, gf[0], gf[1], gf[2], af[0], af[1], af[2], dt);
    return ATTITUDE_OK;
}
