/**
 * ============================================================================
 *  感知组件 — odom（里程计）实现（见 odom.h）
 * ============================================================================
 *  解算链：Δcount → 回绕校正 → 符号修正 → 轮位移 → 差速运动学 → Δs/Δθ
 *          Δθ 融合：IMU yaw 差分优先（打滑不误），未注入回退编码器差分
 *          车体增量：中点积分（ΔX = Δs·cos(Δθ/2), ΔY = Δs·sin(Δθ/2)）
 * ============================================================================
 */
#include "driver/odom.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static odom_cfg_t   g_cfg;
static odom_io_t    g_io;
static uint8_t      g_init = 0;

/* 测速/测距基准（resync 时建立） */
static uint8_t  g_have_base = 0;
static uint32_t g_last_tick = 0;
static int32_t  g_last_cnt[ODOM_MAX_ENC] = {0, 0};
static float    g_last_yaw = 0.0f;

/* 本地累计位姿（参考值；权威在上位机） */
static float g_x = 0.0f, g_y = 0.0f, g_th = 0.0f;   /* mm, mm, ° */

/* 角度归一到 (-180, 180] */
static float wrap_180(float d)
{
    while (d > 180.0f)  d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

odom_ret_t odom_init(const odom_cfg_t *cfg, const odom_io_t *io)
{
    if (cfg == NULL || io == NULL) return ODOM_ERR_BAD_ARG;
    if (io->read_enc == NULL)      return ODOM_ERR_BAD_ARG;
    for (uint8_t i = 0; i < ODOM_MAX_ENC; i++) {
        if (!(cfg->ppr[i] > 0.0f)) return ODOM_ERR_BAD_CFG;
        if (cfg->sign[i] == 0)     return ODOM_ERR_BAD_CFG;
    }
    if (!(cfg->wheel_circ_mm > 0.0f)) return ODOM_ERR_BAD_CFG;
    if (cfg->yaw_sign != 1 && cfg->yaw_sign != -1) return ODOM_ERR_BAD_CFG;
    /* 编码器差分 Δθ 模式必须给轮距；IMU 模式不要求 */
    if (io->get_yaw == NULL && !(cfg->wheel_track_mm > 0.0f))
        return ODOM_ERR_BAD_CFG;

    g_cfg = *cfg;
    g_io  = *io;
    g_init = 1;
    g_have_base = 0;
    g_x = g_y = g_th = 0.0f;
    return ODOM_OK;
}

odom_ret_t odom_resync(uint32_t now_ms)
{
    if (!g_init) return ODOM_ERR_NOT_INIT;
    for (uint8_t i = 0; i < ODOM_MAX_ENC; i++)
        g_last_cnt[i] = g_io.read_enc((uint8_t)i);
    if (g_io.get_yaw != NULL) {
        float yaw;
        g_io.get_yaw(&yaw);
        g_last_yaw = yaw;
    }
    g_last_tick = now_ms;
    g_have_base = 1;
    return ODOM_OK;
}

odom_ret_t odom_update(uint32_t now_ms, odom_delta_t *out)
{
    if (!g_init)     return ODOM_ERR_NOT_INIT;
    if (out == NULL) return ODOM_ERR_BAD_ARG;

    if (!g_have_base) {          /* 未建基准 = 等价 resync 后首帧 */
        odom_resync(now_ms);
        out->dx_mm = 0.0f; out->dy_mm = 0.0f;
        out->dtheta_deg = 0.0f; out->dt_ms = 0;
        return ODOM_OK;
    }

    uint32_t dt = now_ms - g_last_tick;
    if (dt == 0u || dt > ODOM_DT_MAX_MS) {
        odom_resync(now_ms);     /* 节拍失步：丢弃本帧，重建基准 */
        out->dx_mm = 0.0f; out->dy_mm = 0.0f;
        out->dtheta_deg = 0.0f; out->dt_ms = 0;
        return ODOM_ERR_BAD_ARG;
    }

    /* ---- ① 编码器计数差 → 回绕校正 → 符号修正 → 轮位移(mm) ---- */
    float dmm[ODOM_MAX_ENC];
    for (uint8_t i = 0; i < ODOM_MAX_ENC; i++) {
        int32_t c = g_io.read_enc((uint8_t)i);
        int32_t dc = c - g_last_cnt[i];
        if (g_cfg.enc_span > 0.0f) {              /* 16 位计数回绕校正 */
            int32_t half = (int32_t)(g_cfg.enc_span / 2.0f);
            if (dc >  half) dc -= (int32_t)g_cfg.enc_span;
            if (dc < -half) dc += (int32_t)g_cfg.enc_span;
        }
        g_last_cnt[i] = c;
        dmm[i] = (float)dc / g_cfg.ppr[i] * g_cfg.wheel_circ_mm
                 * (float)g_cfg.sign[i];
    }
    float ds  = (dmm[0] + dmm[1]) * 0.5f;         /* 中点弧长 */
    float dth;
    if (g_io.get_yaw != NULL) {
        /* ---- ②a Δθ = IMU yaw 差分（主路径：打滑不影响航向）----
         * yaw_sign 适配安装方向：对外恒为"逆时针为正"（数学惯例/ROS 一致） */
        float yaw;
        g_io.get_yaw(&yaw);
        dth = wrap_180(yaw - g_last_yaw) * (float)g_cfg.yaw_sign;
        g_last_yaw = yaw;
    } else {
        /* ---- ②b 回退：编码器差分（track 有 b=dr-dl/rad 关系） ---- */
        dth = (dmm[1] - dmm[0]) / g_cfg.wheel_track_mm * (float)(180.0 / M_PI);
    }

    /* ---- ③ 车体系增量（中点积分） + 位姿累加 ---- */
    float half_rad = (float)(dth * M_PI / 360.0);          /* Δθ/2 → rad */
    out->dx_mm = ds * cosf(half_rad);
    out->dy_mm = ds * sinf(half_rad);
    out->dtheta_deg = dth;
    out->dt_ms = dt;

    float th_mid = (float)((g_th + dth / 2.0f) * M_PI / 180.0);
    g_x  += ds * cosf(th_mid);
    g_y  += ds * sinf(th_mid);
    g_th  = wrap_180(g_th + dth);

    g_last_tick = now_ms;
    return ODOM_OK;
}

odom_ret_t odom_get_pose(float *x_mm, float *y_mm, float *theta_deg)
{
    if (!g_init) return ODOM_ERR_NOT_INIT;
    if (x_mm == NULL || y_mm == NULL || theta_deg == NULL)
        return ODOM_ERR_BAD_ARG;
    *x_mm = g_x; *y_mm = g_y; *theta_deg = g_th;
    return ODOM_OK;
}

uint8_t odom_is_init(void) { return g_init; }
