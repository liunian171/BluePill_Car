/**
 * ============================================================================
 *  执行组件 — steering 实现（转向：意图 → 器件级目标，纯 C）
 * ============================================================================
 *
 *  数据流：
 *    main.c 命令分发 ──steering_set(输出域角)──▶ [本组件]
 *                                                │ ① 行程限位钳位（车知识）
 *                                                │ ② 域换算 +安装偏置（车知识）
 *                                                │ ③ 状态保持
 *                                                ▼
 *                              steering_output_fn(servo_id, 物理角)
 *                                                ▼
 *                              servo_bridge → Servo → PWMServoProtocol → pwm
 *
 *  设计要点：
 *    ① 零 HAL / 零 HAL_GetTick / 零项目头依赖 → PC 桩可编译（义务 7）
 *    ② 输出走函数指针注入 → 本组件不含桥接层头，执行栈向上不反向依赖（§1.2）
 *    ③ 限位是**配置态**（设置一次），钳位是**执行态**（每次必经）——两者分离（§4 规则 1）
 *    ④ 钳位在本组件内部强制执行 → 调用方绕过也无效（§4 规则 2）
 *
 *  与器件侧两道钳位的关系（各管各的域，不算重复）：
 *    本组件：机构行程 [-115,-65]（输出域，车知识）
 *    Servo ：器件行程 [0,270]（物理角，器件知识）
 *    协议层：器件脉宽 [500,2500]µs（器件知识）
 * ============================================================================
 */

#include "steering.h"

/* ---- 组件内部状态（单实例；执行栈单通道，无需数组化）---- */
static steering_cfg_t     g_cfg;
static steering_output_fn g_out   = 0;
static float              g_cur   = 0.0f;   /* 当前生效角（输出域，已钳位） */
static uint8_t            g_inited = 0;

/* ---- 内部工具 ---- */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float fabsf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 把当前角下发到器件侧：输出域 → 物理角（+安装偏置） */
static void steering_apply(void)
{
    if (g_out) g_out(g_cfg.servo_id, g_cfg.phys_offset + g_cur);
}

/* 限位变更后的统一归一：下限<上限、二者限幅、当前角与直行位拉回区间内 */
static void steering_normalize(void)
{
    float t;
    if (g_cfg.lim_min > g_cfg.lim_max) {          /* 交换 */
        t = g_cfg.lim_min; g_cfg.lim_min = g_cfg.lim_max; g_cfg.lim_max = t;
    }
    g_cfg.lim_min = clampf(g_cfg.lim_min, -g_cfg.lim_abs, g_cfg.lim_abs);
    g_cfg.lim_max = clampf(g_cfg.lim_max, -g_cfg.lim_abs, g_cfg.lim_abs);
    g_cfg.center  = clampf(g_cfg.center,  g_cfg.lim_min, g_cfg.lim_max);
    g_cur         = clampf(g_cur,         g_cfg.lim_min, g_cfg.lim_max);
}

/* ---- 对外接口 ---- */

steering_ret_t steering_init(const steering_cfg_t *cfg, steering_output_fn out)
{
    if (cfg == 0 || out == 0)            return STEERING_ERR_BAD_ARG;
    if (cfg->lim_abs <= 0.0f)            return STEERING_ERR_BAD_CFG;
    if (cfg->lim_min > cfg->lim_max)     return STEERING_ERR_BAD_CFG;
    if (fabsf_local(cfg->lim_min) > cfg->lim_abs ||
        fabsf_local(cfg->lim_max) > cfg->lim_abs) return STEERING_ERR_BAD_CFG;

    g_cfg    = *cfg;
    g_out    = out;
    g_cur    = clampf(cfg->center, cfg->lim_min, cfg->lim_max);
    g_inited = 1;

    steering_apply();          /* 上电即归位（安全铁律：不得输出未定义角） */
    return STEERING_OK;
}

steering_ret_t steering_set(float angle)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;

    g_cur = clampf(angle, g_cfg.lim_min, g_cfg.lim_max);   /* 行程限位（唯一所有者） */
    steering_apply();
    return STEERING_OK;        /* 钳位属正常语义；实际生效值经 steering_get 可读 */
}

steering_ret_t steering_nudge(float delta)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;
    return steering_set(g_cur + delta);
}

steering_ret_t steering_center(void)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;
    return steering_set(g_cfg.center);
}

steering_ret_t steering_set_limit_min(float lim_min)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;

    steering_ret_t ret = STEERING_OK;
    if (fabsf_local(lim_min) > g_cfg.lim_abs) ret = STEERING_ERR_BAD_ARG;

    g_cfg.lim_min = lim_min;
    steering_normalize();
    steering_apply();          /* 新限位立即生效（当前角可能被拉回区间） */
    return ret;
}

steering_ret_t steering_set_limit_max(float lim_max)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;

    steering_ret_t ret = STEERING_OK;
    if (fabsf_local(lim_max) > g_cfg.lim_abs) ret = STEERING_ERR_BAD_ARG;

    g_cfg.lim_max = lim_max;
    steering_normalize();
    steering_apply();
    return ret;
}

steering_ret_t steering_set_center(float center)
{
    if (!g_inited) return STEERING_ERR_NOT_INIT;

    steering_ret_t ret = STEERING_OK;
    /* 直行位必须在行程内，且不超过绝对值上限 */
    if (center < g_cfg.lim_min || center > g_cfg.lim_max) ret = STEERING_ERR_BAD_ARG;
    if (fabsf_local(center) > g_cfg.lim_abs)              ret = STEERING_ERR_BAD_ARG;

    g_cfg.center = clampf(center, g_cfg.lim_min, g_cfg.lim_max);
    return ret;                /* 直行位是"目标"，不立即改变当前角 */
}

float steering_get(void)
{
    return g_cur;
}

float steering_get_lim_min(void)
{
    return g_cfg.lim_min;
}

float steering_get_lim_max(void)
{
    return g_cfg.lim_max;
}

uint8_t steering_is_init(void)
{
    return g_inited;
}
