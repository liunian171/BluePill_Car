/**
 * @file    line_follower.c
 * @brief   循迹模块 — 5路灰度 + 多阶段状态机 + 三层阻尼收敛
 *
 * 黑线宽度 < 传感器间距，居中时仅 S3 亮。
 *
 * 状态机阶段：
 *   FOLLOW  — 巡线（1A~1F 子阶段：居中/微偏/偏离/严重/回中/过冲）
 *   LINE_EXIT  — 直角弯前刹车 200ms
 *   TURNING — 单侧轮驱动转弯，全白不等超时
 *   SEARCH  — 左右扫描搜索
 *
 * 设计文档：巡线逻辑设计文档.md
 */

#include "line_follower.h"
#include "main.h"               /* OUT1~OUT5 */

/* ==========================================================================
 *  直角判定返回值
 * ========================================================================== */
enum {
    CORNER_NONE    = 0,  /* 不是直角 */
    CORNER_RIGHT   = 1,  /* 右直角 */
    CORNER_LEFT    = 2,  /* 左直角 */
    LOST_SIMPLE    = 3,  /* 追线丢线（不是直角，进 SEARCH） */
};

/* ==========================================================================
 *  标定常量（预留，当前未在状态机中使用）
 * ========================================================================== */
static int32_t g_straight_cnt = 690;
static int32_t g_turn_cnt     = 1060;

/* ==========================================================================
 *  静态状态变量
 * ========================================================================== */
static uint8_t  g_enabled   = 0;
static float    g_base_spd  = 30.0f;
static float    g_turn_spd  = 60.0f;
static float    g_kp        = 12.0f;   /* init 中覆盖为 6.0 */
static float    g_kd        = 8.0f;    /* init 中覆盖为 2.0 */
static float    g_last_pos  = 0;       /* EMA 平滑位置 */
static int8_t   g_invert    = 0;
static uint8_t  g_auto      = 1;

/* 状态机 */
static uint8_t  g_state     = LINE_FOLLOW;
static uint32_t g_tick      = 0;
static int8_t   g_turn_dir  = 0;
static uint32_t g_cool_end  = 0;
static float    g_pos_prev  = 0;       /* PD D 项用 */

/* 传感器：当前帧 + 上一帧 */
static int g_s1, g_s2, g_s3, g_s4, g_s5, g_sum;
static int g_ps1, g_ps2, g_ps3, g_ps4, g_ps5;

/* 当前车动作 */
static int8_t  g_action     = 0;       /* -1=左转, 0=直行, 1=右转 */
static float   g_last_corr  = 0.0f;    /* 上一帧 corr，回中反向阻尼用 */

/* 三层阻尼 */
static float   g_decay      = 1.0f;    /* 修正衰减系数 */
static uint8_t g_brake_cnt  = 0;       /* 回中刹车剩余帧数 */

/* 迟滞计数器 */
static uint8_t g_center_cnt = 0;       /* 连续居中帧数 */
static uint8_t g_dev_cnt    = 0;       /* 连续偏离帧数 */
static uint8_t g_osc_cnt    = 0;       /* 震荡帧数（5 帧内换向次数） */

/* 诊断回调 */
static line_event_cb_t g_event_cb = 0;

/* ==========================================================================
 *  辅助函数声明
 * ========================================================================== */
static void notify(const char *msg) { if (g_event_cb) g_event_cb(msg); }
static void save_prev_sensors(void);
static int  judge_corner(void);
static int  is_center(void);
static void update_action(float corr);
static void apply_reverse_brake(void);
static void apply_overshoot_decay(void);
static void recover_decay(void);
static float compute_pd_corr(void);

/* ==========================================================================
 *  公开接口 — 初始化
 * ========================================================================== */
void line_follower_init(float base_spd, float kp)
{
    (void)kp;
    g_base_spd   = base_spd;
    g_kp         = 6.0f;
    g_kd         = 2.0f;
    g_state      = LINE_FOLLOW;
    g_tick       = 0;
    g_turn_dir   = 0;
    g_last_pos   = 0.0f;
    g_pos_prev   = 0.0f;
    g_decay      = 1.0f;
    g_brake_cnt  = 0;
    g_center_cnt = 0;
    g_dev_cnt    = 0;
    g_osc_cnt    = 0;
    g_action     = 0;
    g_last_corr  = 0.0f;
    g_ps1 = g_ps2 = g_ps3 = g_ps4 = g_ps5 = 0;
}

/* ==========================================================================
 *  公开接口 — getter/setter
 * ========================================================================== */
void line_follower_set_event_cb(line_event_cb_t cb)    { g_event_cb = cb; }
void line_follower_enable(uint8_t en)                   { g_enabled = en; if (!en) g_state = LINE_FOLLOW; }
void line_follower_set_auto(uint8_t en)                 { g_auto = en; }
void line_follower_set_speed(float rpm)                  { g_base_spd = rpm; }
void line_follower_set_turn_speed(float rpm)             { g_turn_spd = rpm; }
void line_follower_set_kp(float kp)                      { g_kp = kp; }
void line_follower_set_kd(float kd)                      { g_kd = kd; }
void line_follower_invert(void)                          { g_invert = !g_invert; }
void line_follower_set_straight_cnt(int32_t c)           { g_straight_cnt = c; }
void line_follower_set_turn_cnt(int32_t c)               { g_turn_cnt = c; }

uint8_t  line_follower_enabled(void)     { return g_enabled; }
uint8_t  line_follower_auto(void)        { return g_auto; }
float    line_follower_base_spd(void)    { return g_base_spd; }
float    line_follower_turn_spd(void)    { return g_turn_spd; }
float    line_follower_kp(void)          { return g_kp; }
float    line_follower_kd(void)          { return g_kd; }
int8_t   line_follower_inverted(void)    { return g_invert; }
uint8_t  line_follower_state(void)       { return g_state; }
int8_t   line_follower_turn_dir(void)    { return g_turn_dir; }
int32_t  line_follower_straight_cnt(void){ return g_straight_cnt; }
int32_t  line_follower_turn_cnt(void)    { return g_turn_cnt; }

/* ==========================================================================
 *  辅助函数实现
 * ========================================================================== */

/**
 * @brief 保存当前帧到 prev 帧
 */
static void save_prev_sensors(void)
{
    g_ps1 = g_s1; g_ps2 = g_s2; g_ps3 = g_s3;
    g_ps4 = g_s4; g_ps5 = g_s5;
}

/**
 * @brief 判断是否是居中状态 (仅 S3 亮)
 */
static int is_center(void)
{
    return (g_s1 == 0 && g_s2 == 0 && g_s3 == 1 && g_s4 == 0 && g_s5 == 0);
}

/**
 * @brief 更新 g_action（由 corr 符号决定）
 */
static void update_action(float corr)
{
    if      (corr > 0.5f)  g_action =  1;
    else if (corr < -0.5f) g_action = -1;
    else                   g_action =  0;
}

/**
 * @brief 三层阻尼 — 回中反向刹车：记录当前 corr，设 brake_cnt=2
 */
static void apply_reverse_brake(void)
{
    g_brake_cnt = 2;
    /* g_last_corr 保持当前值，接下来 2 帧用它的反向值 */
    update_action(-g_last_corr * 0.5f);
}

/**
 * @brief 三层阻尼 — 过冲衰减：g_decay *= 0.5
 */
static void apply_overshoot_decay(void)
{
    g_decay *= 0.5f;
    notify("LINE:OVERSHOOT");
}

/**
 * @brief 三层阻尼 — 衰减恢复：连续居中 20 帧（1 秒）后恢复满力
 */
static void recover_decay(void)
{
    if (g_center_cnt >= 20 && g_decay < 1.0f) {
        g_decay = 1.0f;
    }
}

/**
 * @brief PD 修正计算（含衰减系数）
 */
static float compute_pd_corr(void)
{
    float pos   = g_last_pos;
    float d_pos = pos - g_pos_prev;
    g_pos_prev  = pos;
    return (g_kp * pos + g_kd * d_pos) * g_decay;
}

/**
 * @brief 直角判定 — 根据 00000 前一帧图案
 * @return CORNER_RIGHT / CORNER_LEFT / LOST_SIMPLE / CORNER_NONE
 *
 * 右直角：前一刻右侧(S4或S5)有黑线 且 最左侧(S1)全白
 *   → 黑线从右侧边界消失，线往右拐
 * 左直角：前一刻左侧(S1或S2)有黑线 且 最右侧(S5)全白
 *   → 黑线从左侧边界消失，线往左拐
 *
 * 覆盖的合法路径（以右直角为例）：
 *   居中接近:   00111→00011→00001→00000  ← 最后 S5
 *   微偏接近:   00110→00011→00001→00000  ← 最后 S5
 *   偏接近:     00110→00010→00000        ← 最后 S4（之前漏掉！）
 *   带偏移接近: 01111→00011→00000        ← 最后 S4+S5
 */
static int judge_corner(void)
{
    /* 11111 全黑 → 直行通过 */
    if (g_ps1 && g_ps2 && g_ps3 && g_ps4 && g_ps5) return CORNER_NONE;

    /* 右直角：最左侧(S1)全白 + 右侧(S4或S5)有黑线 */
    if (g_ps1 == 0 && (g_ps4 == 1 || g_ps5 == 1)) return CORNER_RIGHT;

    /* 左直角：最右侧(S5)全白 + 左侧(S1或S2)有黑线 */
    if (g_ps5 == 0 && (g_ps1 == 1 || g_ps2 == 1)) return CORNER_LEFT;

    /* 居中丢线（00100）或过渡态（01100/00110/01110）→ 追线丢线 */
    return LOST_SIMPLE;
}

/* ==========================================================================
 *  传感器读取（含 prev 保存 + 迟滞计数）
 * ========================================================================== */
static void line_read_sensors(void)
{
    save_prev_sensors();

    g_s1 = HAL_GPIO_ReadPin(OUT1_GPIO_Port, OUT1_Pin) ? 1 : 0;
    g_s2 = HAL_GPIO_ReadPin(OUT2_GPIO_Port, OUT2_Pin) ? 1 : 0;
    g_s3 = HAL_GPIO_ReadPin(OUT3_GPIO_Port, OUT3_Pin) ? 1 : 0;
    g_s4 = HAL_GPIO_ReadPin(OUT4_GPIO_Port, OUT4_Pin) ? 1 : 0;
    g_s5 = HAL_GPIO_ReadPin(OUT5_GPIO_Port, OUT5_Pin) ? 1 : 0;

    if (g_invert) {
        g_s1 = !g_s1; g_s2 = !g_s2; g_s3 = !g_s3;
        g_s4 = !g_s4; g_s5 = !g_s5;
    }

    g_sum = g_s1 + g_s2 + g_s3 + g_s4 + g_s5;
    if (g_sum > 0) {
        float raw_pos = (float)(-2 * g_s1 - g_s2 + g_s4 + 2 * g_s5) / (float)g_sum;
        g_last_pos = g_last_pos * 0.4f + raw_pos * 0.6f;
    }

    /* 迟滞计数器维护 */
    if (is_center()) { g_center_cnt++; g_dev_cnt = 0; }
    else             { g_dev_cnt++;     g_center_cnt = 0; }

    /* 衰减恢复 */
    recover_decay();
}

/* ==========================================================================
 *  核心：每 50ms 调用
 * ========================================================================== */
void line_follower_update(uint32_t now_ms,
                          float *spd_target, int8_t *spd_dir,
                          int32_t enc0, int32_t enc1)
{
    (void)enc0; (void)enc1;
    if (!g_enabled) return;

    line_read_sensors();

    switch (g_state) {

    /* ================================================================== */
    /*  FOLLOW — 巡线主循环（§1A~§1F 子阶段）                              */
    /* ================================================================== */
    case LINE_FOLLOW: {
        spd_dir[0] = 1; spd_dir[1] = 1;

        /* ── 直角弯判定（非冷却期 + 全白出现）── */
        int in_cooldown = (now_ms < g_cool_end);
        if (!in_cooldown && g_sum == 0) {
            int turn = judge_corner();
            if (turn == CORNER_RIGHT) {
                notify("LINE:CRN R");
                g_turn_dir = 1;
                g_state = LINE_EXIT;
                g_tick  = now_ms;
                break;
            } else if (turn == CORNER_LEFT) {
                notify("LINE:CRN L");
                g_turn_dir = -1;
                g_state = LINE_EXIT;
                g_tick  = now_ms;
                break;
            } else if (turn == LOST_SIMPLE) {
                notify("LINE:LOST");
                g_state = LINE_SEARCH;
                g_tick  = now_ms;
                break;
            }
            /* CORNER_NONE (11111) → 直行 */
        }

        /* ── PD 修正计算 ── */
        float corr = compute_pd_corr();

        /* ── §1E 回中反向阻尼执行中 ── */
        if (g_brake_cnt > 0) {
            corr = -(g_last_corr) * 0.5f;
            g_brake_cnt--;
            if (g_brake_cnt == 0) {
                corr = 0.0f;
                g_action = 0;
            }
        }
        /* ── §1F 过冲检测 ── */
        else if (g_last_corr > 1.0f && corr < -1.0f) {
            /* 上帧右转、这帧要左转 → 过冲！ */
            apply_overshoot_decay();
            corr = corr * 0.5f;
        }
        else if (g_last_corr < -1.0f && corr > 1.0f) {
            /* 上帧左转、这帧要右转 → 过冲！ */
            apply_overshoot_decay();
            corr = corr * 0.5f;
        }
        /* ── §1E 回中刹车：S3 亮 + 车正在转 → 立即反向阻尼 ──
         * 不等迟滞 2 帧。线到 S3 的第一帧就触发，消除 PID 差速残留。
         * 迟滞只防"边界抖动启动修正"，不防"回中刹车"。 */
        else if (g_s3 && g_action != 0) {
            apply_reverse_brake();
            corr = -(g_last_corr) * 0.5f;
            g_center_cnt = 0;
        }
        /* ── 迟滞未满：偏离/回中不足 2 帧 → 不修正 ── */
        else if (g_dev_cnt < 2 && g_center_cnt < 2) {
            corr = 0.0f;
            g_action = 0;
        }

        /* 输出 */
        g_last_corr = corr;
        update_action(corr);
        spd_target[0] = g_base_spd + corr;
        spd_target[1] = g_base_spd - corr;
        if (spd_target[0] < 0.0f) spd_target[0] = 0.0f;
        if (spd_target[1] < 0.0f) spd_target[1] = 0.0f;
        } break;

    /* ================================================================== */
    /*  LINE_EXIT — 直角弯前刹车 200ms                                    */
    /* ================================================================== */
    case LINE_EXIT:
        spd_target[0] = 1.0f;
        spd_target[1] = 1.0f;
        spd_dir[0] = 0;
        spd_dir[1] = 0;
        g_action = 0;
        if (now_ms - g_tick >= 200) {
            g_state = LINE_TURNING;
            g_tick  = now_ms;
            g_pos_prev = 0.0f;   /* 防止退出时 d_pos 伪跳变 */
            g_last_pos = 0.0f;
        }
        break;

    /* ================================================================== */
    /*  TURNING — 单侧轮驱动转弯，全白不等超时，黑线出现交 FOLLOW 自然接管  */
    /* ================================================================== */
    case LINE_TURNING:
        spd_target[0] = g_turn_spd;
        spd_target[1] = g_turn_spd;
        if (g_turn_dir > 0) {
            spd_dir[0] = 1;
            spd_dir[1] = 0;
            g_action   = 1;
        } else {
            spd_dir[0] = 0;
            spd_dir[1] = 1;
            g_action   = -1;
        }

        /* 保持 EMA 更新，退出时 PD 能自然接管 */
        g_pos_prev = g_last_pos;

        /* 优先级 1：任意传感器看到黑线 + 已转弯 ≥ 500ms → 交 FOLLOW 接管 */
        if (g_sum > 0 && (now_ms - g_tick > 500)) {
            notify("LINE:FOUND");
            spd_dir[0] = 1; spd_dir[1] = 1;
            g_state    = LINE_FOLLOW;
            g_cool_end = now_ms + 800;
            /* 不设 g_brake_cnt，FOLLOW 的传感器逻辑自然决定修正方向和力度 */
        }
        /* 优先级 2：真超时 3s 仍全白 → 搜索 */
        else if (g_sum == 0 && (now_ms - g_tick > 3000)) {
            notify("LINE:SRCH");
            spd_dir[0] = 1; spd_dir[1] = 1;
            g_state = LINE_SEARCH;
            g_tick  = now_ms;
            g_action = g_turn_dir;
        }
        /* 优先级 3：全白 → 正常，继续转（不等超时） */
        break;

    /* ================================================================== */
    /*  SEARCH — 左右扫描搜索                                              */
    /* ================================================================== */
    case LINE_SEARCH: {
        if (g_sum > 0) {
            notify("LINE:FOUND");
            spd_dir[0] = 1; spd_dir[1] = 1;
            g_state    = LINE_FOLLOW;
            g_cool_end = now_ms + 500;
            /* 复用 §1E：找线回归 ≠ 稳定居中 */
            g_brake_cnt = 2;
            g_last_corr = (g_action > 0) ? 4.0f : -4.0f;
            g_action = 0;
        } else {
            uint32_t phase = (now_ms - g_tick) % 800;
            if (phase < 400) {
                spd_target[0] = g_base_spd * 0.8f;
                spd_target[1] = g_base_spd * 1.2f;
                g_action = 1;
            } else {
                spd_target[0] = g_base_spd * 1.2f;
                spd_target[1] = g_base_spd * 0.8f;
                g_action = -1;
            }
            spd_dir[0] = 1; spd_dir[1] = 1;
            if (now_ms - g_tick > 4000) {
                spd_target[0] = 0.0f; spd_target[1] = 0.0f;
                g_action = 0;
            }
        }
    } break;
    }
}

/* ==========================================================================
 *  自动启动
 * ========================================================================== */
void line_follower_try_auto_start(uint8_t cal_ok, uint32_t now_ms)
{
    static uint32_t cal_done_tick = 0;

    if (!g_auto || g_enabled || !cal_ok) { cal_done_tick = 0; return; }
    if (cal_done_tick == 0) cal_done_tick = now_ms;
    if (now_ms - cal_done_tick >= 1000) {
        g_enabled = 1;
        g_state   = LINE_FOLLOW;
    }
}
