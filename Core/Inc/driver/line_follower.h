/**
 * @file    line_follower.h
 * @brief   循迹模块 — 5路灰度传感器 + 四状态机（PD 追线 + 三层阻尼）
 *
 * 状态机（权威设计见 doc/巡线逻辑设计文档.md）：
 *   FOLLOW    → PD 追线 + 三层阻尼（回中反向阻尼 / 过冲衰减 g_decay / 迟滞确认）
 *   LINE_EXIT → 直角弯前刹车 200ms，消除直行动量
 *   TURNING   → 单轮支点转弯（内侧轮刹停，外侧轮驱动）
 *   SEARCH    → 丢线搜索，800ms 周期摆扫，4s 超时停车
 *   回正后进入 800ms 冷却期（禁直角检测，PD 自然接管）
 *
 * 依赖：
 *   - 5 路 GPIO 传感器 (main.h 中定义 OUT1~OUT5)
 *     ⚠️ 当前在 line_follower.c 内直调 HAL_GPIO_ReadPin，未走注入接口
 *        → 违反 AGENTS.md §5.3，PC 桩不可编译（架构评审 C2，待治理）
 *   - 每 50ms 调用一次 line_follower_update()，传入两路编码器计数值
 *   - 输出 spd_target[2] / spd_dir[2] 供 PID 速度环消费
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 状态枚举 ---- */
/* 注: 值 1 为历史占用（旧 STRAIGHT 态已删除），保留空洞以兼容既有日志/记录 */
enum {
    LINE_FOLLOW   = 0,  /* 正常追线 (PD + 三层阻尼) */
    LINE_TURNING  = 2,  /* 单轮支点转弯 */
    LINE_EXIT     = 3,  /* 弯前刹车 200ms */
    LINE_SEARCH   = 4,  /* 丢线搜索 (800ms 摆扫) */
};

/* ---- 初始化 ---- */
void line_follower_init(float base_spd, float kp);

/* ---- 运行时控制 ---- */
/**
 * @brief 每 50ms 调用 → 计算 spd_target[2] 和 spd_dir[2]
 * @param now_ms  系统当前 tick
 * @param spd_target 输出：两轮速度目标值 (RPM)
 * @param spd_dir    输出：两轮方向 (±1)
 * @param enc0  电机 0 当前编码器计数值
 * @param enc1  电机 1 当前编码器计数值
 */
void line_follower_update(uint32_t now_ms,
                          float *spd_target, int8_t *spd_dir,
                          int32_t enc0, int32_t enc1);

void line_follower_enable(uint8_t en);
void line_follower_set_auto(uint8_t en);
void line_follower_set_speed(float rpm);
void line_follower_set_turn_speed(float rpm);  /* 转弯时外侧轮转速 */
void line_follower_set_kp(float kp);
void line_follower_set_kd(float kd);  /* PD 微分增益 */
void line_follower_invert(void);

/** @brief 设置编码器标定值（默认已根据你的实测值设置） */
void line_follower_set_straight_cnt(int32_t cnt);  /* 传感器→轮轴直行编码器计数 */
void line_follower_set_turn_cnt(int32_t cnt);      /* 单轮旋转90°编码器计数 */

/** @brief 注册诊断回调：状态变化时调用，传入描述字符串 */
typedef void (*line_event_cb_t)(const char *msg);
void line_follower_set_event_cb(line_event_cb_t cb);

/** @brief IMU 校准完成后自动启动 */
void line_follower_try_auto_start(uint8_t cal_ok, uint32_t now_ms);

/* ---- 查询接口 ----
 * ⚠️ 预留（2026-09-20 P1-6 登记）：auto/base_spd/turn_spd/kp/kd/state/turn_dir/
 * straight_cnt/turn_cnt 共 9 个 getter 当前零调用（GK/GS 应答走 main.c 回显、
 * 页 4/7 已改作他用）。启用条件 = 巡线功能恢复批次（CAR_FEATURE_LINE_FOLLOWER=1
 * 且感知拆分/实车整定时接回 OLED 或诊断链）；届时期满复核，届时不用则删。
 * 注：line_follower_set_turn_speed 同批同判据。当前整模块处于编译期剔除（stub）。 */
uint8_t line_follower_enabled(void);
uint8_t line_follower_auto(void);
float   line_follower_base_spd(void);
float   line_follower_turn_spd(void);
float   line_follower_kp(void);
float   line_follower_kd(void);
int8_t  line_follower_inverted(void);
uint8_t line_follower_state(void);
int8_t  line_follower_turn_dir(void);
int32_t line_follower_straight_cnt(void);
int32_t line_follower_turn_cnt(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWER_H */
