/**
 * @file    line_follower.h
 * @brief   循迹模块 — 5路灰度传感器 + 四阶段状态机（编码器驱动）
 *
 * 策略：
 *   FOLLOW   → 正常追线，比例控制差速转向
 *   STRAIGHT → 丢线后直行（按编码器计数），等轮轴对齐拐角
 *   TURNING  → 原地旋转90°（两轮同速反向），按编码器计数
 *   SEARCH   → 转完前进寻线，找到线恢复跟踪
 *
 * 依赖：
 *   - 5 路 GPIO 传感器 (main.h 中定义 OUT1~OUT5)
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
enum {
    LINE_FOLLOW   = 0,  /* 正常追线 */
    LINE_TURNING  = 2,  /* 原地旋转 */
    LINE_EXIT     = 3,  /* 出弯刹车回正 */
    LINE_SEARCH   = 4,  /* 前进寻线 */
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

/* ---- 查询接口 ---- */
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
