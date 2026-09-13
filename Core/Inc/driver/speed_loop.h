#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

/**
 * ============================================================================
 *  执行组件 — speed_loop（速度环）：运动意图(转速) → 器件级目标
 * ============================================================================
 *
 *  层位（自顶向下，详见 doc/代码风格与模块衔接指南 §1.2）：
 *    组装层        main.c            实例创建 / 命令转发 / 接管权仲裁 / 遥测节拍
 *    决策组件      line_follower     传感器 → 运动意图（写目标）
 *    执行组件      speed_loop        ← 本文件：意图(带符号 RPM) → 器件级目标(带符号 RPM)
 *    桥接层        motor_bridge      器件级目标 → 协议（C 门面，id 查表）
 *    执行对象层    Motor             目标 RPM → 转动方向 + 千分比
 *    器件层        TB6612Protocol   千分比 → IN1/IN2 电平 + PWM 占空比
 *    驱动层        pwm / usergpio    寄存器写（唯一碰 HAL 的平台侧）
 *
 *  ▸ 本组件职责（五件）◂
 *    1) 测速     编码器原始计数 → 带符号 RPM（回绕校正 + 反馈符号）
 *    2) 闭环     增量式 PID + 前馈，工作域 = **带符号 RPM**（方向由符号表达）
 *    3) 限幅     输出钳位到 [out_min, out_max]
 *    4) 停车语义 |目标| < SPEED_LOOP_MIN_RUN_RPM → 释放 PID + 物理刹停（"0 速必停"）
 *    5) 状态保持 目标/实际/输出/运行标志，供遥测与显示读取（不再外泄到组装层 static）
 *
 *  ▸ 接口面板（"交界不拥有"，指南 §1.2）◂
 *    意图入：speed_loop_set_target（带符号 RPM）/ speed_loop_stop
 *    配置入：speed_loop_init（标定表一次性注入）/ set_gains / set_ff_gain
 *    输出注入：speed_loop_io_t（组装层绑定到桥接层与编码器；本组件**不 include** 桥接层头）
 *    状态出：speed_loop_get_state / get_target / get_gains / get_ff_gain
 *    本组件不持有 Motor 实例、不直调 HAL、不调 HAL_GetTick。
 *
 *  ▸ 单位与坐标域声明（义务 8）◂
 *    本组件对外接口转速 = **带符号 RPM（电机轴）**：正 = 前进，负 = 后退。
 *    千分比/占空比/方向电平/脉宽等**器件级**表达全部在桥接层以下，本组件不关心
 *    （换驱动芯片、换电机只改配置，不动本文件）。
 *
 *  ▸ 时间基准（义务 5）◂
 *    tick 由调用方传入（`now_ms`），节拍与调度归组装层；本组件不读时钟。
 *    内部保留"上次测速时刻"，仅用于算 dt，不构成超时/看门狗逻辑
 *    （无命令看门狗属 P1，届时按同一约定由组装层判定后调用 speed_loop_stop）。
 *
 *  ▸ 错误通道（义务 6）◂
 *    统一返回码 speed_loop_ret_t；未初始化 / id 越界 / 非有限值一律返回非 OK，
 *    绝不静默。**目标值不做上限钳位**（超出输出上限时由 PID 输出限幅自然饱和，
 *    与原实现语义一致），只拒绝 NaN/Inf。
 *
 *  ▸ 依赖白名单 ◂
 *    仅 <stdint.h> + "pid.h"（通用算法层，纯数学无 HAL）→ 满足义务 7（PC 桩可编译）
 * ============================================================================
 */

#include <stdint.h>

/* ---- 通道上限（本车 2，留裕量给四轮平台）---- */
#define SPEED_LOOP_MAX_CH        4

/* ---- 停车语义阈值：|目标| < 此值视为"0 速"（沿用原 main.c `spd_target < 1.0f`）---- */
#define SPEED_LOOP_MIN_RUN_RPM   1.0f

/* ---- 返回码（错误通道；模块内保持一致）---- */
typedef enum {
    SPEED_LOOP_OK           = 0,   /* 调用成功 */
    SPEED_LOOP_ERR_NOT_INIT = 1,   /* 组件未初始化 */
    SPEED_LOOP_ERR_BAD_ARG  = 2,   /* 入参非法（id 越界 / 非有限值 / dt 超界已重同步） */
    SPEED_LOOP_ERR_BAD_CFG  = 3,   /* 初始化配置非法（拒绝初始化） */
} speed_loop_ret_t;

/* ============================================================================
 *  注入通道（组装层绑定；本组件只持函数指针，不认识 motor_bridge / encoder）
 * ============================================================================ */

/* 下发器件级目标（带符号 RPM）→ 组装层绑定 motor_bridge_set_speed_rpm */
typedef void    (*speed_loop_output_fn)(uint8_t motor_id, float rpm);

/* 物理刹停 → 组装层绑定 motor_bridge_brake */
typedef void    (*speed_loop_brake_fn)(uint8_t motor_id);

/* 读编码器原始计数 → 组装层绑定 encoder_get_count */
typedef int32_t (*speed_loop_enc_fn)(uint8_t motor_id);

typedef struct {
    speed_loop_output_fn set_rpm;   /* 必填 */
    speed_loop_brake_fn  brake;     /* 必填（0 速必停语义依赖它） */
    speed_loop_enc_fn    read_enc;  /* 必填（测速依赖它） */
} speed_loop_io_t;

/* ============================================================================
 *  配置（每通道一份；全部为标定/硬件知识，"车知识"）
 * ============================================================================ */
typedef struct {
    float   ppr;        /* 编码器每转脉冲数（PPR），>0 */
    int32_t enc_span;   /* 编码器计数模值：16 位定时器 = 65536；0 = 不做回绕校正 */
    int8_t  fb_sign;    /* 反馈符号：**必须镜像驱动侧硬件取反**（本车 id1 = -1，
                         * E2 硬件接反；两侧不一致 → 正反馈飞车，真机实证 2026-09-13） */
    float   ff_gain;    /* 前馈系数（在线可调，理论值 1/K） */
    float   out_min;    /* 输出下限（本车 -319 = -max_rpm） */
    float   out_max;    /* 输出上限（本车 +319 = +max_rpm） */
    float   kp, ki, kd; /* 增量式 PID 增益 */
} speed_loop_ch_cfg_t;

typedef struct {
    uint8_t             ch_count;   /* 通道数，1..SPEED_LOOP_MAX_CH */
    speed_loop_ch_cfg_t ch[SPEED_LOOP_MAX_CH];
} speed_loop_cfg_t;

/* ============================================================================
 *  状态读出（遥测 / 显示 / PC 桩断言共用）
 * ============================================================================ */
typedef struct {
    float   target_rpm;   /* 带符号目标（意图），+ 前进 / − 后退 */
    float   actual_rpm;   /* 带符号实测（最近一次有效测速） */
    float   out_rpm;      /* 实际下发的值（PID + 前馈，限幅后）；停车时为 0 */
    uint8_t running;      /* 1 = 闭环运行；0 = 停车/刹停（0 速语义或内轮停车） */
} speed_loop_state_t;

/* ============================================================================
 *  接口
 * ============================================================================ */

/* 初始化：注入标定表与 IO 通道；失败（NULL / ch_count 越界 / ppr<=0 / out_min>out_max）
 * 返回 ERR_BAD_CFG 或 ERR_BAD_ARG 且不置位。成功后所有通道目标 = 0（不发车）。 */
speed_loop_ret_t speed_loop_init(const speed_loop_cfg_t *cfg, const speed_loop_io_t *io);

/* 节拍驱动：每 50ms 调用一次（节拍由组装层决定）。逐通道执行
 * 测速 → PID+前馈 → 限幅 → 下发；|目标| < MIN_RUN_RPM 的通道改走"释放 PID + 物理刹停"。 */
speed_loop_ret_t speed_loop_update(uint32_t now_ms);

/* 测速（带符号 RPM）：维护内部"上次计数/时刻"。dt 超界时返回 ERR_BAD_ARG
 * 并已重新同步基准（该帧不产出有效值）。阶跃测试（开环）复用本函数，
 * 保证"闭环与开环测速同源"，杜绝两处回绕/符号逻辑分叉。 */
speed_loop_ret_t speed_loop_measure(uint8_t id, uint32_t now_ms, float *rpm_out);

/* 重同步测速基准（阶跃开始 / 接管瞬间调用，避免跨段 dt 失真） */
speed_loop_ret_t speed_loop_resync(uint8_t id, uint32_t now_ms);

/* ---- 意图入 ---- */
speed_loop_ret_t speed_loop_set_target(uint8_t id, float rpm_signed);
speed_loop_ret_t speed_loop_get_target(uint8_t id, float *rpm_out);
/* 零速停车：清目标 + 释放 PID + 物理刹停（BRK / 接管 / 急停用） */
speed_loop_ret_t speed_loop_stop(uint8_t id);
/* 仅清 PID 与测速状态，不动目标与输出（阶跃结束等场景） */
speed_loop_ret_t speed_loop_reset(uint8_t id);

/* ---- 参数入（在线整定）---- */
speed_loop_ret_t speed_loop_set_gains(uint8_t id, float kp, float ki, float kd);
speed_loop_ret_t speed_loop_get_gains(uint8_t id, float *kp, float *ki, float *kd);
speed_loop_ret_t speed_loop_set_ff_gain(uint8_t id, float gain);
speed_loop_ret_t speed_loop_get_ff_gain(uint8_t id, float *gain);

/* ---- 状态出 ---- */
speed_loop_ret_t speed_loop_get_state(uint8_t id, speed_loop_state_t *out);
uint8_t          speed_loop_is_init(void);

#endif /* SPEED_LOOP_H */
