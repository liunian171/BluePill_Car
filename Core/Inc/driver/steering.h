#ifndef STEERING_H
#define STEERING_H

/**
 * ============================================================================
 *  执行组件 — steering（转向）：运动意图 → 器件级目标
 * ============================================================================
 *
 *  层位（自顶向下，详见 doc/代码风格与模块衔接指南 §1.2）：
 *    组装层        main.c            实例创建 / 命令转发 / 接管权仲裁
 *    执行组件      steering          ← 本文件：意图(输出域角) → 器件级目标(物理角)
 *    执行对象层    Servo             物理角 → 千分比（+ 器件行程兜底）
 *    器件层        PWMServoProtocol   千分比 → 脉宽 µs（+ 脉宽兜底）
 *    驱动层        pwm               脉宽 → CCR → 寄存器
 *
 *  ▸ 本组件职责（三件，都是"车知识"）◂
 *    1) 机构行程限位钳位 —— 本车转向连杆可用区间，标定得来（唯一所有者）
 *    2) 坐标域换算       —— 输出域绝对角 → 物理角（+安装偏置 135°）
 *    3) 当前角状态保持   —— 供显示/应答读取（不再外泄到组装层的 static 变量）
 *
 *  ▸ 接口面板（"交界不拥有"，指南 §1.2）◂
 *    意图入：steering_set / steering_nudge / steering_center
 *    配置入：steering_set_limit_min / _limit_max / _center（限位=配置态，设置一次）
 *    输出注入：steering_output_fn（组装层绑定到桥接层；本组件**不 include** 桥接层头）
 *    状态出：steering_get
 *    本组件不持有 Servo 实例、不直调 HAL、不调 HAL_GetTick。
 *
 *  ▸ 单位与坐标域声明（义务 8）◂
 *    本组件对外接口角度 = **输出域绝对角**（°）：直行 = -90，右满舵 = -115，左满舵 = -65
 *    ⚠️ 与执行对象层/器件层的"物理角 [0,270]"**不同域**，换算是本组件职责。
 *
 *  ▸ 时间基准（义务 5）◂
 *    本组件无超时/斜坡/周期逻辑，故**不引入 tick 参数**（避免"预留未用"）。
 *    若将来加转向斜坡或转向模型，再按"tick 由调用方传入"补接口。
 *
 *  ▸ 错误通道（义务 6）◂
 *    统一返回码 steering_ret_t；非法值一律"**钳位保底 + 返回非 OK**"双管，
 *    绝不静默。钳位属正常业务语义（取值经 steering_get 可读），不算错误。
 *
 *  ▸ 依赖白名单 ◂
 *    仅 <stdint.h>（零 HAL、零项目依赖）→ 满足义务 7（PC 桩可编译）
 * ============================================================================
 */

#include <stdint.h>

/* ---- 返回码（错误通道；取值顺序对齐 bridge_ret 家族: OK=0 优先, 其后依次为错误）---- */
typedef enum {
    STEERING_OK          = 0,   /* 调用成功 */
    STEERING_ERR_NOT_INIT = 1,  /* 组件未初始化 */
    STEERING_ERR_BAD_ARG  = 2,  /* 入参非法（已钳位保底，实际生效值见 steering_get/get_limit） */
    STEERING_ERR_BAD_CFG  = 3,  /* 初始化配置非法（拒绝初始化） */
} steering_ret_t;

/* 输出注入：组装层绑定到桥接层（参数：舵机 id、物理角 °） */
typedef void (*steering_output_fn)(uint8_t servo_id, float phys_angle);

/* ---- 配置（全部为标定值，"车知识"）---- */
typedef struct {
    float   lim_min;      /* 输出域下限（右满舵），本项目实测 -115 */
    float   lim_max;      /* 输出域上限（左满舵），本项目实测 -65  */
    float   center;       /* 直行位（输出域），本项目实测 -90 */
    float   phys_offset;  /* 输出域 → 物理角 的安装偏置（本车 135） */
    float   lim_abs;      /* 输出域绝对值上限，用于配置合法性校验（本车 135） */
    uint8_t servo_id;     /* 桥接层舵机 id */
} steering_cfg_t;

/* 初始化：注入配置与输出通道，并按 center 归位一次。
 * 配置非法（lim_abs<=0、lim_min>lim_max）返回 STEERING_ERR_BAD_CFG 且不置位。 */
steering_ret_t steering_init(const steering_cfg_t *cfg, steering_output_fn out);

/* 设置转向角（输出域绝对角）：越界钳位到行程并生效；未初始化返回 NOT_INIT */
steering_ret_t steering_set(float angle);

/* 微步：在当前角基础上 ±delta（SV+ / SV-） */
steering_ret_t steering_nudge(float delta);

/* 回直行位（上电归位 / STOP 急停用） */
steering_ret_t steering_center(void);

/* ---- 限位配置（配置态，不做逐次调用）----
 * 越 lim_abs 或 center 不在行程内 → 钳位保底并返回 BAD_ARG */
steering_ret_t steering_set_limit_min(float lim_min);
steering_ret_t steering_set_limit_max(float lim_max);
steering_ret_t steering_set_center(float center);

/* ---- 状态读出（输出域绝对角）---- */
float steering_get(void);      /* 当前生效角（钳位后） */
float steering_get_lim_min(void);
float steering_get_lim_max(void);
uint8_t steering_is_init(void);

#endif /* STEERING_H */
