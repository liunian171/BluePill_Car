#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stddef.h>  /* NULL */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  PID 控制器 — 通用驱动模块
 *
 *  架构对标 pwm.h / encoder.h，使用句柄 + 参数分离模式。
 *  每个控制环创建一个 PID_Handle 实例，支持级联嵌套。
 *
 *  使用示例：
 *     PID_Handle speed_l = {
 *         .mode = PID_MODE_POSITIONAL,
 *         .params = { .kp = 1.0f, .ki = 0.1f, .kd = 0.0f,
 *                     .out_min = -1000, .out_max = 1000,
 *                     .integral_limit = 500 },
 *     };
 *
 *     float output = pid_update(&speed_l, target, measurement, dt);
 * ============================================================================ */

/* ============================================================================
 *  PID 计算模式
 * ============================================================================ */
typedef enum {
    PID_MODE_POSITIONAL,   /* 位置式 PID：output = Kp*e + Ki*∫e + Kd*de/dt   */
    PID_MODE_INCREMENTAL,  /* 增量式 PID：output += Δ(Kp*e + Ki*e + Kd*Δe)   */
} PID_Mode_t;

/* ============================================================================
 *  PID 参数（可在运行时通过指针修改）
 * ============================================================================ */
typedef struct {
    float kp;               /* 比例系数                    */
    float ki;               /* 积分系数                    */
    float kd;               /* 微分系数                    */
    float out_min;          /* 输出下限（防失控）          */
    float out_max;          /* 输出上限                    */
    float integral_limit;   /* 积分限幅（防积分饱和）      */
} PID_Params_t;

/* ============================================================================
 *  PID 内部状态（由 pid_update 自动维护）
 * ============================================================================ */
typedef struct {
    float integral;
    float prev_error;
    float prev_output;
    float prev_measurement;
} PID_State_t;

/* ============================================================================
 *  PID 句柄 — 每个控制环一个实例
 * ============================================================================ */
typedef struct PID_Handle {
    PID_Mode_t   mode;              /* 计算模式              */
    PID_Params_t params;            /* 参数（可外部修改）    */
    PID_State_t  state;             /* 内部状态              */

    struct PID_Handle *cascade_outer;  /* 级联：外层 PID 句柄
                                        * 启用后 pid_update 先调外层，
                                        * 外层输出作为本层的目标值。
                                        * NULL = 无级联       */
} PID_Handle;

/* ============================================================================
 *  策略层函数声明
 * ============================================================================ */

/**
 * @brief  初始化 PID 句柄（重置状态）
 * @param  pid    PID 句柄指针
 * @param  mode   计算模式
 * @param  params 参数指针（pid 内部只存值，不持有指针）
 */
void pid_init(PID_Handle *pid, PID_Mode_t mode, PID_Params_t *params);

/**
 * @brief  PID 计算一次
 * @param  pid          PID 句柄
 * @param  setpoint     目标值
 * @param  measurement  当前测量值
 * @param  dt           上次调用到这次的时间间隔（秒）
 * @return float        控制输出
 *
 * @note  启用了 cascade_outer 时，setpoint 被忽略，
 *        实际目标值 = cascade_outer->pid_update() 的输出。
 */
float pid_update(PID_Handle *pid, float setpoint, float measurement, float dt);

/**
 * @brief  重置 PID 内部状态（积分清零、历史值清零）
 */
void pid_reset(PID_Handle *pid);

/**
 * @brief  运行时修改 PID 参数
 */
void pid_set_gains(PID_Handle *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
