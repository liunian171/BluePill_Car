/**
 * @file    pid.c
 * @brief   PID 控制器 — 策略层实现
 *
 *  支持位置式和增量式两种模式，可选级联嵌套。
 *  纯数学计算，不依赖任何硬件，跨平台通用。
 */
#include "pid.h"

/* ============================================================================
 *  工具宏
 * ============================================================================ */
#define CLAMP(x, min, max)  (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

/* ============================================================================
 *  位置式 PID
 *
 *  output = Kp * e + Ki * ∫e + Kd * (e - e_prev) / dt
 *
 *  积分分离：误差过大时停止积分（防积分饱和）
 *  微分先行：用测量值变化代替误差变化（防设定值突变冲击）
 * ============================================================================ */
static float pid_positional(PID_Handle *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    float d_measurement = measurement - pid->state.prev_measurement;

    /* 积分分离：误差在允许范围内才积分 */
    if (error * pid->params.kp < pid->params.out_max * 0.5f)
    {
        pid->state.integral += error * dt;
        /* 积分限幅 */
        pid->state.integral = CLAMP(pid->state.integral,
                                    -pid->params.integral_limit,
                                     pid->params.integral_limit);
    }

    /* 微分项：用 -d(measurement) 代替 d(error)，避免 setpoint 突变冲击 */
    float derivative = -pid->params.kd * d_measurement / dt;

    /* 计算输出 */
    float output = pid->params.kp * error
                 + pid->params.ki * pid->state.integral
                 + derivative;

    /* 保存历史值 */
    pid->state.prev_error = error;
    pid->state.prev_measurement = measurement;

    /* 输出限幅 */
    return CLAMP(output, pid->params.out_min, pid->params.out_max);
}

/* ============================================================================
 *  增量式 PID
 *
 *  Δu = Kp * (e - e_prev) + Ki * e * dt + Kd * (e - 2*e_prev + e_prev2)
 *  output += Δu
 *
 *  优点：输出增量，不会大幅跳变；积分在输出侧自然累积。
 *  缺点：需要保存两个历史误差。
 * ============================================================================ */
static float pid_incremental(PID_Handle *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* 比例项 */
    float p_term = pid->params.kp * (error - pid->state.prev_error);

    /* 积分项 */
    float i_term = pid->params.ki * error * dt;

    /* 微分项 */
    float d_term = pid->params.kd * (error - 2.0f * pid->state.prev_error
                                     + pid->state.prev_measurement) / dt;

    /* 计算增量 */
    float delta = p_term + i_term + d_term;

    /* 新输出 = 旧输出 + 增量 */
    float output = pid->state.prev_output + delta;

    /* 保存历史 */
    pid->state.prev_measurement = pid->state.prev_error;
    pid->state.prev_error       = error;
    pid->state.prev_output      = output;

    /* 输出限幅 */
    return CLAMP(output, pid->params.out_min, pid->params.out_max);
}

/* ============================================================================
 *  公有 API
 * ============================================================================ */

void pid_init(PID_Handle *pid, PID_Mode_t mode, PID_Params_t *params)
{
    pid->mode   = mode;
    pid->params = *params;     /* 拷贝参数，不持有指针 */

    /* 默认积分限幅 = 输出限幅的一半 */
    if (pid->params.integral_limit == 0.0f)
        pid->params.integral_limit = (pid->params.out_max - pid->params.out_min) * 0.5f;

    pid_reset(pid);
}

float pid_update(PID_Handle *pid, float setpoint, float measurement, float dt)
{
    /* 保护 dt 为 0 或负值 */
    if (dt <= 0.0f)
        return pid->state.prev_output;

    /* 级联：先调外层，外层输出作为本层目标值 */
    if (pid->cascade_outer != NULL)
    {
        float outer_output = pid_update(pid->cascade_outer, setpoint, measurement, dt);
        setpoint = outer_output;
    }

    switch (pid->mode)
    {
    case PID_MODE_INCREMENTAL:
        return pid_incremental(pid, setpoint, measurement, dt);
    case PID_MODE_POSITIONAL:
    default:
        return pid_positional(pid, setpoint, measurement, dt);
    }
}

void pid_reset(PID_Handle *pid)
{
    pid->state.integral         = 0.0f;
    pid->state.prev_error       = 0.0f;
    pid->state.prev_output      = 0.0f;
    pid->state.prev_measurement = 0.0f;
}

void pid_set_gains(PID_Handle *pid, float kp, float ki, float kd)
{
    pid->params.kp = kp;
    pid->params.ki = ki;
    pid->params.kd = kd;
}
