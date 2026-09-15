/**
 * ============================================================================
 *  IMU 数据处理 — 换算、姿态、Mahony 滤波（纯 C，common 算法层）
 * ============================================================================
 *
 *  C3 姿态组件下沉（2026-09-15）：原 C++ 类 ImuFilter（imu_filter.cpp）1:1 移植为 C，
 *  与 pid.h / ringbuf.h 同层（纯数学零 HAL，PC 桩可编译）。算法不动——Mahony 四元数
 *  + PI 反馈（参考 Mahony et al. 2008 / PX4 / ArduPilot），halfez=0（IMU-only 无磁力计，
 *  加速度计不得修 yaw，真机实证 2026-09-14）。
 *
 *  归属划分：本文件只做"数学"；校准状态机 / 静止检测 / 漂移跟踪(门限) / dt 处理
 *  在 driver/attitude.c 感知组件内（与 speed_loop→pid 的关系同构）。
 * ============================================================================
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 量程换算（LSB/物理单位，由器件参数表查得） */
    float   accel_scale;                    /* LSB/g，默认 16384 (±2g) */
    float   gyro_scale;                     /* LSB/(°/s)，默认 131 (±250°/s) */

    /* 原始零偏（int16 器件码；由 calibrate_*_bias 写入） */
    int16_t accel_bx, accel_by, accel_bz;
    int16_t gyro_bx,  gyro_by,  gyro_bz;

    /* Mahony 状态 */
    float   q0, q1, q2, q3;                 /* 四元数 */
    float   ifb_x, ifb_y, ifb_z;            /* 积分反馈项 */
    float   kp, ki;                         /* PI 增益（默认 0.5 / 0.0） */

    /* 输出姿态角 (°) */
    float   roll, pitch, yaw;
} imu_filter_t;

/* 初始化：默认量程 ±2g/±250°/s、增益 0.5/0、四元数 = 单位、角度全零 */
void imu_filter_init(imu_filter_t *f);

void imu_filter_set_accel_scale(imu_filter_t *f, float lsb_per_g);
void imu_filter_set_gyro_scale(imu_filter_t *f, float lsb_per_dps);
void imu_filter_set_gains(imu_filter_t *f, float kp, float ki);

/* raw（已减零偏）→ 物理量 */
void imu_filter_raw_to_accel(const imu_filter_t *f, int16_t rx, int16_t ry, int16_t rz,
                             float *ax, float *ay, float *az);
void imu_filter_raw_to_gyro(const imu_filter_t *f, int16_t rx, int16_t ry, int16_t rz,
                            float *gx, float *gy, float *gz);

/* 加速度 → roll/pitch（仅初始化用） */
void imu_filter_calc_roll_pitch(const imu_filter_t *f, float ax, float ay, float az,
                                float *roll, float *pitch);

/* 从加速度初始化四元数（yaw=0；首次 mahony 前调一次） */
void imu_filter_init_from_accel(imu_filter_t *f, float ax, float ay, float az);

/* Mahony 融合一步：gx/gy/gz 单位 °/s，ax/ay/az 单位 g，dt 秒 */
void imu_filter_mahony(imu_filter_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az, float dt);

/* 全状态清零（四元数=单位、积分=0、角度=0；不动量程/增益/零偏） */
void imu_filter_reset(imu_filter_t *f);

/* 零偏校准：静止采样 N 拍取均值（count=1 时传入的即均值本身） */
void imu_filter_calibrate_accel_bias(imu_filter_t *f, const int16_t *sx, const int16_t *sy,
                                     const int16_t *sz, uint16_t count);
void imu_filter_calibrate_gyro_bias(imu_filter_t *f, const int16_t *sx, const int16_t *sy,
                                    const int16_t *sz, uint16_t count);

float imu_filter_get_roll(const imu_filter_t *f);
float imu_filter_get_pitch(const imu_filter_t *f);
float imu_filter_get_yaw(const imu_filter_t *f);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTER_H */
