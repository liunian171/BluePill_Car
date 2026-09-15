/**
 * ============================================================================
 *  IMU 数据处理实现 — Mahony 滤波（纯 C；由 imu_filter.cpp 1:1 移植，C3 2026-09-15）
 * ============================================================================
 *  算法说明见头文件与原 C++ 版注释（Mahony et al. 2008 / PX4 / ArduPilot）。
 *  移植纪律：与 C++ 版逐行等价（含 halfez=0、accel 三档运动分级、deg2rad 积分），
 *  搬迁前后行为差异为零——真机以 ITEL 曲线等价比对背书。
 * ============================================================================
 */

#include "imu_filter.h"
#include <math.h>

/* 度/弧度换算常量（与 C++ 版一致的截断值） */
#define DEG2RAD_F   0.01745329252f    /* π/180 */
#define RAD2DEG_F   57.29578f         /* 180/π */
#define HALF_DEG2RAD 0.00872665f      /* π/360 */

void imu_filter_init(imu_filter_t *f)
{
    f->accel_scale = 16384.0f;
    f->gyro_scale  = 131.0f;
    f->accel_bx = 0; f->accel_by = 0; f->accel_bz = 0;
    f->gyro_bx  = 0; f->gyro_by  = 0; f->gyro_bz  = 0;
    f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
    f->ifb_x = 0.0f; f->ifb_y = 0.0f; f->ifb_z = 0.0f;
    f->kp = 0.5f; f->ki = 0.0f;
    f->roll = 0.0f; f->pitch = 0.0f; f->yaw = 0.0f;
}

void imu_filter_set_accel_scale(imu_filter_t *f, float lsb_per_g) { f->accel_scale = lsb_per_g; }
void imu_filter_set_gyro_scale(imu_filter_t *f, float lsb_per_dps) { f->gyro_scale = lsb_per_dps; }
void imu_filter_set_gains(imu_filter_t *f, float kp, float ki) { f->kp = kp; f->ki = ki; }

void imu_filter_raw_to_accel(const imu_filter_t *f, int16_t rx, int16_t ry, int16_t rz,
                             float *ax, float *ay, float *az)
{
    *ax = (float)(rx - f->accel_bx) / f->accel_scale;
    *ay = (float)(ry - f->accel_by) / f->accel_scale;
    *az = (float)(rz - f->accel_bz) / f->accel_scale;
}

void imu_filter_raw_to_gyro(const imu_filter_t *f, int16_t rx, int16_t ry, int16_t rz,
                            float *gx, float *gy, float *gz)
{
    *gx = (float)(rx - f->gyro_bx) / f->gyro_scale;
    *gy = (float)(ry - f->gyro_by) / f->gyro_scale;
    *gz = (float)(rz - f->gyro_bz) / f->gyro_scale;
}

void imu_filter_calc_roll_pitch(const imu_filter_t *f, float ax, float ay, float az,
                                float *roll, float *pitch)
{
    (void)f;
    *roll  = atan2f(ay, az) * RAD2DEG_F;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG_F;
}

/* ---------------------------------------------------------------------------
 *  Mahony 核心步骤（与 C++ 版逐行对应）：
 *    1. 加速度归一化 + 三档运动分级（剧烈运动弃权防拉偏）
 *    2. 四元数估算重力方向 v，叉积误差 e = â × v
 *    3. halfez = 0 — IMU-only：重力无航向信息，yaw 修正必须清零
 *    4. PI 修正角速度（Ki>0 时累积积分项）
 *    5. deg→rad，四元数积分 q += qdot·dt
 *    6. 归一化 → 欧拉角 (ZYX)
 * --------------------------------------------------------------------------- */
void imu_filter_mahony(imu_filter_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float norm;

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    float accel_gain = 1.0f;

    if (accel_norm < 1e-6f) {
        goto gyro_only;
    }

    if (accel_norm < 0.5f || accel_norm > 1.5f) {
        accel_gain = 0.0f;   /* 剧烈运动 → 完全不信 accel */
    } else if (accel_norm < 0.8f || accel_norm > 1.2f) {
        accel_gain = 0.3f;   /* 中等运动 → 降权 */
    }

    if (accel_gain > 0.0f) {
        recipNorm = 1.0f / accel_norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = f->q1 * f->q3 - f->q0 * f->q2;
        halfvy = f->q0 * f->q1 + f->q2 * f->q3;
        halfvz = f->q0 * f->q0 - 0.5f + f->q3 * f->q3;

        halfex = (ay * halfvz - az * halfvy) * accel_gain;
        halfey = (az * halfvx - ax * halfvz) * accel_gain;
        halfez = 0.0f;   /* IMU-only: yaw 修正清零（IMU-1 前置修复, 真机实证） */
    }

    if (f->ki > 0.0f) {
        f->ifb_x += f->ki * halfex * dt;
        f->ifb_y += f->ki * halfey * dt;
        f->ifb_z += f->ki * halfez * dt;
        gx += f->ifb_x;
        gy += f->ifb_y;
        gz += f->ifb_z;
    } else {
        f->ifb_x = 0.0f;
        f->ifb_y = 0.0f;
        f->ifb_z = 0.0f;
    }

    gx += f->kp * halfex;
    gy += f->kp * halfey;
    gz += f->kp * halfez;

gyro_only:
    {
        float gxr = gx * DEG2RAD_F;
        float gyr = gy * DEG2RAD_F;
        float gzr = gz * DEG2RAD_F;
        float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

        f->q0 += 0.5f * (-q1 * gxr - q2 * gyr - q3 * gzr) * dt;
        f->q1 += 0.5f * ( q0 * gxr + q2 * gzr - q3 * gyr) * dt;
        f->q2 += 0.5f * ( q0 * gyr - q1 * gzr + q3 * gxr) * dt;
        f->q3 += 0.5f * ( q0 * gzr + q1 * gyr - q2 * gxr) * dt;
    }

    norm = sqrtf(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    if (norm < 1e-6f) return;
    recipNorm = 1.0f / norm;
    f->q0 *= recipNorm;
    f->q1 *= recipNorm;
    f->q2 *= recipNorm;
    f->q3 *= recipNorm;

    f->roll  = atan2f(2.0f * (f->q0 * f->q1 + f->q2 * f->q3),
                      1.0f - 2.0f * (f->q1 * f->q1 + f->q2 * f->q2)) * RAD2DEG_F;
    f->pitch = asinf(2.0f * (f->q0 * f->q2 - f->q3 * f->q1)) * RAD2DEG_F;
    f->yaw   = atan2f(2.0f * (f->q0 * f->q3 + f->q1 * f->q2),
                      1.0f - 2.0f * (f->q2 * f->q2 + f->q3 * f->q3)) * RAD2DEG_F;
}

void imu_filter_init_from_accel(imu_filter_t *f, float ax, float ay, float az)
{
    float roll, pitch;
    imu_filter_calc_roll_pitch(f, ax, ay, az, &roll, &pitch);

    float half_roll  = roll  * HALF_DEG2RAD;
    float half_pitch = pitch * HALF_DEG2RAD;

    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);

    /* yaw = 0 → cos(0)=1, sin(0)=0 */
    f->q0 = cp * cr;
    f->q1 = cp * sr;
    f->q2 = sp * cr;
    f->q3 = sp * sr;

    f->ifb_x = 0.0f;
    f->ifb_y = 0.0f;
    f->ifb_z = 0.0f;

    f->roll  = roll;
    f->pitch = pitch;
    f->yaw   = 0.0f;
}

void imu_filter_reset(imu_filter_t *f)
{
    f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
    f->ifb_x = 0.0f;
    f->ifb_y = 0.0f;
    f->ifb_z = 0.0f;
    f->roll = 0.0f; f->pitch = 0.0f; f->yaw = 0.0f;
}

static int16_t filter_mean(const int16_t *samples, uint16_t count)
{
    int32_t sum = 0;
    for (uint16_t i = 0; i < count; i++) sum += samples[i];
    return (int16_t)(sum / count);
}

void imu_filter_calibrate_accel_bias(imu_filter_t *f, const int16_t *sx, const int16_t *sy,
                                     const int16_t *sz, uint16_t count)
{
    f->accel_bx = filter_mean(sx, count);
    f->accel_by = filter_mean(sy, count);
    f->accel_bz = filter_mean(sz, count) - (int16_t)f->accel_scale;  /* 减 1g */
}

void imu_filter_calibrate_gyro_bias(imu_filter_t *f, const int16_t *sx, const int16_t *sy,
                                    const int16_t *sz, uint16_t count)
{
    f->gyro_bx = filter_mean(sx, count);
    f->gyro_by = filter_mean(sy, count);
    f->gyro_bz = filter_mean(sz, count);
}

float imu_filter_get_roll(const imu_filter_t *f)  { return f->roll; }
float imu_filter_get_pitch(const imu_filter_t *f) { return f->pitch; }
float imu_filter_get_yaw(const imu_filter_t *f)   { return f->yaw; }
