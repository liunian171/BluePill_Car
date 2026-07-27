/**
 * @file    imu_filter.cpp
 * @brief   IMU 数据处理实现 — Mahony 滤波
 *
 * 参考：
 *   - "Indirect Kalman Filter for 3D Attitude Estimation" (Mahony et al.)
 *   - ArduPilot AP_AHRS / PX4 ECL 的互补/Mahony 实现
 *
 * Mahony 的核心思想：
 *   加速度计测量"重力方向"作为参考，计算与四元数估算的重力方向之间的
 *   叉积误差，通过 PI 反馈修正陀螺仪角速度，再积分更新四元数。
 *
 *   因为没有磁力计，yaw 只能靠陀螺仪积分，仍会缓慢漂移。
 */

#include "imu_filter.h"
#include <math.h>

/* ==========================================================================
 *  构造
 * ========================================================================== */

ImuFilter::ImuFilter()
    : accel_scale_(16384.0f)
    , gyro_scale_(131.0f)
    , accel_bx_(0), accel_by_(0), accel_bz_(0)
    , gyro_bx_(0), gyro_by_(0), gyro_bz_(0)
    , q0_(1.0f), q1_(0), q2_(0), q3_(0)
    , integral_fb_x_(0), integral_fb_y_(0), integral_fb_z_(0)
    , kp_(0.5f), ki_(0.0f)
    , roll_(0), pitch_(0), yaw_(0)
{}

void ImuFilter::set_accel_scale(float lsb_per_g) { accel_scale_ = lsb_per_g; }
void ImuFilter::set_gyro_scale(float lsb_per_dps) { gyro_scale_ = lsb_per_dps; }
void ImuFilter::set_mahony_gains(float kp, float ki) { kp_ = kp; ki_ = ki; }


/* ==========================================================================
 *  raw → 物理量
 * ========================================================================== */

void ImuFilter::raw_to_accel(int16_t rx, int16_t ry, int16_t rz,
                             float *ax, float *ay, float *az) const
{
    *ax = (float)(rx - accel_bx_) / accel_scale_;
    *ay = (float)(ry - accel_by_) / accel_scale_;
    *az = (float)(rz - accel_bz_) / accel_scale_;
}

void ImuFilter::raw_to_gyro(int16_t rx, int16_t ry, int16_t rz,
                            float *gx, float *gy, float *gz) const
{
    *gx = (float)(rx - gyro_bx_) / gyro_scale_;
    *gy = (float)(ry - gyro_by_) / gyro_scale_;
    *gz = (float)(rz - gyro_bz_) / gyro_scale_;
}


/* ==========================================================================
 *  姿态角（加速度 → roll/pitch），仅用于初始化
 * ========================================================================== */

void ImuFilter::calc_roll_pitch(float ax, float ay, float az,
                                float *roll, float *pitch) const
{
    *roll  = atan2f(ay, az) * 57.29578f;   // radians → degrees
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
}


/* ==========================================================================
 *  Mahony 滤波 — 核心算法
 *
 *  公式推导（简化）：
 *    1. 加速度计归一化： â = a / |a|
 *    2. 从当前四元数估算重力参考方向 v
 *       vx = 2*(q1*q3 - q0*q2)
 *       vy = 2*(q0*q1 + q2*q3)
 *       vz = q0² - q1² - q2² + q3²
 *    3. 误差 = â × v (叉积，表示两个方向之间的差异)
 *    4. PI 修正角速度：
 *       ω = ω_raw + Kp × e + Ki × ∫e
 *    5. 四元数导数： qdot = 0.5 × q ⊗ ω
 *    6. 四元数积分： q += qdot × dt
 *    7. 四元数归一化防止数值漂移
 * ========================================================================== */

void ImuFilter::mahony_filter(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt_sec)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0, halfey = 0, halfez = 0;
    float norm;

    /* ================================================================
     *  步骤 1-3：加速度计归一化 + 叉积误差
     *
     *  加速度异常保护：当 |accel| 远离 1g 时降低修正权重，
     *  防止运动加速度把姿态拉偏。
     * ================================================================ */

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    float accel_gain = 1.0f;

    if (accel_norm < 1e-6f) {
        goto gyro_only;
    }

    if (accel_norm < 0.5f || accel_norm > 1.5f) {
        accel_gain = 0.0f;   /* 剧烈运动 → 完全不信 accel */
    } else if (accel_norm < 0.8f || accel_norm > 1.2f) {
        accel_gain = 0.3f;   /* 中等运动 → 降低权重 */
    }

    if (accel_gain > 0.0f) {
        recipNorm = 1.0f / accel_norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1_ * q3_ - q0_ * q2_;
        halfvy = q0_ * q1_ + q2_ * q3_;
        halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        halfex = (ay * halfvz - az * halfvy) * accel_gain;
        halfey = (az * halfvx - ax * halfvz) * accel_gain;
        halfez = (ax * halfvy - ay * halfvx) * accel_gain;
    }

    /* ================================================================
     *  步骤 4：PI 反馈修正陀螺仪角速度
     * ================================================================ */

    if (ki_ > 0.0f) {
        /* 积分项累积（用 ki_ × 叉积代替单独积分增益，简化 */
        integral_fb_x_ += ki_ * halfex * dt_sec;
        integral_fb_y_ += ki_ * halfey * dt_sec;
        integral_fb_z_ += ki_ * halfez * dt_sec;

        gx += integral_fb_x_;
        gy += integral_fb_y_;
        gz += integral_fb_z_;
    } else {
        integral_fb_x_ = 0.0f;
        integral_fb_y_ = 0.0f;
        integral_fb_z_ = 0.0f;
    }

    /* 比例项 */
    gx += kp_ * halfex;
    gy += kp_ * halfey;
    gz += kp_ * halfez;

    /* ================================================================
     *  步骤 5-6：四元数积分
     *
     *  注意：陀螺仪以 °/s 传入并用于 Kp/Ki 修正，
     *        但四元数导数公式 qdot = 0.5 × q × ω 要求 ω 为 rad/s。
     *        此处做 deg→rad 转换。
     * ================================================================ */
gyro_only:
    {
        const float deg2rad = 0.01745329252f;  // π/180
        float gxr = gx * deg2rad;
        float gyr = gy * deg2rad;
        float gzr = gz * deg2rad;

        float dq0 = 0.5f * (-q1_ * gxr - q2_ * gyr - q3_ * gzr) * dt_sec;
        float dq1 = 0.5f * ( q0_ * gxr + q2_ * gzr - q3_ * gyr) * dt_sec;
        float dq2 = 0.5f * ( q0_ * gyr - q1_ * gzr + q3_ * gxr) * dt_sec;
        float dq3 = 0.5f * ( q0_ * gzr + q1_ * gyr - q2_ * gxr) * dt_sec;

        q0_ += dq0;
        q1_ += dq1;
        q2_ += dq2;
        q3_ += dq3;
    }

    /* ================================================================
     *  步骤 7：四元数归一化
     * ================================================================ */
    norm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (norm < 1e-6f) return;
    recipNorm = 1.0f / norm;
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;

    /* ================================================================
     *  四元数 → 欧拉角
     *
     *  旋转顺序：ZYX (Yaw → Pitch → Roll)
     *  Roll  : φ = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
     *  Pitch : θ = asin(2(q0q2 - q3q1))
     *  Yaw   : ψ = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
     * ================================================================ */
    roll_  = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                    1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * 57.29578f;
    pitch_ = asinf(2.0f * (q0_ * q2_ - q3_ * q1_)) * 57.29578f;
    yaw_   = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                    1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * 57.29578f;
}


/* ==========================================================================
 *  从加速度计初始化四元数（首次调用 mahony_filter 前调用一次）
 *
 *  假设初始 yaw = 0，从 accel 计算 roll/pitch 后转四元数。
 * ========================================================================== */

void ImuFilter::init_from_accel(float ax, float ay, float az)
{
    float roll, pitch;
    calc_roll_pitch(ax, ay, az, &roll, &pitch);

    /* 度 → 弧度 */
    float half_roll  = roll  * 0.00872665f;  /* PI / 360 */
    float half_pitch = pitch * 0.00872665f;

    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);

    /* yaw = 0 → cos(0)=1, sin(0)=0 */
    q0_ = cp * cr;
    q1_ = cp * sr;
    q2_ = sp * cr;
    q3_ = sp * sr;

    /* 清除积分累积 */
    integral_fb_x_ = 0.0f;
    integral_fb_y_ = 0.0f;
    integral_fb_z_ = 0.0f;

    /* 更新输出角度 */
    roll_  = roll;
    pitch_ = pitch;
    yaw_   = 0.0f;
}


/* ==========================================================================
 *  重置滤波器 — 清零所有状态
 * ========================================================================== */

void ImuFilter::reset()
{
    q0_ = 1.0f; q1_ = 0; q2_ = 0; q3_ = 0;
    integral_fb_x_ = 0.0f;
    integral_fb_y_ = 0.0f;
    integral_fb_z_ = 0.0f;
    roll_ = 0; pitch_ = 0; yaw_ = 0;
}


/* ==========================================================================
 *  偏置校准：静止采样 N 次取均值
 * ========================================================================== */

static int16_t mean(const int16_t *samples, uint16_t count)
{
    int32_t sum = 0;
    for (uint16_t i = 0; i < count; i++) sum += samples[i];
    return (int16_t)(sum / count);
}

void ImuFilter::calibrate_accel_bias(const int16_t *sx, const int16_t *sy,
                                     const int16_t *sz, uint16_t count)
{
    accel_bx_ = mean(sx, count);
    accel_by_ = mean(sy, count);
    accel_bz_ = mean(sz, count) - (int16_t)accel_scale_;  // 减 1g
}

void ImuFilter::calibrate_gyro_bias(const int16_t *sx, const int16_t *sy,
                                    const int16_t *sz, uint16_t count)
{
    gyro_bx_ = mean(sx, count);
    gyro_by_ = mean(sy, count);
    gyro_bz_ = mean(sz, count);
}
