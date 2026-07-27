/**
 * @file    imu_filter.h
 * @brief   通用 IMU 数据处理 — 换算、姿态、Mahony 滤波
 *
 * 零硬件依赖，纯数学运算，跨芯片复用。
 * Mahony 滤波算法参考开源飞控实现（PX4/ArduPilot），
 * 用四元数 + PI 反馈做加速度计/陀螺仪融合。
 */

#ifndef __IMU_FILTER_H__
#define __IMU_FILTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

class ImuFilter {
public:
    ImuFilter();

    /** @brief 注入芯片参数 */
    void set_accel_scale(float lsb_per_g);
    void set_gyro_scale(float lsb_per_dps);

    /* ---- raw → 物理量 ---- */
    void raw_to_accel(int16_t rx, int16_t ry, int16_t rz,
                      float *ax, float *ay, float *az) const;
    void raw_to_gyro(int16_t rx, int16_t ry, int16_t rz,
                     float *gx, float *gy, float *gz) const;

    /* ---- 姿态（加速度 → roll/pitch，仅初始化用）---- */
    void calc_roll_pitch(float ax, float ay, float az,
                         float *roll, float *pitch) const;

    /**
     * @brief Mahony 滤波 — 四元数 + PI 反馈
     *
     * 原理：
     *   1. 加速度计归一化
     *   2. 从四元数估算重力方向
     *   3. 叉积误差 = 测量重力 × 估算重力  → 角速度修正量
     *   4. PI 反馈修正陀螺仪角速度
     *   5. 修正后角速度积分更新四元数
     *   6. 四元数归一化 → 欧拉角
     *
     * @param gx/gy/gz  陀螺仪物理量 (°/s)
     * @param ax/ay/az  加速度物理量 (g)
     * @param dt_sec    距上次调用间隔 (秒)
     */
    void mahony_filter(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt_sec);

    /** @brief 从加速度计初始化四元数（首次调用 mahony_filter 前调用一次） */
    void init_from_accel(float ax, float ay, float az);

    /** @brief 设置 Mahony 增益（默认 Kp=0.5, Ki=0.0） */
    void set_mahony_gains(float kp, float ki);

    /** @brief 重置滤波器状态 */
    void reset();

    /* ---- 偏置校准 ---- */
    void calibrate_accel_bias(const int16_t *sx, const int16_t *sy,
                              const int16_t *sz, uint16_t count);
    void calibrate_gyro_bias(const int16_t *sx, const int16_t *sy,
                             const int16_t *sz, uint16_t count);

    /* ---- 读取结果 ---- */
    float get_roll()  const { return roll_; }
    float get_pitch() const { return pitch_; }
    float get_yaw()   const { return yaw_; }

    int16_t accel_bias_x() const { return accel_bx_; }
    int16_t accel_bias_y() const { return accel_by_; }
    int16_t accel_bias_z() const { return accel_bz_; }

private:
    float accel_scale_;       ///< LSB/g
    float gyro_scale_;        ///< LSB/(°/s)

    int16_t accel_bx_, accel_by_, accel_bz_;
    int16_t gyro_bx_,  gyro_by_,  gyro_bz_;

    /* ---- Mahony 滤波状态 ---- */
    float q0_, q1_, q2_, q3_;       ///< 四元数
    float integral_fb_x_;           ///< 积分反馈项 (I term)
    float integral_fb_y_;
    float integral_fb_z_;
    float kp_;                      ///< 比例增益
    float ki_;                      ///< 积分增益

    float roll_, pitch_, yaw_;      ///< 姿态角 (度)
};

#endif /* __cplusplus */

#endif /* __IMU_FILTER_H__ */
