/**
 * @file    imu_bridge.h
 * @brief   IMU C 桥接层 — main.c 可调用的 IIMU 接口
 */

#ifndef __IMU_BRIDGE_H__
#define __IMU_BRIDGE_H__

#include <stdint.h>
#include "imu.h"

#define MAX_IMUS 4

typedef enum {
    IMU_MPU6050 = 0,
    // IMU_MPU9250,
} ImuType_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 IMU（创建对象 + init） */
void   imu_bridge_init(uint8_t id, ImuType_t type, void *i2c_handle);
/** @brief 是否初始化成功 */
uint8_t imu_bridge_ready(uint8_t id);

/* ---- 原始值 ---- */
int8_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az);
int8_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz);
float  imu_bridge_accel_scale(uint8_t id);
float  imu_bridge_gyro_scale(uint8_t id);

/* ---- Mahony 滤波（C 接口）---- */
/** @brief 读 IMU → 互补滤波 → 更新内部状态 */
void   imu_bridge_update_filter(uint8_t id);
/** @brief 获取滤波后的欧拉角 */
float  imu_bridge_get_roll(uint8_t id);
float  imu_bridge_get_pitch(uint8_t id);
float  imu_bridge_get_yaw(uint8_t id);

/** @brief 初始校准进度 (0~100, 100=完成) */
uint8_t imu_bridge_cal_progress(uint8_t id);

/** @brief 运行时调整 Mahony 滤波增益 (串口调参用) */
void imu_bridge_set_mahony_gains(uint8_t id, float kp, float ki);

#ifdef __cplusplus
}
#endif

#endif
