#ifndef IMU_BRIDGE_H
#define IMU_BRIDGE_H

/**
 * ============================================================================
 *  桥接层 — imu_bridge（IMU C 门面）
 * ============================================================================
 *
 *  层位：组装层 / 感知组件 → **本文件** → 器件层(MPU6050 → IIMU) → I2C ops
 *  职责：查 id → 校验 → 转发。零业务逻辑、零 I/O、零 HAL。
 *
 *  ▸ 家族契约（正文 doc/桥接层家族契约.md，返回码 bridge_ret.h）◂
 *    · init 用配置结构注入：`imu_bridge_init(id, const imu_bridge_cfg_t*)`
 *    · 动作类返回 bridge_ret_t；取值类返回**安全默认值**（见各函数注释）
 *    · 静态池 + placement new，禁堆 new
 *    · 通道状态一律 per-id（**校准进度/漂移补偿状态按 id 分账**）
 *
 *  ▸ ⚠️ 已知越层（C3 待下沉，本契约不处理业务逻辑）◂
 *    本桥目前是"胖桥"：陀螺零偏校准状态机 + 静止漂移补偿 + 滤波调度
 *    （~90 行）都在桥里，按分层应下沉为**姿态组件**（感知组件，与决策组件同构）。
 *    C1 只统一形式（id / 错误通道 / 静态池 / per-id 状态）并修掉其中一处
 *    真实 bug（原 `cal_progress` 忽略 id），搬逻辑留给 C3。
 *
 *  ▸ 单位 ▸ 欧拉角 °；加速度 g；角速度 °/s；原始值 int16 器件码
 * ============================================================================
 */

#include <stdint.h>
#include "bridge_ret.h"
#include "imu.h"                 /* IIMU 抽象接口 */

#define MAX_IMUS 4

typedef enum {
    IMU_MPU6050 = 0,
    /* IMU_MPU9250, 未实现：init 返回 BRIDGE_ERR_BAD_CFG，不静默 */
} ImuType_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（组装层注入）---- */
typedef struct {
    ImuType_t type;
    void     *i2c_handle;   /* 必填（NULL → BAD_ARG）。本工程为 I2C_Handle* */
} imu_bridge_cfg_t;

/**
 * @brief  初始化一个 IMU 通道（静态池 + placement new + 器件 init）
 * @retval BRIDGE_OK / BAD_ARG（id 越界、cfg/handle 为 NULL）/ BAD_CFG（类型未实现）
 * @note   器件 `init()` 失败时走"克隆芯片兜底配置"路径（沿用原策略），仍返回 OK；
 *         是否真正就绪用 `imu_bridge_ready(id)` 判断
 */
bridge_ret_t imu_bridge_init(uint8_t id, const imu_bridge_cfg_t *cfg);

/* ---- 取值类：越界/未初始化返回安全默认值（注释中标明），不返回未定义值 ---- */
uint8_t imu_bridge_ready(uint8_t id);                     /* 越界/未初始化 → 0 */
float   imu_bridge_accel_scale(uint8_t id);               /* 越界/未初始化 → 1.0f（中性比例，勿当标定值用） */
float   imu_bridge_gyro_scale(uint8_t id);                /* 越界/未初始化 → 1.0f */
float   imu_bridge_get_roll(uint8_t id);                  /* 越界/未初始化 → 0.0f（**不可当真实姿态**） */
float   imu_bridge_get_pitch(uint8_t id);                 /* 同上 */
float   imu_bridge_get_yaw(uint8_t id);                   /* 同上 */
uint8_t imu_bridge_cal_progress(uint8_t id);              /* 0~100；越界 → 0（=未完成，安全侧） */

/* ---- 动作类：全部返回 bridge_ret_t ---- */
bridge_ret_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az);
bridge_ret_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz);
/* 读 → 校准/漂移补偿 → Mahony 融合。**时间基准由调用方传入**（义务 5），
 * 故本桥不含 HAL_GetTick → 全桥家族零 HAL，可上 PC 桩 */
bridge_ret_t imu_bridge_update_filter(uint8_t id, uint32_t now_ms);
bridge_ret_t imu_bridge_set_mahony_gains(uint8_t id, float kp, float ki);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BRIDGE_H */
