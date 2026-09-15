#ifndef ATTITUDE_H
#define ATTITUDE_H

/**
 * ============================================================================
 *  感知组件 — attitude（姿态）：器件裸数 → 物理姿态（roll/pitch/yaw）
 * ============================================================================
 *
 *  层位：driver 组件层（感知栈），与决策组件 line_follower / 执行组件
 *  speed_loop / steering 同构。C3（2026-09-15）从"胖桥" imu_bridge 下沉——
 *  C1 契约划定的边界：桥只做"查 id → 校验 → 转发"，业务逻辑归组件。
 *
 *  ▸ 本组件职责（四件）◂
 *    1) 采集    注入的 raw 读函数 → 经 imu_filter 换算物理量（含零偏）
 *    2) 校准    启动零偏状态机：静稳采集 cal_samples 拍 → 均值写入滤波器
 *               → 加速度计初始化四元数（yaw 归零）
 *    3) 漂移    静止判据(加速度模值) + 陀螺模值门限(双条件)下跟踪零偏漂移，
 *               一阶修正（IMU-1 修复核心：转动中不误吸旋转）
 *    4) 融合    dt 计算/钳位 → imu_filter_mahony → roll/pitch/yaw
 *
 *  ▸ 接口面板（"交界不拥有"）◂
 *    采集注入：attitude_io_t（组装层绑定到器件层；本组件不认识 MPU6050/I2C）
 *    配置注入：attitude_cfg_t（校准拍数/静止阈值/门限/跟踪速率 = 算法知识）
 *    时间外部化：attitude_update(id, now_ms)，本组件不读时钟（义务 5）
 *    状态出：get_roll/pitch/yaw / cal_progress / stable / get_drift
 *
 *  ▸ 依赖白名单（义务 7，PC 桩可编译）◂
 *    仅 <stdint.h> <math.h> + "imu_filter.h"（common 算法层，纯数学零 HAL）
 *
 *  ▸ 单位与坐标域声明（义务 8）◂
 *    对外角度一律 °（roll/pitch/yaw），角速度 °/s，加速度 g，漂移补偿 °/s；
 *    原始值 int16 器件码只在注入边界出现。
 * ============================================================================
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 通道上限（本车 1，留多 IMU 裕量）---- */
#define ATTITUDE_MAX_CH   4

/* ---- 返回码（语义对齐 bridge_ret_t；桥做 1:1 映射）---- */
typedef enum {
    ATTITUDE_OK           = 0,
    ATTITUDE_ERR_NOT_INIT = 1,
    ATTITUDE_ERR_BAD_ARG  = 2,
    ATTITUDE_ERR_BAD_CFG  = 3,
    ATTITUDE_ERR_IO       = 4,
} attitude_ret_t;

/* ---- 配置（算法知识，每通道一份；NULL 成员取默认值）---- */
typedef struct {
    uint16_t cal_samples;      /* 启动零偏校准采样拍数（默认 50 ≈ 5s@10Hz） */
    float    accel_tol_g;      /* 静止判据 |a|−1g 阈值（默认 0.05g） */
    float    gyro_gate_dps;    /* 漂移跟踪陀螺模值门限（默认 1.5°/s，IMU-1 修复） */
    float    drift_rate_slow;  /* 静止 1~5s 漂移跟踪速率（默认 0.02） */
    float    drift_rate_fast;  /* 静止 >5s 漂移跟踪速率（默认 0.05） */
} attitude_cfg_t;

/* ---- 注入通道（桥接层绑定到器件层；必填）---- */
typedef struct {
    attitude_ret_t (*read_accel_raw)(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az);
    attitude_ret_t (*read_gyro_raw)(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz);
} attitude_io_t;

/* ---- 生命周期 ---- */

/* 初始化（cfg/io 为 NULL 或校验不过 → 非 OK 且不置位）；成功后进校准状态机首拍 */
attitude_ret_t attitude_init(uint8_t id, const attitude_cfg_t *cfg, const attitude_io_t *io);

/* 器件量程注入（换桥在器件 init 后调用；影响 raw→物理量换算与零偏校准） */
attitude_ret_t attitude_set_scales(uint8_t id, float accel_lsb_per_g, float gyro_lsb_per_dps);

/* 节拍驱动：读 raw → 校准/漂移/融合全流程（桥接层按 IRATE 节拍调用） */
attitude_ret_t attitude_update(uint8_t id, uint32_t now_ms);

/* ---- 运行时调参（IMU 工具链命令 ITEL/IGAIN/IDRIFT/IRATE/ICAL 的执行端）---- */

attitude_ret_t attitude_set_mahony_gains(uint8_t id, float kp, float ki);

/* 漂移补偿开关：关闭 = 停止跟踪**并清零**当前补偿值（完全无补偿，E1 实验语义） */
attitude_ret_t attitude_set_drift_enable(uint8_t id, uint8_t en);
uint8_t        attitude_drift_enabled(uint8_t id);          /* 越界/未初始化 → 0 */

/* 重新触发零偏校准：复位状态机 + 清漂移；之后 cal_samples 拍需静止，完成后 yaw 归零 */
attitude_ret_t attitude_recalibrate(uint8_t id);

/* ---- 状态出（取值类：越界/未初始化返回安全默认值，不返回未定义值）---- */

uint8_t attitude_ready(uint8_t id);                          /* 越界/未初始化 → 0 */
float   attitude_get_roll(uint8_t id);                       /* 越界/未初始化 → 0.0（不可当真实姿态） */
float   attitude_get_pitch(uint8_t id);                      /* 同上 */
float   attitude_get_yaw(uint8_t id);                        /* 同上 */
uint8_t attitude_cal_progress(uint8_t id);                   /* 0~100；越界 → 0（=未完成，安全侧） */
uint8_t attitude_stable(uint8_t id);                         /* 最近一拍静止标志；越界 → 0 */
attitude_ret_t attitude_get_drift(uint8_t id, float *dx, float *dy, float *dz);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
