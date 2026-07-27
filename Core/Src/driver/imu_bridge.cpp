/**
 * @file    imu_bridge.cpp
 * @brief   IMU C 桥接实现
 *
 * Mahony 滤波替代了原来的互补滤波，用四元数 + PI 反馈做传感器融合。
 *
 * 校准策略：
 *   1. 启动后前 100 次采样做陀螺仪零偏校准（~50秒 @ 2Hz）
 *   2. 使用加速度计初始化四元数
 *   3. 运行中持续检测静止状态，跟踪陀螺零偏漂移
 *   4. Mahony 滤波的 Kp 项实时修正姿态误差
 */

#include "imu_bridge.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <math.h>           /* fabsf, sqrtf */

static IIMU       *imu_devices[MAX_IMUS]  = {nullptr};
static ImuType_t   imu_types[MAX_IMUS]    = {IMU_MPU6050};
static uint8_t     imu_ready[MAX_IMUS]    = {0};
static ImuFilter   imu_filters[MAX_IMUS];
static uint32_t    imu_last_tick[MAX_IMUS] = {0};

/* 初始校准状态（文件级，供 imu_bridge_cal_progress 读取） */
static int32_t     g_gb_sum[3] = {0};
static uint8_t     g_cal_cnt   = 0;


void imu_bridge_init(uint8_t id, ImuType_t type, void *i2c_handle)
{
    if (id >= MAX_IMUS) return;

    switch (type) {
    case IMU_MPU6050:
        imu_devices[id] = new MPU6050((I2C_Handle *)i2c_handle);
        imu_types[id]   = IMU_MPU6050;
        break;
    default:
        return;
    }

    if (imu_devices[id]->init() == 0) {
        imu_ready[id] = 1;
    } else {
        /* 兜底：克隆芯片 WHO_AM_I 不匹配，手动唤醒 + 配置 */
        MPU6050 *mpu = static_cast<MPU6050 *>(imu_devices[id]);
        mpu->write_reg(0x6B, 0x00);
        for (volatile int i = 0; i < 500000; i++) { }
        mpu->write_reg(0x19, 9);
        mpu->write_reg(0x1A, 0x06);
        mpu->write_reg(0x1B, 0x00);
        mpu->write_reg(0x1C, 0x00);
        imu_ready[id] = 1;
    }

    /* 滤波器系数 — 默认 Kp=0.5, Ki=0（运行时漂移补偿代替积分项） */
    imu_filters[id].set_accel_scale(imu_devices[id]->accel_scale());
    imu_filters[id].set_gyro_scale(imu_devices[id]->gyro_scale());
    imu_filters[id].set_mahony_gains(0.5f, 0.0f);
}

uint8_t imu_bridge_ready(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_ready[id] : 0;
}

int8_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return -1;
    return imu_devices[id]->read_accel_raw(ax, ay, az);
}

int8_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return -1;
    return imu_devices[id]->read_gyro_raw(gx, gy, gz);
}

float imu_bridge_accel_scale(uint8_t id)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return 1.0f;
    return imu_devices[id]->accel_scale();
}

float imu_bridge_gyro_scale(uint8_t id)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return 1.0f;
    return imu_devices[id]->gyro_scale();
}

/* ==========================================================================
 *  Mahony 滤波更新
 *
 *  流程：
 *    1. 初始零偏校准（100 次采样取均值）
 *    2. 加速度计初始化四元数
 *    3. 运行中零偏漂移补偿（静止时慢速跟踪）
 *    4. 调用 Mahony 滤波融合
 * ========================================================================== */

void imu_bridge_update_filter(uint8_t id)
{
    if (!imu_bridge_ready(id)) return;

    int16_t ax, ay, az, gx, gy, gz;
    if (imu_bridge_read_accel_raw(id, &ax, &ay, &az) != 0 ||
        imu_bridge_read_gyro_raw(id, &gx, &gy, &gz) != 0)
        return;

    float af[3], gf[3];
    imu_filters[id].raw_to_accel(ax, ay, az, &af[0], &af[1], &af[2]);
    imu_filters[id].raw_to_gyro(gx, gy, gz, &gf[0], &gf[1], &gf[2]);

    /* ═══════════════════════════════════════════════════════════════
     *  阶段①：初始静态零偏校准（前 50 次 ≈ 5 秒）
     * ═══════════════════════════════════════════════════════════════ */
    if (g_cal_cnt < 50) {
        g_gb_sum[0] += gx; g_gb_sum[1] += gy; g_gb_sum[2] += gz;
        g_cal_cnt++;
        imu_last_tick[id] = HAL_GetTick();
        return;
    } else if (g_cal_cnt == 50) {
        int16_t avg[3];
        avg[0] = (int16_t)(g_gb_sum[0] / 50);
        avg[1] = (int16_t)(g_gb_sum[1] / 50);
        avg[2] = (int16_t)(g_gb_sum[2] / 50);
        imu_filters[id].calibrate_gyro_bias(&avg[0], &avg[1], &avg[2], 1);
        g_cal_cnt = 101;

        /* 用加速度计初始化四元数 */
        imu_filters[id].init_from_accel(af[0], af[1], af[2]);
        imu_last_tick[id] = HAL_GetTick();
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt    = (float)(now - imu_last_tick[id]) * 0.001f;
    imu_last_tick[id] = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 1.0f)   dt = 0.01f;   /* 长时间中断后用小 dt 渐进恢复 */

    /* ═══════════════════════════════════════════════════════════════
     *  阶段②：运行中动态零偏漂移补偿
     *
     *  注意：Mahony 的 Kp 项修正的是姿态误差（加速度 vs 四元数估算重力），
     *        不能替代陀螺仪零偏的直接测量。这里用静止时的陀螺输出来
     *        慢速跟踪实际的零偏漂移。
     * ═══════════════════════════════════════════════════════════════ */
    static float   gb_drift[3] = {0, 0, 0};
    static uint32_t stable_since = 0;

    /* 用加速度模值判断静止：|accel - 1g| < 0.05g → 设备没动 */
    float amag = sqrtf(af[0]*af[0] + af[1]*af[1] + af[2]*af[2]);
    int is_stable = (fabsf(amag - 1.0f) < 0.05f);

    if (is_stable) {
        if (stable_since == 0) stable_since = now;
        uint32_t stable_ms = now - stable_since;

        if (stable_ms > 1000) {
            /* 静止后快速跟踪零偏，1~5秒用 0.02，之后用 0.05 */
            float rate = (stable_ms > 5000) ? 0.05f : 0.02f;
            gb_drift[0] += (gf[0] - gb_drift[0]) * rate;
            gb_drift[1] += (gf[1] - gb_drift[1]) * rate;
            gb_drift[2] += (gf[2] - gb_drift[2]) * rate;
        }
    } else {
        stable_since = 0;
    }

    /* 减去漂移补偿量 */
    gf[0] -= gb_drift[0];
    gf[1] -= gb_drift[1];
    gf[2] -= gb_drift[2];

    /* ═══════════════════════════════════════════════════════════════
     *  阶段③：Mahony 滤波（替代原来的互补滤波）
     *
     *  Mahony 自动处理了加速度计和陀螺仪的融合：
     *   - 加速度计提供重力参考方向
     *   - 陀螺仪提供短时间内的稳定角速度
     *   - 叉积误差 + Kp 保证姿态快速收敛
     *   - 不需要外部的动态 alpha 调节
     * ═══════════════════════════════════════════════════════════════ */
    imu_filters[id].mahony_filter(gf[0], gf[1], gf[2],
                                   af[0], af[1], af[2], dt);
}

float imu_bridge_get_roll(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_roll() : 0.0f;
}
float imu_bridge_get_pitch(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_pitch() : 0.0f;
}
float imu_bridge_get_yaw(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_yaw() : 0.0f;
}

uint8_t imu_bridge_cal_progress(uint8_t id)
{
    (void)id;
    return (g_cal_cnt > 50) ? 100 : g_cal_cnt;
}

/* ==========================================================================
 *  运行时调参接口（可通过 UART 命令调用）
 * ========================================================================== */

void imu_bridge_set_mahony_gains(uint8_t id, float kp, float ki)
{
    if (id < MAX_IMUS)
        imu_filters[id].set_mahony_gains(kp, ki);
}
