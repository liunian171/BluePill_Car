/**
 * @file    imu_bridge.cpp
 * @brief   IMU C 桥接实现
 *
 * Mahony 滤波替代了原来的互补滤波，用四元数 + PI 反馈做传感器融合。
 *
 * 校准策略：
 *   1. 启动后前 50 次采样做陀螺仪零偏校准
 *   2. 使用加速度计初始化四元数
 *   3. 运行中持续检测静止状态，跟踪陀螺零偏漂移
 *   4. Mahony 滤波的 Kp 项实时修正姿态误差
 *
 * 家族契约（C1，2026-09-13）：
 *   · 静态池 + placement new（原为堆 new，违反禁堆铁律）
 *   · 校准/漂移状态**按 id 分账**（原为文件级全局 → cal_progress(id) 忽略 id 的 bug）
 *   · 动作类返回 bridge_ret_t；时间基准由调用方传入（去 HAL_GetTick → 全桥零 HAL）
 *   ⚠️ 本桥仍是"胖桥"（校准/漂移/滤波调度 ~90 行），下沉为姿态组件属 C3。
 */

#include "imu_bridge.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include <new>              /* placement new */
#include <stddef.h>
#include <math.h>           /* fabsf, sqrtf */

/* ---- 静态池（禁堆 new：heap 仅 512B，分配失败即硬错误）---- */
static uint8_t    g_imu_mem[MAX_IMUS][sizeof(MPU6050)];
static IIMU      *imu_devices[MAX_IMUS]  = {nullptr};
static uint8_t    imu_ready[MAX_IMUS]    = {0};
static uint32_t   imu_last_tick[MAX_IMUS] = {0};

/* ImuFilter 为纯数据对象（无可变大小），直接数组化 */
static ImuFilter  imu_filters[MAX_IMUS];

/* ---- 校准状态：per-id（多通道各记各的账）---- */
static int32_t    g_gb_sum[MAX_IMUS][3]   = {{0}};
static uint8_t    g_cal_cnt[MAX_IMUS]     = {0};
static float      g_gb_drift[MAX_IMUS][3] = {{0}};
static uint32_t   g_stable_since[MAX_IMUS] = {0};

/* ---- IMU 工具链状态 (2026-09-15): 漂移补偿开关 / 最近一拍静止标志 ---- */
static uint8_t    g_drift_en[MAX_IMUS]    = {1, 1, 1, 1};   /* 默认开 = 现有行为 */
static uint8_t    g_is_stable[MAX_IMUS]   = {0};

#define IMU_CAL_SAMPLES   50    /* 初始零偏校准采样次数 */
#define IMU_CAL_DONE      101   /* 完成标志（>50 即视为完成，沿用原语义） */

bridge_ret_t imu_bridge_init(uint8_t id, const imu_bridge_cfg_t *cfg)
{
    if (cfg == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (cfg->i2c_handle == nullptr) return BRIDGE_ERR_BAD_ARG;   /* 原实现未查 NULL */

    if (cfg->type != IMU_MPU6050) return BRIDGE_ERR_BAD_CFG;     /* 未实现类型：显式报错 */

    imu_devices[id] = new (g_imu_mem[id]) MPU6050((I2C_Handle *)cfg->i2c_handle);

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

    /* 校准状态复位（per-id） */
    for (int i = 0; i < 3; i++) { g_gb_sum[id][i] = 0; g_gb_drift[id][i] = 0.0f; }
    g_cal_cnt[id] = 0;
    g_stable_since[id] = 0;
    g_drift_en[id]     = 1;      /* 工具链开关恢复默认 (init = 上电态) */
    g_is_stable[id]    = 0;
    imu_last_tick[id]  = 0;

    /* 滤波器系数 — 默认 Kp=0.5, Ki=0（运行时漂移补偿代替积分项） */
    imu_filters[id].set_accel_scale(imu_devices[id]->accel_scale());
    imu_filters[id].set_gyro_scale(imu_devices[id]->gyro_scale());
    imu_filters[id].set_mahony_gains(0.5f, 0.0f);

    return BRIDGE_OK;
}

/* ---- 取值类：越界/未初始化返回安全默认值 ---- */

uint8_t imu_bridge_ready(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_ready[id] : 0;
}

float imu_bridge_accel_scale(uint8_t id)
{
    if (id >= MAX_IMUS || imu_devices[id] == nullptr) return 1.0f;
    return imu_devices[id]->accel_scale();
}

float imu_bridge_gyro_scale(uint8_t id)
{
    if (id >= MAX_IMUS || imu_devices[id] == nullptr) return 1.0f;
    return imu_devices[id]->gyro_scale();
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
    if (id >= MAX_IMUS) return 0;   /* 安全侧：0 = 未完成 */
    return (g_cal_cnt[id] > IMU_CAL_SAMPLES) ? 100 : g_cal_cnt[id];
}

/* ---- 动作类 ---- */

bridge_ret_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (ax == nullptr || ay == nullptr || az == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return (imu_devices[id]->read_accel_raw(ax, ay, az) == 0) ? BRIDGE_OK : BRIDGE_ERR_IO;
}

bridge_ret_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (gx == nullptr || gy == nullptr || gz == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return (imu_devices[id]->read_gyro_raw(gx, gy, gz) == 0) ? BRIDGE_OK : BRIDGE_ERR_IO;
}

/* ==========================================================================
 *  Mahony 滤波更新
 *
 *  流程：
 *    1. 初始零偏校准（前 50 次采样取均值）
 *    2. 加速度计初始化四元数
 *    3. 运行中零偏漂移补偿（静止时慢速跟踪）
 *    4. 调用 Mahony 滤波融合
 * ========================================================================== */

bridge_ret_t imu_bridge_update_filter(uint8_t id, uint32_t now_ms)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (!imu_bridge_ready(id)) return BRIDGE_ERR_NOT_INIT;

    int16_t ax, ay, az, gx, gy, gz;
    if (imu_bridge_read_accel_raw(id, &ax, &ay, &az) != BRIDGE_OK ||
        imu_bridge_read_gyro_raw(id, &gx, &gy, &gz) != BRIDGE_OK)
        return BRIDGE_ERR_IO;

    float af[3], gf[3];
    imu_filters[id].raw_to_accel(ax, ay, az, &af[0], &af[1], &af[2]);
    imu_filters[id].raw_to_gyro(gx, gy, gz, &gf[0], &gf[1], &gf[2]);

    /* ═══════════════════════════════════════════════════════════════
     *  阶段①：初始静态零偏校准（前 IMU_CAL_SAMPLES 次）
     * ═══════════════════════════════════════════════════════════════ */
    if (g_cal_cnt[id] < IMU_CAL_SAMPLES) {
        g_gb_sum[id][0] += gx; g_gb_sum[id][1] += gy; g_gb_sum[id][2] += gz;
        g_cal_cnt[id]++;
        imu_last_tick[id] = now_ms;
        return BRIDGE_OK;
    } else if (g_cal_cnt[id] == IMU_CAL_SAMPLES) {
        int16_t avg[3];
        avg[0] = (int16_t)(g_gb_sum[id][0] / IMU_CAL_SAMPLES);
        avg[1] = (int16_t)(g_gb_sum[id][1] / IMU_CAL_SAMPLES);
        avg[2] = (int16_t)(g_gb_sum[id][2] / IMU_CAL_SAMPLES);
        imu_filters[id].calibrate_gyro_bias(&avg[0], &avg[1], &avg[2], 1);
        g_cal_cnt[id] = IMU_CAL_DONE;

        /* 用加速度计初始化四元数 */
        imu_filters[id].init_from_accel(af[0], af[1], af[2]);
        imu_last_tick[id] = now_ms;
        return BRIDGE_OK;
    }

    uint32_t now = now_ms;
    float dt    = (float)(now - imu_last_tick[id]) * 0.001f;
    imu_last_tick[id] = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 1.0f)   dt = 0.01f;   /* 长时间中断后用小 dt 渐进恢复 */

    /* ═══════════════════════════════════════════════════════════════
     *  阶段②：运行中动态零偏漂移补偿（per-id）
     *
     *  注意：Mahony 的 Kp 项修正的是姿态误差（加速度 vs 四元数估算重力），
     *        不能替代陀螺仪零偏的直接测量。这里用静止时的陀螺输出来
     *        慢速跟踪实际的零偏漂移。
     * ═══════════════════════════════════════════════════════════════ */

    /* 用加速度模值判断静止：|accel - 1g| < 0.05g → 设备没动 */
    float amag = sqrtf(af[0]*af[0] + af[1]*af[1] + af[2]*af[2]);
    int is_stable = (fabsf(amag - 1.0f) < 0.05f);
    g_is_stable[id] = (uint8_t)(is_stable ? 1 : 0);   /* ITEL 诊断列 */

    /* ⚠️ 已知缺陷 (IMU-1 yaw 失真首嫌疑, 2026-09-15 仿真证实机理, 见
     * doc/IMU调试工具链规划.md §3): 绕竖直轴慢转时 |a|≈1g → 误判"静止",
     * 漂移跟踪把旋转角速度当零偏吸收 → yaw 积分冻结 + 停转后回落。
     * 临时对策 = IDRIFT 命令关补偿 (E1 实验); 永久修法 = 陀螺模值门限 (E1 证实后落地) */
    if (!g_drift_en[id]) {
        for (int i = 0; i < 3; i++) g_gb_drift[id][i] = 0.0f;  /* 关闭即清零: 完全无补偿 */
    }

    if (is_stable && g_drift_en[id]) {
        if (g_stable_since[id] == 0) g_stable_since[id] = now;
        uint32_t stable_ms = now - g_stable_since[id];

        if (stable_ms > 1000) {
            /* 静止后快速跟踪零偏，1~5秒用 0.02，之后用 0.05 */
            float rate = (stable_ms > 5000) ? 0.05f : 0.02f;
            g_gb_drift[id][0] += (gf[0] - g_gb_drift[id][0]) * rate;
            g_gb_drift[id][1] += (gf[1] - g_gb_drift[id][1]) * rate;
            g_gb_drift[id][2] += (gf[2] - g_gb_drift[id][2]) * rate;
        }
    } else {
        g_stable_since[id] = 0;
    }

    /* 减去漂移补偿量 */
    gf[0] -= g_gb_drift[id][0];
    gf[1] -= g_gb_drift[id][1];
    gf[2] -= g_gb_drift[id][2];

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
    return BRIDGE_OK;
}

/* ==========================================================================
 *  运行时调参接口（可通过 UART 命令调用）
 * ========================================================================== */

bridge_ret_t imu_bridge_set_mahony_gains(uint8_t id, float kp, float ki)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    imu_filters[id].set_mahony_gains(kp, ki);
    return BRIDGE_OK;
}

/* ---- IMU 工具链 (2026-09-15) ---- */

bridge_ret_t imu_bridge_set_drift_enable(uint8_t id, uint8_t en)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    g_drift_en[id] = en ? 1u : 0u;
    if (!g_drift_en[id]) {
        /* 关闭瞬间清零补偿量: 下一拍起完全无补偿 (E1 实验语义, 见 update_filter 注) */
        for (int i = 0; i < 3; i++) g_gb_drift[id][i] = 0.0f;
    }
    return BRIDGE_OK;
}

uint8_t imu_bridge_drift_enabled(uint8_t id)
{
    return (id < MAX_IMUS) ? g_drift_en[id] : 0;
}

bridge_ret_t imu_bridge_recalibrate(uint8_t id)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    /* 复位校准状态机 → update_filter 重新走"50 拍采均值 → init_from_accel"路径 */
    for (int i = 0; i < 3; i++) { g_gb_sum[id][i] = 0; g_gb_drift[id][i] = 0.0f; }
    g_cal_cnt[id] = 0;
    g_stable_since[id] = 0;
    return BRIDGE_OK;
}

uint8_t imu_bridge_stable(uint8_t id)
{
    return (id < MAX_IMUS) ? g_is_stable[id] : 0;
}

bridge_ret_t imu_bridge_get_drift(uint8_t id, float *dx, float *dy, float *dz)
{
    if (id >= MAX_IMUS || dx == nullptr || dy == nullptr || dz == nullptr)
        return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    *dx = g_gb_drift[id][0];
    *dy = g_gb_drift[id][1];
    *dz = g_gb_drift[id][2];
    return BRIDGE_OK;
}
