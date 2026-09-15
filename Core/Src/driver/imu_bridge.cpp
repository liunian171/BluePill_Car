/**
 * ============================================================================
 *  桥接层 — imu_bridge（IMU C 门面）— C3 薄桥版
 * ============================================================================
 *
 *  层位：组装层 / 感知组件 → **本文件** → 器件层(MPU6050 → IIMU) / attitude 组件
 *
 *  C3（2026-09-15）：校准状态机 / 静止检测 / 门限漂移跟踪 / 滤波调度 ~90 行
 *  业务逻辑下沉至 driver/attitude.c 感知组件（C2 同款搬迁），本桥按家族契约
 *  回归"查 id → 校验 → 转发"三件事——对 main.c 的 C 接口 **零变化**
 *  （imu_bridge_* 全保留，搬迁等价性用 ITEL 曲线背书）。
 *
 *  桥内现仅剩：器件静态池 + placement new（C++ 侧）+ attitude 通道绑定 + 转发。
 *
 *  ▸ 家族契约（doc/桥接层家族契约.md）◂
 *    · init 配置注入 / 返回码 / 静态池禁堆 / per-id 状态 / 零 HAL（器件层除外）
 *    · attitude_ret_t 与 bridge_ret_t 值序 1:1 对齐（直接 cast 映射）
 * ============================================================================
 */

#include "imu_bridge.h"
#include "mpu6050.h"
#include "attitude.h"
#include <new>              /* placement new */
#include <stddef.h>

/* ---- 器件静态池（禁堆 new：heap 仅 512B，分配失败即硬错误）---- */
static uint8_t    g_imu_mem[MAX_IMUS][sizeof(MPU6050)];
static IIMU      *imu_devices[MAX_IMUS]  = {nullptr};
static uint8_t    imu_ready[MAX_IMUS]    = {0};

/* ---- attitude 组件的 IO 绑定：转发到本 id 的器件实例 ---- */

static attitude_ret_t io_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az)
{
    if (id >= MAX_IMUS || imu_devices[id] == nullptr) return ATTITUDE_ERR_NOT_INIT;
    return (imu_devices[id]->read_accel_raw(ax, ay, az) == 0) ? ATTITUDE_OK : ATTITUDE_ERR_IO;
}

static attitude_ret_t io_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (id >= MAX_IMUS || imu_devices[id] == nullptr) return ATTITUDE_ERR_NOT_INIT;
    return (imu_devices[id]->read_gyro_raw(gx, gy, gz) == 0) ? ATTITUDE_OK : ATTITUDE_ERR_IO;
}

static const attitude_io_t g_att_io = {
    .read_accel_raw = io_read_accel_raw,
    .read_gyro_raw  = io_read_gyro_raw,
};

/* ---- attitude 配置：成员置零 → 组件取默认（= 原固件字面量：50 拍 / 0.05g /
 *      门限 1.5°/s / 速率 0.02→0.05）。参数资产见 doc/IMU调试SOP §7 ---- */
static const attitude_cfg_t g_att_cfg = { 0 };

/* attitude_ret_t ↔ bridge_ret_t 值序 1:1（两枚举同序同值，直接映射） */
static bridge_ret_t ret(attitude_ret_t r) { return (bridge_ret_t)r; }

bridge_ret_t imu_bridge_init(uint8_t id, const imu_bridge_cfg_t *cfg)
{
    if (cfg == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (cfg->i2c_handle == nullptr) return BRIDGE_ERR_BAD_ARG;

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

    /* 姿态组件：注入 IO + 器件量程（校准状态机从首拍开始） */
    attitude_ret_t r = attitude_init(id, &g_att_cfg, &g_att_io);
    if (r != ATTITUDE_OK) return ret(r);
    (void)attitude_set_scales(id, imu_devices[id]->accel_scale(), imu_devices[id]->gyro_scale());

    return BRIDGE_OK;
}

/* ---- 取值类：越界/未初始化返回安全默认值（attitude 组件内实现，此处纯转发）---- */

uint8_t imu_bridge_ready(uint8_t id)
{
    return (id < MAX_IMUS) ? (uint8_t)(imu_ready[id] && attitude_ready(id)) : 0u;
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

float imu_bridge_get_roll(uint8_t id)  { return attitude_get_roll(id); }
float imu_bridge_get_pitch(uint8_t id) { return attitude_get_pitch(id); }
float imu_bridge_get_yaw(uint8_t id)   { return attitude_get_yaw(id); }

uint8_t imu_bridge_cal_progress(uint8_t id) { return attitude_cal_progress(id); }
uint8_t imu_bridge_stable(uint8_t id)       { return attitude_stable(id); }

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

/* 读 → 校准/漂移(门限)/融合 全流程在 attitude 组件内；时间基准由调用方传入 */
bridge_ret_t imu_bridge_update_filter(uint8_t id, uint32_t now_ms)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (!imu_bridge_ready(id)) return BRIDGE_ERR_NOT_INIT;
    return ret(attitude_update(id, now_ms));
}

bridge_ret_t imu_bridge_set_mahony_gains(uint8_t id, float kp, float ki)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return ret(attitude_set_mahony_gains(id, kp, ki));
}

bridge_ret_t imu_bridge_set_drift_enable(uint8_t id, uint8_t en)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return ret(attitude_set_drift_enable(id, en));
}

uint8_t imu_bridge_drift_enabled(uint8_t id)
{
    return (id < MAX_IMUS) ? attitude_drift_enabled(id) : 0u;
}

bridge_ret_t imu_bridge_recalibrate(uint8_t id)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return ret(attitude_recalibrate(id));
}

bridge_ret_t imu_bridge_get_drift(uint8_t id, float *dx, float *dy, float *dz)
{
    if (id >= MAX_IMUS) return BRIDGE_ERR_BAD_ARG;
    if (imu_devices[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return ret(attitude_get_drift(id, dx, dy, dz));
}
