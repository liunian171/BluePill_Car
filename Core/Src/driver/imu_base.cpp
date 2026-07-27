/**
 * ============================================================================
 *  IMU 基类 — 通用默认实现
 * ============================================================================
 *
 *  init / read_accel_raw / read_gyro_raw / read_temp_raw / scale
 *  六个方法均由 ImuChipDesc 参数表驱动，子类不用重写。
 *  write_reg / read_regs 内部封装异步 I2C 为同步阻塞调用。
 *
 *  参考: IMU_Driver_Design.md
 * ============================================================================
 */

#include "imu_base.h"


/* ==========================================================================
 *  同步封装：异步 I2C → 忙等阻塞
 *
 *  关键：I2C FSM 回调固定传 I2C_SoftwareContext* 作为第一参数，
 *        不能用栈上的 struct 接收。用 static volatile 全局变量代替。
 * ========================================================================== */

static volatile uint8_t  g_i2c_sync_done;
static int8_t            g_i2c_sync_result;

/** @brief I2C 完成回调 — ISR 末尾调用，ctx 传的是 I2C_SoftwareContext*（忽略） */
static void i2c_sync_cb(void *ctx, int8_t result)
{
    (void)ctx;
    g_i2c_sync_result = result;
    g_i2c_sync_done   = 1;
}

#define I2C_SYNC_TIMEOUT_MS  100

static int8_t i2c_wait_done(void)
{
    uint32_t timeout = I2C_SYNC_TIMEOUT_MS * 100000;
    while (!g_i2c_sync_done && --timeout) {
        /* 忙等 */
    }
    if (!g_i2c_sync_done) {
        g_i2c_sync_result = -2;
        return -2;
    }
    int8_t r = g_i2c_sync_result;
    g_i2c_sync_done = 0;
    return r;
}


/* ==========================================================================
 *  构造
 * ========================================================================== */

ImuBase::ImuBase(I2C_Handle *bus, const ImuChipDesc *desc,
                 ImuAccelRange_t accel_range, ImuGyroRange_t gyro_range,
                 uint8_t dev_addr)
    : bus_(bus)
    , desc_(desc)
    , dev_addr_(dev_addr)
    , accel_range_(accel_range)
    , gyro_range_(gyro_range)
{
}


/* ==========================================================================
 *  同步 I2C 读写封装
 * ========================================================================== */

int8_t ImuBase::write_reg(uint8_t reg, uint8_t data)
{
    g_i2c_sync_done = 0;
    int8_t ret = i2c_write_reg_async(bus_, dev_addr_, reg,
                                     &data, 1, i2c_sync_cb);
    if (ret != 0) return -1;
    return i2c_wait_done();
}

int8_t ImuBase::read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    g_i2c_sync_done = 0;
    int8_t ret = i2c_read_reg_async(bus_, dev_addr_, reg,
                                    buf, len, i2c_sync_cb);
    if (ret != 0) return -1;
    return i2c_wait_done();
}


/* ==========================================================================
 *  IIMU 接口默认实现（由 desc 表驱动）
 * ========================================================================== */

int8_t ImuBase::init()
{
    /* ① 读 WHO_AM_I 验证通信 */
    uint8_t id;
    if (read_regs(desc_->who_am_i_reg, &id, 1) != 0)
        return -1;
    if (id != desc_->who_am_i_val)
        return -1;

    /* ② 按 desc 表循环写入初始化序列 */
    for (int i = 0; i < 16; i++) {
        if (desc_->init_seq[i].reg == 0 && desc_->init_seq[i].val == 0)
            break;
        if (write_reg(desc_->init_seq[i].reg, desc_->init_seq[i].val) != 0)
            return -1;
    }

    /* ③ 配置量程（GYRO_CONFIG / ACCEL_CONFIG 地址在 MPU 系列统一） */
    if (write_reg(0x1B, (uint8_t)(gyro_range_  << 3)) != 0)
        return -1;
    if (write_reg(0x1C, (uint8_t)(accel_range_ << 3)) != 0)
        return -1;

    return 0;
}

int8_t ImuBase::get_device_id()
{
    uint8_t id;
    if (read_regs(desc_->who_am_i_reg, &id, 1) != 0)
        return -1;
    return (id == desc_->who_am_i_val) ? 0 : -1;
}

int8_t ImuBase::read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    if (read_regs(desc_->accel_reg, buf, 6) != 0)
        return -1;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

int8_t ImuBase::read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    if (read_regs(desc_->gyro_reg, buf, 6) != 0)
        return -1;

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

int8_t ImuBase::read_temp_raw(int16_t *temp)
{
    uint8_t buf[2];
    if (read_regs(desc_->temp_reg, buf, 2) != 0)
        return -1;

    *temp = (int16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

float ImuBase::accel_scale() const
{
    return desc_->accel_scales[accel_range_];
}

float ImuBase::gyro_scale() const
{
    return desc_->gyro_scales[gyro_range_];
}
