/**
 * ============================================================================
 *  MPU6050 驱动 — 芯片实现
 * ============================================================================
 *
 *  本文件只包含参数表定义和构造函数。
 *  所有 init / read / scale 逻辑在 ImuBase（imu_base.cpp）中。
 *
 *  参考: IMU_Driver_Design.md, MPU6050 Datasheet
 * ============================================================================
 */

#include "mpu6050.h"


/* ==========================================================================
 *  MPU6050 参数表
 *
 *  这张表是 ImuBase 执行所有通用操作的数据源：
 *    - init()    循环 init_seq 写寄存器
 *    - read_accel_raw() 读 accel_reg 起的 6 字节
 *    - scale()   按量程查 accel_scales / gyro_scales
 * ========================================================================== */

static const ImuChipDesc kMpu6050Desc = {
    .name           = "MPU6050",

    .who_am_i_reg   = 0x75,
    .who_am_i_val   = 0x68,

    .accel_reg      = 0x3B,
    .temp_reg       = 0x41,
    .gyro_reg       = 0x43,

    .accel_scales   = {16384.0f, 8192.0f, 4096.0f, 2048.0f},
    .gyro_scales    = {131.0f, 65.5f, 32.8f, 16.4f},

    /* 初始化序列 (reg, val)，以 {0,0} 结尾
     *
     *   0x6B, 0x01  → PWR_MGMT_1: 唤醒 + 温度传感器 + PLL 时钟
     *   0x19, 9     → SMPLRT_DIV: 1kHz / (1+9) = 100Hz
     *   0x1A, 0x05  → CONFIG: DLPF=5, BW=10Hz
     *   GYRO_CONFIG 和 ACCEL_CONFIG 由 ImuBase::init() 根据量程参数动态写入
     */
    .init_seq       = {
        {0x6B, 0x01},
        {0x19, 9},
        {0x1A, 0x06},
        {0, 0},
    },
};


/* ==========================================================================
 *  构造
 * ========================================================================== */

MPU6050::MPU6050(I2C_Handle *bus,
                 ImuAccelRange_t accel_range,
                 ImuGyroRange_t  gyro_range,
                 uint8_t         dev_addr)
    : ImuBase(bus, &kMpu6050Desc, accel_range, gyro_range, dev_addr)
{
}
