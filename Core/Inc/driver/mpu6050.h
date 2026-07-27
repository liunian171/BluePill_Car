/**
 * ============================================================================
 *  MPU6050 驱动 — 芯片实现（继承 ImuBase）
 * ============================================================================
 *
 *  硬件: I2C SCL=PF0, SDA=PF1, AD0=GND → 7位地址 0x68
 *
 *  本类只做一件事：填 kMpu6050Desc 表 + 调用 ImuBase 构造。
 *  所有 init / read_accel_raw / read_gyro_raw / scale 逻辑在 ImuBase 中。
 *
 *  参考: IMU_Driver_Design.md
 * ============================================================================
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "imu_base.h"


#define MPU6050_ADDR_AD0_LOW   0x68


class MPU6050 : public ImuBase
{
public:
    /**
     * @param bus          I2C 句柄（将来→ IBusTransport*）
     * @param accel_range  加速度量程（默认 ±2g）
     * @param gyro_range   角速度量程（默认 ±250°/s）
     * @param dev_addr     7 位地址（默认 0x68，AD0 接高时为 0x69）
     */
    MPU6050(I2C_Handle *bus,
            ImuAccelRange_t accel_range = IMU_ACCEL_RANGE_2G,
            ImuGyroRange_t  gyro_range  = IMU_GYRO_RANGE_250,
            uint8_t         dev_addr    = MPU6050_ADDR_AD0_LOW);
};

#endif /* __MPU6050_H__ */
