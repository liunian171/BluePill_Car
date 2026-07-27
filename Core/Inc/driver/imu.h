/**
 * ============================================================================
 *  IMU 芯片能力接口 — IIMU 纯虚类
 * ============================================================================
 *
 *  定位：所有 IMU 芯片的统一抽象。应用层通过此接口读写传感器原始值，
 *        不关心底层是 MPU6050 / MPU9250 / BMI160 还是 I2C / SPI。
 *
 *  架构：
 *    main.c → imu_bridge → IIMU → MPU6050 → I2C 驱动
 *                                         → SPI 驱动（将来）
 *
 *  换芯片时只需新增派生类 + Bridge 加一个 case，IIMU 和 ImuFilter 不动。
 *
 *  参考: IMU_Driver_Design.md
 * ============================================================================
 */

#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 量程枚举（C 兼容，供 Bridge 传参） ---- */
typedef enum {
    IMU_ACCEL_RANGE_2G  = 0,
    IMU_ACCEL_RANGE_4G  = 1,
    IMU_ACCEL_RANGE_8G  = 2,
    IMU_ACCEL_RANGE_16G = 3,
} ImuAccelRange_t;

typedef enum {
    IMU_GYRO_RANGE_250  = 0,
    IMU_GYRO_RANGE_500  = 1,
    IMU_GYRO_RANGE_1000 = 2,
    IMU_GYRO_RANGE_2000 = 3,
} ImuGyroRange_t;

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

class IIMU {
public:
    /**
     * @brief 初始化芯片（唤醒 + 配置量程/DLPF/采样率）
     * @retval 0  成功
     * @retval -1 I2C 超时或 WHO_AM_I 不匹配
     */
    virtual int8_t init() = 0;

    /** @brief 读取 WHO_AM_I 寄存器，用于验证通信 */
    virtual int8_t get_device_id() = 0;

    /**
     * @brief 读取加速度原始值（3 轴 x 2 字节 = 6 字节，从 0x3B 开始）
     * @param[out] ax / ay / az  原始 ADC 值（int16_t）
     * @retval 0 成功  -1 失败
     */
    virtual int8_t read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) = 0;

    /**
     * @brief 读取角速度原始值（3 轴 x 2 字节 = 6 字节，从 0x43 开始）
     * @param[out] gx / gy / gz  原始 ADC 值（int16_t）
     * @retval 0 成功  -1 失败
     */
    virtual int8_t read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) = 0;

    /**
     * @brief 读取温度原始值（2 字节，从 0x41 开始）
     * @param[out] temp  原始 ADC 值
     * @retval 0 成功  -1 失败
     */
    virtual int8_t read_temp_raw(int16_t *temp) = 0;

    /** @brief 返回当前加速度量程的换算系数（LSB/g） */
    virtual float accel_scale() const = 0;

    /** @brief 返回当前角速度量程的换算系数（LSB/(°/s)） */
    virtual float gyro_scale() const = 0;

    virtual ~IIMU() = default;
};

#endif /* __cplusplus */

#endif /* __IMU_H__ */
