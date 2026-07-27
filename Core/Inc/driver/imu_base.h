/**
 * ============================================================================
 *  IMU 基类 — 参数表驱动的默认实现
 * ============================================================================
 *
 *  定位：ImuBase 为所有 MPU 系列 IMU 提供通用默认实现。
 *        子类只需填一张 ImuChipDesc 表即可获得完整的 register read/init/scale 功能。
 *        不兼容的参数表场景（如 BMI160）可以全部覆写。
 *
 *  继承链：
 *    IIMU（纯虚接口）
 *      ↑
 *    ImuBase（参数表默认实现）← 本文件
 *      ↑
 *    MPU6050 / MPU9250
 *
 *  当前直接持有 I2C_Handle*，将来加 SPI 时改为 IBusTransport*。
 *  参考: IMU_Driver_Design.md
 * ============================================================================
 */

#ifndef __IMU_BASE_H__
#define __IMU_BASE_H__

#include "imu.h"
#include "useri2c.h"


/* ==========================================================================
 *  ImuChipDesc — 芯片参数表
 *
 *  每颗 MPU 系列芯片填一张表，ImuBase 据此执行所有通用操作。
 *  当前只用了 init_seq 的前几个槽位，最大 16 个以备长序列。
 *  init_seq 以 {0, 0} 结尾。
 * ========================================================================== */

typedef struct ImuChipDesc {
    const char *name;              ///< 芯片名（调试用）

    uint8_t    who_am_i_reg;       ///< WHO_AM_I 寄存器地址
    uint8_t    who_am_i_val;       ///< 期望返回值（0x68/0x71/...）

    uint8_t    accel_reg;          ///< 加速度起始寄存器（如 0x3B）
    uint8_t    temp_reg;           ///< 温度起始寄存器（如 0x41）
    uint8_t    gyro_reg;           ///< 角速度起始寄存器（如 0x43）

    float      accel_scales[4];    ///< 量程 0~3 对应的 LSB/g
    float      gyro_scales[4];     ///< 量程 0~3 对应的 LSB/°/s

    struct {
        uint8_t reg;               ///< 寄存器地址
        uint8_t val;               ///< 写入值
    } init_seq[16];                ///< 初始化序列，以 {0,0} 结尾
} ImuChipDesc;


/* ==========================================================================
 *  ImuBase — MPU 系列默认实现
 * ========================================================================== */

class ImuBase : public IIMU
{
public:
    /**
     * @brief 构造
     * @param bus          I2C 句柄（将来改为 IBusTransport*）
     * @param desc         芯片参数表
     * @param accel_range  加速度量程
     * @param gyro_range   角速度量程
     * @param dev_addr     7 位 I2C 地址（默认 0x68）
     */
    ImuBase(I2C_Handle *bus, const ImuChipDesc *desc,
            ImuAccelRange_t accel_range = IMU_ACCEL_RANGE_2G,
            ImuGyroRange_t  gyro_range  = IMU_GYRO_RANGE_250,
            uint8_t         dev_addr    = 0x68);

    /* ---- IIMU 接口（virtual，子类可按需覆写） ---- */
    virtual int8_t  init() override;
    virtual int8_t  get_device_id() override;
    virtual int8_t  read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) override;
    virtual int8_t  read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) override;
    virtual int8_t  read_temp_raw(int16_t *temp) override;
    virtual float   accel_scale() const override;
    virtual float   gyro_scale() const override;

    virtual ~ImuBase() = default;

    /** @brief 同步写单个寄存器（调试用） */
    int8_t write_reg(uint8_t reg, uint8_t data);
    /** @brief 同步读连续寄存器（调试用） */
    int8_t read_regs(uint8_t reg, uint8_t *buf, uint16_t len);

protected:
    I2C_Handle         *bus_;             ///< 总线（将来→ IBusTransport*）
    const ImuChipDesc  *desc_;            ///< 芯片参数表
    uint8_t             dev_addr_;        ///< 7 位 I2C 地址
    ImuAccelRange_t     accel_range_;     ///< 加速度量程
    ImuGyroRange_t      gyro_range_;      ///< 角速度量程
};

#endif /* __IMU_BASE_H__ */
