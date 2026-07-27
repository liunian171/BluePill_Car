/**
 * ============================================================================
 *  I2C 驱动 — 策略层（跨平台通用）
 * ============================================================================
 *
 *  本文件实现 useri2c.h 中声明的策略层 API。
 *  对标 pwm.c / uart.c 的模式：策略函数通过 handle->ops 调用平台层，
 *  自身只做参数转发和忙检查，不碰任何硬件。
 */

#include "useri2c.h"
#include <stdint.h>


/* ==========================================================================
 *  i2c_is_busy — 查询句柄是否正在传输
 * ========================================================================== */

uint8_t i2c_is_busy(I2C_Handle *handle)
{
    return handle->ops->is_busy(handle->i2c_context);
}


/* ==========================================================================
 *  i2c_write_reg_async — 异步写 I2C 设备寄存器
 *  委托给 ops->start_transfer，is_read=0。
 * ========================================================================== */

int8_t i2c_write_reg_async(I2C_Handle *handle,
    uint8_t device_address, uint8_t register_addr,
    uint8_t *data, uint16_t length,
    I2C_TransferCallback callback)
{
    if (!handle || !handle->ops || !handle->ops->start_transfer)
        return -1;

    if (handle->ops->is_busy(handle->i2c_context))
        return -1;

    return handle->ops->start_transfer(handle->i2c_context,
                                       device_address, register_addr,
                                       0, data, length, callback);
}


/* ==========================================================================
 *  i2c_read_reg_async — 异步读 I2C 设备寄存器
 *  委托给 ops->start_transfer，is_read=1。
 *  读事务由平台层宏状态机自动处理（先写寄存器地址 + RESTART + 读取）。
 * ========================================================================== */

int8_t i2c_read_reg_async(I2C_Handle *handle,
    uint8_t device_address, uint8_t register_addr,
    uint8_t *buffer, uint16_t length,
    I2C_TransferCallback callback)
{
    if (!handle || !handle->ops || !handle->ops->start_transfer)
        return -1;

    if (handle->ops->is_busy(handle->i2c_context))
        return -1;

    return handle->ops->start_transfer(handle->i2c_context,
                                       device_address, register_addr,
                                       1, buffer, length, callback);
}
