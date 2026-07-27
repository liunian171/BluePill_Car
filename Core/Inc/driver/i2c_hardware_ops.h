/**
 * @file    i2c_hardware_ops.h
 * @brief   硬件 I2C 平台操作表 — STM32 HAL 实现声明
 *
 * 将 STM32 HAL I2C 的阻塞读写封装为 I2C_PlatformOps_t 接口，
 * 供 IMU 等驱动通过 I2C_Handle + ops 表模式使用。
 */

#ifndef __I2C_HARDWARE_OPS_H__
#define __I2C_HARDWARE_OPS_H__

#include "useri2c.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const I2C_PlatformOps_t i2c_hardware_platform_ops_stm32;

int8_t i2c_hardware_start_transfer(void *i2c_context,
                                   uint8_t device_address,
                                   uint8_t register_addr,
                                   uint8_t is_read,
                                   uint8_t *buffer, uint16_t length,
                                   I2C_TransferCallback callback);

uint8_t i2c_hardware_is_busy(void *i2c_context);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_HARDWARE_OPS_H__ */
