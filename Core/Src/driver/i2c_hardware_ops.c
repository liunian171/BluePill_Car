/**
 * @file    i2c_hardware_ops.c
 * @brief   硬件 I2C 平台操作表 — STM32 HAL 实现
 *
 * 使用 HAL_I2C_Mem_Write/Read 阻塞模式完成传输，
 * 通过回调通知上层（ImuBase 的 i2c_wait_done 机制）。
 *
 * 地址说明：
 *   useri2c.h 协议层使用 7 位地址（不含 R/W 位），
 *   HAL 需要 8 位地址（左移 1 位），所以在函数内部左移。
 */

#include "i2c_hardware_ops.h"
#include <stddef.h>

#define I2C_TIMEOUT_MS  10

static uint8_t g_hardware_busy = 0;

int8_t i2c_hardware_start_transfer(void *i2c_context,
                                   uint8_t device_address,
                                   uint8_t register_addr,
                                   uint8_t is_read,
                                   uint8_t *buffer, uint16_t length,
                                   I2C_TransferCallback callback)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)i2c_context;
    if (!hi2c || !buffer) return -1;
    if (g_hardware_busy) return -1;

    g_hardware_busy = 1;
    HAL_StatusTypeDef status;

    if (is_read) {
        status = HAL_I2C_Mem_Read(hi2c,
                                  (uint16_t)(device_address << 1),
                                  register_addr,
                                  I2C_MEMADD_SIZE_8BIT,
                                  buffer, length,
                                  I2C_TIMEOUT_MS);
    } else {
        status = HAL_I2C_Mem_Write(hi2c,
                                   (uint16_t)(device_address << 1),
                                   register_addr,
                                   I2C_MEMADD_SIZE_8BIT,
                                   buffer, length,
                                   I2C_TIMEOUT_MS);
    }

    g_hardware_busy = 0;

    if (callback) {
        callback(i2c_context, (status == HAL_OK) ? 0 : -1);
    }

    return (status == HAL_OK) ? 0 : -1;
}

uint8_t i2c_hardware_is_busy(void *i2c_context)
{
    (void)i2c_context;
    return g_hardware_busy;
}

const I2C_PlatformOps_t i2c_hardware_platform_ops_stm32 = {
    .start_transfer = i2c_hardware_start_transfer,
    .is_busy        = i2c_hardware_is_busy,
};
