/**
 * @file    oled_platform_ops.c
 * @brief   OLED 平台操作 — STM32 HAL 实现
 *
 *  本文件是 OLED 链中**唯一** include CubeMX 头（i2c.h → stm32f1xx_hal.h）的地方。
 *  器件层 `oled_driver.cpp` 因此零 HAL 依赖（换平台只换本文件）。
 *
 *  注：原实现把 `&hi2c2` 硬编码在 oled_driver.cpp 里，并把 i2c.h 传染给器件层。
 */

#include "oled_platform_ops.h"
#include "i2c.h"               /* hi2c2 / I2C_HandleTypeDef — 本文件唯一使用者 */

#define OLED_IO_TIMEOUT_MS  100

int8_t oled_platform_write_stm32(void *i2c_context, uint8_t addr7, uint8_t reg,
                                 const uint8_t *data, uint16_t length)
{
    if (i2c_context == 0 || data == 0 || length == 0) return -1;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)i2c_context,
                                             (uint16_t)(addr7 << 1),  /* HAL 要 8 位形式 */
                                             reg, I2C_MEMADD_SIZE_8BIT,
                                             (uint8_t *)data, length,
                                             OLED_IO_TIMEOUT_MS);
    return (st == HAL_OK) ? 0 : -1;
}
