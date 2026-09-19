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

/** OLED 写事务超时 (ms)
 *
 *  ▸ 取值依据（2026-09-19 节拍拉长专案结案后下调，原值 100）◂
 *    · 正常耗时：I2C2 = 100kHz，整页 128B 写 ≈ (128+2)×9bit / 100kHz ≈ **11.7ms**
 *      → 超时必须大于它，留余量取 20ms
 *    · 原值 100ms 的实测代价：OLED 排线接触不稳时每笔事务吃满 100ms，而显示块
 *      基准在块首赋值 → **自锁**（每轮主循环都到期）→ pass 锁死 100ms
 *      → 50ms 控制环减半（ODOM 20→10Hz、ATT 10→5Hz、ITEL 200→204ms）
 *    · 教训：**与电机控制环同住一条主循环的外设，失败代价必须按控制节拍预算**
 *      （单笔超时 × 每轮重试次数 = 最坏阻塞量），"够用就行"会变成整车节拍故障
 *    · 二次防线见 oled_driver.cpp 的失败熔断（连续失败即停刷，不每轮重试）
 */
#define OLED_IO_TIMEOUT_MS  20

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
