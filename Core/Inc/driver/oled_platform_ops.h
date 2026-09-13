#ifndef OLED_PLATFORM_OPS_H
#define OLED_PLATFORM_OPS_H

/**
 * ============================================================================
 *  OLED 平台操作 — 写事务注入（唯一碰 HAL / CubeMX 的地方）
 * ============================================================================
 *
 *  为什么是"写事务函数"而不是 I2C 句柄：
 *    OLED 对总线的需求只有一种——「把 N 字节写到器件某寄存器」。
 *    注入 I2C_Handle 会把异步 ops（start_transfer + 回调）拖进显示链，
 *    注入 HAL 句柄则把器件层重新绑回 HAL。注入**一个函数**即最小且够用。
 *
 *  分层：器件层 `oled_driver` ←构造注入← 本文件（平台绑定）←组装层配置← 组装层
 *  平台对照：与 `pwm_platform_ops.c` / `encoder_platform_ops.c` 同构
 * ============================================================================
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 向 OLED 的指定寄存器写一段数据（完成一次 I2C 事务）
 * @param i2c_context 平台资源指针（STM32 为 I2C_HandleTypeDef*）
 * @param addr7       从机 7 位地址（不含 R/W 位）
 * @param reg         控制字节：0x00=命令 / 0x40=数据
 * @param data        待写字节
 * @param length      字节数
 * @retval 0 成功；非 0 失败（显示链按"尽力而为"处理，忽略返回码）
 */
typedef int8_t (*oled_write_fn)(void *i2c_context, uint8_t addr7, uint8_t reg,
                                const uint8_t *data, uint16_t length);

/** STM32 HAL 实现（阻塞式 HAL_I2C_Mem_Write，超时 100ms） */
int8_t oled_platform_write_stm32(void *i2c_context, uint8_t addr7, uint8_t reg,
                                 const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* OLED_PLATFORM_OPS_H */
