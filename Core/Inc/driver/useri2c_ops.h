/**
 * ============================================================================
 *  软件 I2C 平台操作表 — STM32 平台实现声明
 * ============================================================================
 *
 *  对标 pwm_platform_ops.h / uart_platform_ops.h 的模式：
 *    - 声明 STM32 平台下所有 ops 函数的原型
 *    - extern 操作表实例
 *
 *  换平台时创建同名文件（如 i2c_software_ops_gd32.h）：
 *    - 声明一套新函数（i2c_gd32_start_transfer / i2c_gd32_is_busy）
 *    - 声明新的 I2C_PlatformOps_t 实例 i2c_software_platform_ops_gd32
 *    - 函数签名与本文件完全一致（void *i2c_context 参数）
 *
 *  useri2c.h 中的 I2C_PlatformOps_t 是"接口契约"，
 *  本文件是"STM32 软件 I2C 对该契约的实现声明"。
 * ============================================================================
 */

#ifndef __I2C_SOFTWARE_OPS_H__
#define __I2C_SOFTWARE_OPS_H__

#include "useri2c.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================
//  STM32 软件 I2C 平台操作表实例
//
//  在 useri2c_ops.c 中定义，应用层使用时：
//      I2C_Handle i2c_device = {
//          .ops         = &i2c_software_platform_ops_stm32,
//          .i2c_context = &software_i2c_context
//      };
// ================================================================
extern const I2C_PlatformOps_t i2c_software_platform_ops_stm32;

// ================================================================
//  平台函数声明（实现 I2C_PlatformOps_t 的接口契约）
// ================================================================

/**
 * @brief 填入事务参数 + 启动定时器 ISR，立即返回
 *
 * 将 device_address / register_address / is_read / buffer / length / callback
 * 写入 I2C_SoftwareContext，然后启动 TIM6/7 Update 中断，CPU 立即返回主循环。
 *
 * @retval 0  启动成功
 * @retval -1 句柄忙（当前传输未结束）
 */
int8_t i2c_software_start_transfer(void *i2c_context,
                                   uint8_t device_address,
                                   uint8_t register_addr,
                                   uint8_t is_read,
                                   uint8_t *buffer, uint16_t length,
                                   I2C_TransferCallback callback);

/**
 * @brief 查询当前句柄是否正在传输
 * @retval 0  空闲
 * @retval 1  传输中
 */
uint8_t i2c_software_is_busy(void *i2c_context);

// ================================================================
//  ISR 入口（由 stm32f4xx_it.c 的 TIM6_IRQHandler 调用）
// ================================================================

/**
 * @brief 定时器 ISR 实际处理函数
 *
 * 与 PWM/UART 不同：软件 I2C 的 ISR 是定时器**主动周期触发**（每 5μs），
 * 不依赖外设事件。在此实现双层 FSM：
 *
 *   1. ISR 先查 macro_state 是否为 START/STOP → 走子 FSM
 *   2. 否则 → 走微状态机（PREPARE / SAMPLE 交替）
 *   3. 微状态机 SAMPLE 末尾 → 通知宏状态机推进
 *   4. 事务全部完成 → 关定时器 + 调 complete_callback
 *
 * 调用方（stm32f4xx_it.c）：
 * @code
 * void TIM6_IRQHandler(void) {
 *     HAL_TIM_IRQHandler(&htim6);
 *     if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
 *         __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
 *         i2c_software_timer_isr(&software_i2c_context);
 *     }
 * }
 * @endcode
 *
 * @param ctx  I2C_SoftwareContext 指针（包含所有 FSM 状态和事务参数）
 */
void i2c_software_timer_isr(I2C_SoftwareContext *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SOFTWARE_OPS_H__ */
