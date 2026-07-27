/**
 * @file    uart_platform_ops.h
 * @brief   STM32 UART 平台操作表声明
 *
 * 提供 uart_platform_ops_stm32 操作表实例的 extern 声明以及
 * send/receive/receive_IT 三个平台函数的原型。
 *
 * 换其他 MCU 时新增一套 xxx_uart_platform_ops.h/.c，接口签名不变，
 * 策略层（uart.c）、命令解析层、环形缓冲区代码不动。
 *
 * 参考: pwm_platform_ops.h 的对应模式
 */

#ifndef __UART_PLATFORM_OPS_H__
#define __UART_PLATFORM_OPS_H__

#include "uart.h"
#include "tool.h"


/* ================================================================
 *  STM32 平台操作表实例
 *  在 uart_platform_ops.c 中定义，由 main.c 通过 uart_debug.ops 引用
 * ================================================================ */
extern const UART_PlatformOps_t uart_platform_ops_stm32;

/* ---------- 平台函数声明 ---------- */

/** @brief 阻塞发送，timeout 1000ms，返回 0 成功 / -1 失败 */
int8_t uart_stm32_send(void *huart, const uint8_t *data, uint16_t len);

/** @brief 阻塞接收，timeout 1000ms，返回 0 成功 / -1 失败 */
int8_t uart_stm32_receive(void *huart, uint8_t *data, uint16_t len);

/** @brief 启动单字节中断接收，回调中需重新调用以连续接收 */
int8_t uart_stm32_receive_IT(void *huart, uint8_t *p_byte, uint16_t len);

#endif /* __UART_PLATFORM_OPS_H__ */
