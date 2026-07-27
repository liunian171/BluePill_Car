#ifndef PWM_PLATFORM_OPS_H
#define PWM_PLATFORM_OPS_H

#include "pwm.h"
#include  "stm32f1xx_hal.h"
/**
 * ============================================================================
 *  平台操作表 — STM32 平台实现声明
 * ============================================================================
 *
 *  本文件声明 STM32 平台下所有 ops 函数的原型及操作表实例。
 *
 *  新增平台（如 GD32）时应创建同名文件（如 pwm_gd32_ops.h）：
 *    - 声明一套新的 pwm_gd32_xxx() 函数
 *    - 声明一个新的 PWM_PlatformOps_t 实例 pwm_platform_ops_gd32
 *    - 接口签名与本文件完全一致（void *handle 参数）
 *
 *  pwm.h 中的 PWM_PlatformOps_t 定义是"接口契约"，
 *  本文件是"STM32 对该契约的实现声明"。
 * ============================================================================
 */


/*----- 基本启停 -----*/
void pwm_stm32_start(void *htim, uint32_t channel);
void pwm_stm32_stop(void *htim, uint32_t channel);

/*----- 读写公有参数（作用于整个定时器）-----*/
uint16_t pwm_stm32_get_psc(void *htim);
uint32_t pwm_stm32_get_arr(void *htim);
void pwm_stm32_set_psc(void *htim, uint32_t psc);
void pwm_stm32_set_arr(void *htim, uint32_t arr);

uint32_t pwm_stm32_get_clk_freq(void *htim);

/*----- 读写通道私有参数（作用于单个通道）-----*/
uint32_t pwm_stm32_get_ccr(void *htim, uint32_t channel);
void pwm_stm32_set_ccr(void *htim, uint32_t channel, uint32_t ccr);


/*==============================================================================
 *  STM32 平台操作表实例
 *  在 pwm_platform_ops.c 中定义，应用层使用时赋值给 PWM_Handle.ops
 *  例：PWM_Handle pwm_ch1 = { .ops = &pwm_platform_ops_stm32 };
 *==============================================================================*/
extern PWM_PlatformOps_t pwm_platform_ops_stm32;


#endif
