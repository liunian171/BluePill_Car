/**
 * @file    pwm_instance.c
 * @brief   PWM 实例定义 — Bluepill 平台
 *
 * 将 PWM_Handle 实例从 pwm.c 中分离出来，使 pwm.c 保持纯策略层。
 * 换平台时只需重写此文件。
 */
#include "pwm.h"
#include "pwm_platform_ops.h"
#include "tim.h"

/* 电机A PWM — TIM1_CH1 (PA8) */
PWM_Handle pwm_tim1_ch1 = {
    .htim     = &htim1,
    .ops      = &pwm_platform_ops_stm32,
    .Channel  = TIM_CHANNEL_1,
    .Ch_State = PWM_Ch_State_OK,
    .PWM_CCR  = 0,
};

/* 电机B PWM — TIM1_CH2 (PA9) */
PWM_Handle pwm_tim1_ch2 = {
    .htim     = &htim1,
    .ops      = &pwm_platform_ops_stm32,
    .Channel  = TIM_CHANNEL_2,
    .Ch_State = PWM_Ch_State_OK,
    .PWM_CCR  = 0,
};

/* 舵机 PWM — TIM4_CH3 (PB8) */
PWM_Handle pwm_tim4_ch3 = {
    .htim     = &htim4,
    .ops      = &pwm_platform_ops_stm32,
    .Channel  = TIM_CHANNEL_3,
    .Ch_State = PWM_Ch_State_OK,
    .PWM_CCR  = 0,
};
