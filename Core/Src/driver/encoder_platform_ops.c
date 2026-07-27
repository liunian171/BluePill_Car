/**
 * @file    encoder_platform_ops.c
 * @brief   编码器平台层 — STM32 TIM Encoder Mode
 */

#include "encoder.h"
#include "tim.h"        // htim2

static void start(void *htim)
{
    HAL_TIM_Encoder_Start((TIM_HandleTypeDef *)htim, TIM_CHANNEL_ALL);
}

static void stop(void *htim)
{
    HAL_TIM_Encoder_Stop((TIM_HandleTypeDef *)htim, TIM_CHANNEL_ALL);
}

static int32_t get_counter(void *htim)
{
    return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)htim);
}

static void set_counter(void *htim, int32_t cnt)
{
    __HAL_TIM_SET_COUNTER((TIM_HandleTypeDef *)htim, (uint32_t)cnt);
}

static Encoder_PlatformOps_t g_encoder_ops = {
    .start       = start,
    .stop        = stop,
    .get_counter = get_counter,
    .set_counter = set_counter,
};

Encoder_PlatformOps_t *encoder_platform_get_ops(void)
{
    return &g_encoder_ops;
}
