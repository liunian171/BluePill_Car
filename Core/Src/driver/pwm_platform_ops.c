#include "pwm_platform_ops.h"
#include "tim.h"               // STM32 HAL 库头文件，本文件是唯一依赖它的地方

/**
 * ============================================================================
 *  平台实现 — STM32 版
 * ============================================================================
 *
 *  本文件是 PWM 驱动的"平台实现层"，负责将 PWM_PlatformOps_t 中定义的
 *  抽象操作映射到 STM32 的具体硬件操作。
 *
 *  ▸ 设计要点 ◂
 *  1. 所有函数接收 void *handle，函数体第一行强转为 STM32 自己的类型
 *     → 这样 pwm.h / pwm.c 完全不认识 TIM_HandleTypeDef，实现了解耦
 *  2. 本文件是整套驱动中唯一 #include "tim.h" 的地方
 *  3. 换平台（如 GD32）时，新建 pwm_gd32_ops.c，编译本文件即可
 *
 *  ▸ 参数命名约定 ◂
 *  形参统一叫 void *handle，避免与局部变量 TIM_HandleTypeDef *htim 混淆
 * ============================================================================
 */


/*==============================================================================
 *  基本启停
 *==============================================================================*/

/**
 * @brief  启动指定通道的 PWM 输出
 *         直接委托给 STM32 HAL 库函数
 */
void pwm_stm32_start(void *handle, uint32_t channel)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief  停止指定通道的 PWM 输出
 */
void pwm_stm32_stop(void *handle, uint32_t channel)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    HAL_TIM_PWM_Stop(htim, channel);
}


/*==============================================================================
 *  读 —— 公有参数（PSC / ARR / 时钟频率）
 *==============================================================================*/

/**
 * @brief  获取预分频器值（Prescaler）
 *         直接从硬件寄存器读取，确保读到的是实际生效的值
 */
uint16_t pwm_stm32_get_psc(void *handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    return (uint16_t) htim->Instance->PSC;
}

/**
 * @brief  获取自动重装载值（Auto-Reload）
 *         直接从硬件寄存器读取
 */
uint32_t pwm_stm32_get_arr(void *handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    return __HAL_TIM_GET_AUTORELOAD(htim);
}

/**
 * @brief  获取定时器时钟频率
 *
 *         注意：STM32 的 TIM 时钟挂载在 APB1 或 APB2 总线上，
 *         当 APB 预分频器 ≠ 1 时，TIM 实际时钟 = APB 频率 × 2。
 *         当前查询 APB1 时直接 ×2，APB2 则返回原始频率，
 *         实际上 APB2 也需要判断是否 ×2，此处为简化处理。
 *
 *         换平台时，此函数是重点修改对象（时钟树完全不同）。
 */
uint32_t pwm_stm32_get_clk_freq(void *handle)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    (void)htim;

    // STM32F103 定时器时钟规则：
    //   APB1 预分频 ≠ 1 → TIM 时钟 = PCLK × 2
    //   APB2 预分频 = 1 → TIM 时钟 = PCLK (不分频)
    //
    // 当前项目时钟配置：
    //   APB1 = HCLK/2 = 36MHz → TIM2/3/4 = 36×2 = 72MHz
    //   APB2 = HCLK/1 = 72MHz → TIM1     = 72MHz (不分频)

    /* APB1 上的定时器 (STM32F103: TIM2, TIM3, TIM4) */
    if (htim->Instance == TIM2 || htim->Instance == TIM3 ||
        htim->Instance == TIM4)
    {
        return HAL_RCC_GetPCLK1Freq() * 2;
    }
    else
    {
        /* APB2 上的定时器 (TIM1, TIM8) — 需判断 APB2 预分频 */
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        uint32_t hclk  = HAL_RCC_GetHCLKFreq();
        /* 若 PCLK2 < HCLK，说明 APB2 有分频 → TIM 时钟 = PCLK2 × 2 */
        return (pclk2 < hclk) ? (pclk2 * 2) : pclk2;
    }
}


/*==============================================================================
 *  读 —— 通道私有参数（CCR）
 *==============================================================================*/

/**
 * @brief  获取指定通道的比较值（Capture/Compare Register）
 */
uint32_t pwm_stm32_get_ccr(void *handle, uint32_t channel)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    return __HAL_TIM_GET_COMPARE(htim, channel);
}


/*==============================================================================
 *  写 —— 公有参数（PSC / ARR）
 *==============================================================================*/

/**
 * @brief  设置预分频器值
 *
 *         同时更新 Init 结构体（保持 get_psc 通过 Init 读取时的同步）
 *         和硬件寄存器（实际生效）。
 *
 *         注意：PSC 是影子寄存器，写入后会在下次更新事件（UEV）
 *         ——即当前周期自然结束，CNT 溢出时——自动加载到工作寄存器。
 *         无需强制触发 UEV，避免周期中途突变产生毛刺。
 */
void pwm_stm32_set_psc(void *handle, uint32_t psc)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    htim->Init.Prescaler = psc;
    __HAL_TIM_SET_PRESCALER(htim, psc);
}

/**
 * @brief  设置自动重装载值
 *
 *         TIM2/TIM5 是 32 位定时器，直接写入 uint32_t；
 *         其余定时器为 16 位，硬件会自动截取低 16 位。
 *
 *         同样，ARR 是影子寄存器（ARPE 默认使能），
 *         写入后等当前周期结束自然加载，不强制触发 UEV。
 */
void pwm_stm32_set_arr(void *handle, uint32_t arr)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    htim->Init.Period = arr;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
}


/*==============================================================================
 *  写 —— 通道私有参数（CCR）
 *==============================================================================*/

/**
 * @brief  设置指定通道的比较值
 *         TIM2/TIM5 使用 32 位比较值，其余使用 16 位
 */
void pwm_stm32_set_ccr(void *handle, uint32_t channel, uint32_t ccr)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;
    /* F103 所有定时器都是 16 位 */
    __HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)ccr);
}


/*==============================================================================
 *  操作表实例定义
 *
 *  将以上所有函数指针填入 PWM_PlatformOps_t 结构体，
 *  上层通过 PWM_Handle.ops 引用此表，实现"多态调用"。
 *
 *  例：hpwm->ops->start(hpwm->htim, hpwm->Channel);
 *      → 实际调用 pwm_stm32_start()
 *      → 替换为 &pwm_platform_ops_gd32 后，同样代码调用 pwm_gd32_start()
 *==============================================================================*/
PWM_PlatformOps_t pwm_platform_ops_stm32 =
{
    .start       = &pwm_stm32_start,
    .stop        = &pwm_stm32_stop,
    .get_psc     = &pwm_stm32_get_psc,
    .get_arr     = &pwm_stm32_get_arr,
    .get_ccr     = &pwm_stm32_get_ccr,
    .set_psc     = &pwm_stm32_set_psc,
    .set_arr     = pwm_stm32_set_arr,
    .set_ccr     = &pwm_stm32_set_ccr,
    .get_clk_freq = &pwm_stm32_get_clk_freq,
};
