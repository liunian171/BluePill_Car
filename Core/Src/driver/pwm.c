#include "pwm.h"
#include "tim.h"
#include "pwm_platform_ops.h"            
                                // TODO: 仅因下方实例定义引用了 htim5 / TIM_CHANNEL_4
                               //       待实例移至 pwm_instance.c 后可删除此行
#include <stdint.h>

/**
 * ============================================================================
 *  PWM 驱动 — 策略层（跨平台通用）
 * ============================================================================
 *
 *  本文件包含 PWM 驱动的"策略函数"——它们通过 hPWM->ops 调用底层
 *  硬件操作，自身只做组合逻辑和数学计算，不直接碰任何硬件寄存器。
 *
 *  ▸ 设计分界线 ◂
 *  本文件中的函数（策略层）与 pwm_platform_ops.c（平台实现层）的界限：
 *
 *    ┌─ 策略层（本文件）─────────────┬─ 平台实现层（ops）──────────┐
 *    │  pwm_set_freq()               │  start() / stop()          │
 *    │  pwm_set_duty_0E3()           │  get_psc() / set_psc()     │
 *    │  pwm_get_duty_0E3()           │  get_arr() / set_arr()     │
 *    │  pwm_get_pre_arr()            │  get_ccr() / set_ccr()     │
 *    │  pwm_get_psc_calculate()      │  get_clk_freq()            │
 *    ├───────────────────────────────┼────────────────────────────┤
 *    │  组合逻辑（读→算→写）          │  原子操作（直接读写寄存器）   │
 *    │  跨平台通用                    │  每款芯片一套               │
 *    │  换平台零修改                  │  换平台整个文件替换          │
 *    └───────────────────────────────┴────────────────────────────┘
 *
 *  ▸ 为什么 pwm_set_duty_0E3 / pwm_get_duty_0E3 放在这里而不是 ops？ ◂
 *  因为它们内部调用了 ops->get_arr() + ops->set_ccr() 两个原语，
 *  并夹带了"公式计算"的纯业务逻辑——没有直接碰硬件。
 *  如果放在 ops 里，每个平台都要重复写一遍相同的计算公式，显然不合理。
 * ============================================================================
 */


/*==============================================================================
 *  ARR 频率区间表
 *
 *  用途：为不同的频率范围预设 ARR 值，保证定时器计数值落在合理范围内，
 *        避免因 ARR 过大或过小导致 PSC 计算精度下降。
 *
 *  例如：
 *    20Hz ～ 1kHz  → ARR = 19999（舵机控制常用范围）
 *    1kHz ～ 100kHz → ARR = 1999（电机控制常用范围）
 *
 *  每一行：{ 最小频率, 最大频率, 预设 ARR }
 *  注意：
 *    - 频率范围用 uint32_t（最大100kHz 超过 uint16_t 的 65535）
 *    - ARR 用 uint32_t（TIM2/TIM5 是 32 位定时器，支持更大的 ARR 值）
 *==============================================================================*/
static const struct {
    uint32_t min_freq;
    uint32_t max_freq;
    uint32_t arr;
} freq_ranges[] = {
    {20, 1000, 20000 - 1},      // 低频段（舵机用），ARR=19999
    {1000, 100000, 2000 - 1},   // 高频段（电机用），ARR=1999
};


/*==============================================================================
 *  工具函数
 *==============================================================================*/

/**
 * @brief  根据目标频率查询预设的 ARR 值
 *         遍历 freq_ranges 表，返回匹配区间的 ARR
 * @param  freq  目标频率（Hz）
 * @return uint32_t  预设 ARR 值；若未匹配到任何区间则返回 0
 */
uint32_t pwm_get_pre_arr(uint32_t freq)
{
    for (uint8_t i = 0; i < sizeof(freq_ranges) / sizeof(freq_ranges[0]); i++)
    {
        if (freq >= freq_ranges[i].min_freq && freq <= freq_ranges[i].max_freq)
        {
            return freq_ranges[i].arr;
        }
    }
    return 0;       // 不在任何区间内，由调用方决定保持原值
}

/**
 * @brief  根据目标频率和 ARR 值计算所需的 PSC 值
 *
 *         推导：
 *           TIM 输出频率 = clk_freq / ((ARR + 1) × (PSC + 1))
 *           → PSC = clk_freq / ( (ARR + 1) × freq ) - 1
 *
 * @param  hPWM     PWM 句柄（用于获取 clk_freq）
 * @param  arr      预设的 ARR 值
 * @param  new_freq 目标频率
 * @return uint16_t 计算得到的 PSC 值
 */
uint16_t pwm_get_psc_calculate(PWM_Handle *hPWM, uint32_t arr, uint32_t new_pwm_freq)
{
    uint32_t clk_freq = hPWM->ops->get_clk_freq(hPWM->htim);
    uint16_t psc = clk_freq / ((arr + 1) * new_pwm_freq) - 1;
    return psc;
}


/*==============================================================================
 *  占空比操作
 *==============================================================================*/

/**
 * @brief  设置占空比（千分比，0～1000）
 *
 *         流程：
 *          1. 通过 ops->get_arr() 获取当前 ARR
 *          2. 新 CCR = duty × ARR / 1000
 *          3. 若新 CCR 与当前不同，通过 ops->set_ccr() 写入
 *
 *         注意：duty 为 0 表示 0%，1000 表示 100%
 *               如 duty=500，对应 50% 占空比
 */
void pwm_set_duty_0E3(PWM_Handle *hPWM, uint16_t duty_cycle)
{
    uint32_t current_arr = hPWM->ops->get_arr(hPWM->htim);
    uint32_t new_ccr = (duty_cycle * current_arr) / 1000;

    if (hPWM->ops->get_ccr(hPWM->htim, hPWM->Channel) != new_ccr)
        hPWM->ops->set_ccr(hPWM->htim, hPWM->Channel, new_ccr);
}

/**
 * @brief  读取当前占空比（千分比，0～1000）
 *
 *         流程：
 *          1. 通过 ops->get_ccr() 获取当前 CCR
 *          2. 通过 ops->get_arr() 获取当前 ARR
 *          3. 计算 duty = CCR × 1000 / (ARR + 1)
 *
 *         注意：此函数曾放在 ops 内，每个平台重复实现了一次相同的公式。
 *               移至本层后，所有平台共享同一份代码。→ 这就是分层的好处。
 */
uint16_t pwm_get_duty_0E3(PWM_Handle *hPWM)
{
    uint32_t ccr = hPWM->ops->get_ccr(hPWM->htim, hPWM->Channel);
    uint32_t arr = hPWM->ops->get_arr(hPWM->htim);
    return (uint16_t)(ccr * 1000 / (arr + 1));
}


/*==============================================================================
 *  频率操作
 *==============================================================================*/

/**
 * @brief  设置频率（单位：Hz）
 *
 *         流程：
 *          1. 通过 ops->get_arr() 获取当前 ARR
 *          2. 通过 pwm_get_duty_0E3() 获取当前占空比（保证切换后占空比不变）
 *          3. 通过 pwm_get_pre_arr() 查询新频率对应的预设 ARR
 *          4. 若 ARR 需要变化：
 *             a. ops->set_arr() 写入新 ARR
 *             b. pwm_get_psc_calculate() 计算新 PSC
 *             c. ops->set_psc() 写入新 PSC
 *          5. pwm_set_duty_0E3() 恢复原来的占空比
 *
 *         PSC 和 ARR 都是影子寄存器，写入硬件后会在当前周期自然结束
 *         （CNT 溢出触发 UEV）时自动加载，无需软件强制触发，
 *         避免周期中途突变产生毛刺。
 */
void pwm_set_freq(PWM_Handle *hPWM, uint32_t freq)
{
    uint32_t current_arr = hPWM->ops->get_arr(hPWM->htim);
    uint16_t current_duty = pwm_get_duty_0E3(hPWM);

    uint32_t new_arr = pwm_get_pre_arr(freq);
    if (new_arr != current_arr)     // 需要切换 ARR 区间
    {
        hPWM->ops->set_arr(hPWM->htim, new_arr);
    }
    uint16_t new_psc = pwm_get_psc_calculate(hPWM, new_arr, freq);
    hPWM->ops->set_psc(hPWM->htim, new_psc);

    // 保持占空比不变（ARR 变化后，相同 CCR 对应的占空比会变，故重新计算）
    pwm_set_duty_0E3(hPWM, current_duty);
}

uint32_t pwm_get_freq(PWM_Handle *hPWM)
{
    //tim时钟,psc,arr
    uint32_t clk_freq = hPWM->ops->get_clk_freq(hPWM->htim);
    uint16_t psc = hPWM->ops->get_psc(hPWM->htim);
    uint32_t arr = hPWM->ops->get_arr(hPWM->htim);
    return clk_freq / ((psc + 1) * (arr + 1));
}

void pwm_start(PWM_Handle *hPWM)
{
    hPWM->ops->start(hPWM->htim, hPWM->Channel);
}

void pwm_stop(PWM_Handle *hPWM)
{
    hPWM->ops->stop(hPWM->htim, hPWM->Channel);
}


// 实例定义已移至 pwm_instance.c（换平台时重写该文件）
// 本文件专注纯策略函数，与平台无关。
