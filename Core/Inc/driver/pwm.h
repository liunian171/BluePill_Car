#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ============================================================================
 *  PWM 驱动 — 抽象层（与具体芯片无关）
 * ============================================================================
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │  架构总览（三层分离）                                            │
 *  │                                                                 │
 *  │  应用层（调用方）                                                │
 *  │    pwm_set_freq(hpwm, 50Hz)  /  pwm_set_duty_0E3(hpwm, 500)    │
 *  │                    │                                           │
 *  │                    ▼                                           │
 *  │  ┌─ pwm.c ──────────────────────────────────────────────────┐  │
 *  │  │  策略层（跨平台通用，与芯片无关）                          │  │
 *  │  │  职责：组合调用 ops + 纯算法计算                          │  │
 *  │  │  换平台时：此文件零修改                                    │  │
 *  │  └──────────────────────┬──────────────────────────────────┘  │
 *  │                         │ hpwm->ops->xxx()                    │
 *  │                         ▼                                     │
 *  │  ┌─ pwm_platform_ops.c ──────────────────────────────────┐  │
 *  │  │  平台实现层（每款芯片一套）                              │  │
 *  │  │  职责：直接操作硬件寄存器 / HAL 库                       │  │
 *  │  │  换平台时：新增一套 .c/.h，接口签名不变                  │  │
 *  │  └─────────────────────────────────────────────────────────┘  │
 *  └─────────────────────────────────────────────────────────────────┘
 *
 *  核心解耦手段：
 *    1. void *handle     — 隐藏具体芯片的句柄类型
 *    2. 操作表 ops       — 函数指针集合，多态的实现方式
 *    3. PWM_Handle 句柄  — 串联 handle + ops + channel 信息的统一结构
 * ============================================================================
 */


/*==============================================================================
 *  通道状态枚举
 *
 *  注意：当前驱动采用"实时读/写硬件寄存器"模式（通过 ops），
 *        Ch_State 字段已在 PWM_Handle 中预留但尚未接入运行逻辑，
 *        所有策略函数不会自动更新此状态。
 *        如需使用（如错误检测、多通道占用管理），需手动在对应函数中维护。
 *==============================================================================*/
typedef enum PWM_Ch_State
{
	PWM_Ch_State_OK,        //通道正常
	PWM_Ch_State_Using,     //通道正在使用
    PWM_Ch_State_Uninit,    //通道未初始化
	PWM_Ch_State_Error      //通道错误
} PWM_Ch_State;

/*==============================================================================
 *  定时器全局参数结构体（预留 — 当前未被使用）
 *
 *  设计意图：用于存储 PSC / ARR 等与具体定时器相关的配置参数，
 *           供未来需要"批量获取/恢复 TIM 参数"的场景使用。
 *
 *  当前状态：只有 typedef，全工程无任何实例化或引用。
 *           驱动当前全部通过 ops->get_arr/set_arr/get_psc/set_psc
 *           直接读/写硬件寄存器，不经过此结构体。
 *           待有明确需求（如快照保存/恢复）时再启用。
 *==============================================================================*/
typedef struct TIM_PWM_g_Param
{
    uint16_t PSC_16;
    uint32_t ARR_32;
} TIM_PWM_g_Param;

/*==============================================================================
 *  平台操作表类型定义 — 接口抽象核心
 *
 *  用途：定义一组与硬件交互的"原语操作"，每个平台各自实现一套
 *  所有函数指针的第一个参数统一为 void *htim，不暴露具体芯片类型
 *
 *  添加新操作时要注意：
 *    - 操作必须是与硬件寄存器打交道的"原子操作"
 *    - 组合逻辑（读 → 算 → 写）应放在 pwm.c 的策略层，而非 ops 内
 *      例：读占空比 = 读CCR + 读ARR + 公式计算 → 应在 pwm.c
 *          设置占空比 = 读ARR + 算CCR + 写CCR → 应在 pwm.c
 *==============================================================================*/
// 先 typedef（前向 + 类型定义放在一起，顺序对）


/*==============================================================================
*  PWM 句柄 — 所有操作的中心数据结构
*  应用层通过此结构体与驱动交互
*==============================================================================*/
// 再定义完整的 PWM_PlatformOps_t 结构体
typedef struct PWM_PlatformOps_t
{

/*----- 基本启停 -----*/
    void (*start)(void *htim, uint32_t channel);            // 启动指定通道的 PWM 输出
    void (*stop)(void *htim, uint32_t channel);             // 停止指定通道的 PWM 输出

/*----- 读写与平台公有参数（作用于整个定时器）-----*/
    uint16_t (*get_psc)(void *htim);                        // 读取预分频器（PSC）值
    uint32_t (*get_arr)(void *htim);                        // 读取自动重装载（ARR）值
    void (*set_psc)(void *htim, uint32_t psc);              // 写入预分频器（PSC）值
    void (*set_arr)(void *htim, uint32_t arr);              // 写入自动重装载（ARR）值

    uint32_t (*get_clk_freq)(void *htim);                   // 获取定时器时钟频率

/*----- 读写通道私有参数（作用于单个通道）-----*/
    uint32_t (*get_ccr)(void *htim, uint32_t channel);      // 读取指定通道的比较值（CCR）
    void (*set_ccr)(void *htim, uint32_t channel, uint32_t ccr);    // 写入指定通道的比较值（CCR）

} PWM_PlatformOps_t;



typedef struct  
{
    void *htim;                             // 平台句柄（不关心具体类型）
                                            // 例：STM32 中是 TIM_HandleTypeDef *
                                            //     GD32 中可能是 gd32_timer_regs *

    PWM_PlatformOps_t *ops;           // 当前平台的操作表指针
                                            // 决定了调用 start/stop/get_arr 等操作时
                                            // 执行哪一套平台函数

    uint32_t Channel;                       // 当前使用的 PWM 通道号
                                            // 例：TIM_CHANNEL_1 / TIM_CHANNEL_2

    PWM_Ch_State Ch_State;                  // 当前通道的状态
                                            // 注意：当前驱动未自动维护此字段，
                                            //       set_freq/set_duty/start/stop
                                            //       均不更新 Ch_State，需手动管理。

    uint32_t PWM_CCR;                       // 缓存当前通道的比较值(CCR) — 预留
                                            // 注意：当前驱动不读取/写入此字段。
                                            //       set_duty_0E3 执行流程：
                                            //         get_arr(硬件) → 算new_ccr → set_ccr(硬件)
                                            //       全程不经过 PWM_CCR。此字段待未来
                                            //       有"缓存优先"或"快照恢复"需求时再接入。

} PWM_Handle;



/*==============================================================================
 *  策略层函数声明（实现在 pwm.c）
 *  这些函数通过 hpwm->ops 调用底层硬件操作，自身只做组合逻辑和数学计算
 *==============================================================================*/

/**
 * @brief  读取当前占空比（千分比，0～1000）
 *         组合逻辑：ops->get_ccr() → ops->get_arr() → 公式计算
 *         公式：duty = CCR * 1000 / (ARR + 1)
 */
uint16_t pwm_get_duty_0E3(PWM_Handle *hPWM);

/**
 * @brief  设置占空比（千分比，0～1000）
 *         组合逻辑：ops->get_arr() → 新CCR = duty * ARR / 1000 → ops->set_ccr()
 */
void pwm_set_duty_0E3(PWM_Handle *hPWM, uint16_t duty_cycle);



/**
 * @brief  获取当前频率（单位：Hz）
 *         组合逻辑：ops->get_clk_freq() → ops->get_psc() → ops->get_arr() → 公式计算
 *         公式：freq = clk_freq / ( (psc + 1) * (arr + 1) )
 */
uint32_t pwm_get_freq(PWM_Handle *hPWM);
/**
 * @brief  设置频率（单位：Hz）
 *         组合逻辑：保存当前占空比 → 查预设ARR表 → 计算新PSC → 写入ARR和PSC → 恢复占空比
 *         psc = clk_freq / ( (arr + 1) * freq ) - 1
 */
void pwm_set_freq(PWM_Handle *hPWM, uint32_t freq);


uint16_t pwm_get_psc_calculate(PWM_Handle *hPWM, uint32_t arr, uint32_t new_pwm_freq);

void pwm_start(PWM_Handle *hPWM);

void pwm_stop(PWM_Handle *hPWM);

/* PWM 实例：具体的通道对象，在 pwm_instance.c 中定义 */
extern PWM_Handle pwm_tim1_ch1;      /* 电机A PWM (PA8) */
extern PWM_Handle pwm_tim1_ch2;      /* 电机B PWM (PA9) */
extern PWM_Handle pwm_tim4_ch3;      /* 舵机 PWM (PB8) */

#ifdef __cplusplus
}
#endif

#endif
