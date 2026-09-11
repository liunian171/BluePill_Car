/**
 * ============================================================================
 *  器件层 — PWMServoProtocol 实现（PWM 舵机器件翻译）
 * ============================================================================
 *
 *  翻译规则（只属于"PWM 舵机"这一类器件）：
 *    千分比(0~1000) → 脉宽(µs)：pulse = min_pulse + rate × (max_pulse - min_pulse) / 1000
 *    脉宽 → 硬件：pwm_set_pulse_us()（µs 级直写 CCR）
 *
 *  ▸ 为什么不用 0E3 占空比接口 ◂
 *    50Hz 下 0E3 量化步长 = 周期/1000 = 20µs ≈ 2.7°/步（270° 舵机），
 *    SV+ ±1° 微步推不动 → 改用脉宽直写（1µs 分辨率）。见调试总结 §13。
 * ============================================================================
 */

#include "servo_protocol.h"

PWMServoProtocol::PWMServoProtocol(PWM_Handle& handle, uint16_t min_pulse, uint16_t max_pulse)
   : hpwm(handle),
     min_pulse(min_pulse),
     max_pulse(max_pulse)
{
}

void PWMServoProtocol::set_position(uint16_t rate_0E3)
{
    /* 配置合法性：脉宽域非法时拒绝输出（不写 CCR） */
    if (max_pulse <= min_pulse) return;

    start();        /* 确保 PWM 已启动（HAL 幂等，重复调用无副作用） */

    /* 千分比域兜底（上游 Servo 已钳位，此处为防御性二次保护） */
    if (rate_0E3 > 1000u) rate_0E3 = 1000u;

    /* 千分比 → 脉宽（整数运算，避免浮点；误差 ≤1µs） */
    uint32_t span     = (uint32_t)(max_pulse - min_pulse);
    uint32_t pulse_us = (uint32_t)min_pulse + ((uint32_t)rate_0E3 * span) / 1000u;

    /* 器件脉宽极限钳位（本层职责，显式保证舵机不越行程） */
    if (pulse_us < (uint32_t)min_pulse) pulse_us = (uint32_t)min_pulse;
    if (pulse_us > (uint32_t)max_pulse) pulse_us = (uint32_t)max_pulse;

    pwm_set_pulse_us(&hpwm, pulse_us);
}

void PWMServoProtocol::start()
{
    pwm_start(&hpwm);
}

void PWMServoProtocol::stop()
{
    pwm_stop(&hpwm);
}
