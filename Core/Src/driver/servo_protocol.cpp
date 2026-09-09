#include "servo_protocol.h"
#include "tool.h"
//pwm控制协议只需要控制一个pwm舵机,处理脉宽
PWMServoProtocol::PWMServoProtocol(PWM_Handle& handle):
    hpwm(handle)
    {};

void PWMServoProtocol::set_position(uint16_t rate_0E3)
{
    start();                                        // 确保 PWM 已启动（HAL 幂等，重复调无副作用）
    // 0E3(0~1000) → 脉宽(µs): min_pulse~max_pulse (本项目 500~2500µs / 270° 舵机)
    // 注: rate 量化步长 = 行程/1000 = 0.27°, 足够转向微步
    uint32_t pulse_us = (uint32_t) map(rate_0E3, 0, 1000, min_pulse, max_pulse);
    // 脉宽直写 CCR (1µs 分辨率) — 0E3 占空比接口在 50Hz 下量化 20µs ≈ 2.7°, 精度不足
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