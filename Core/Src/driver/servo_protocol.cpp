#include "servo_protocol.h"
#include "tool.h"
//pwm控制协议只需要控制一个pwm舵机,处理脉宽
PWMServoProtocol::PWMServoProtocol(PWM_Handle& handle):
    hpwm(handle)
    {};

void PWMServoProtocol::set_position(uint16_t rate_0E3)
{
    start();                                        // 确保 PWM 已启动（HAL 幂等，重复调无副作用）
    //找pwm驱动内改占空比的函数,前提是tim的频率已经设置好(如50hz->pwm驱动初始化)
    //占空比=pulse/tim_period
    //获取当前频率 ->当前周期
    uint32_t pulse_us = (uint32_t) map(rate_0E3, 0, 1000, min_pulse, max_pulse);
    uint32_t pwm_freq = pwm_get_freq(&hpwm);
    if (pwm_freq == 0) return;                      // 避免除零
    uint32_t pwm_us_period_0E3 = 1000000 / pwm_freq; // 周期（微秒），0E3 = 千分级定点数
    uint32_t duty_0E3 = pulse_us * 1000 / pwm_us_period_0E3;
    pwm_set_duty_0E3(&hpwm, (uint16_t)duty_0E3);
}

void PWMServoProtocol::start()
{
    pwm_start(&hpwm);
}

void PWMServoProtocol::stop()
{
    pwm_stop(&hpwm);
}