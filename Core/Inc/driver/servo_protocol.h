#ifndef SERVO_PROTOCOL_H
#define SERVO_PROTOCOL_H

/**
 * ============================================================================
 *  器件层 — PWMServoProtocol（PWM 舵机器件翻译）
 * ============================================================================
 *
 *  层位：器件层（执行栈底端）—— 本类只做"这一种器件"的翻译规则
 *  职责：千分比(0E3) → 脉宽 µs（PWM 舵机的器件语言）
 *
 *  ▸ 单位与坐标域声明（义务 8）◂
 *    输入：千分比 0~1000（0 = min_pulse 端）
 *    输出：脉宽 µs（经 pwm_set_pulse_us 直写 CCR）
 *
 *  ▸ 器件极限兜底（本层，器件知识）◂
 *    脉宽显式钳到 [min_pulse, max_pulse]（本项目 500~2500µs / 270° 舵机）。
 *    注：策略层 pwm_set_pulse_us 另有"占空比 ≤ 100%"的**周期上限**兜底
 *    （50Hz → 20000µs），语义不同、对舵机无效（500~2500µs 远小于它），
 *    故舵机安全必须由本层显式保证，不能依赖那道通用兜底。
 *
 *  依赖白名单：PWM_Handle（策略层句柄，构造注入）、pwm 策略层函数
 *  可移植性：换舵机器件 —— 新增 IServoProtocol 派生类，不动本文件；
 *            换 MCU 平台 —— 不动本文件（pwm 策略层已抽象）
 * ============================================================================
 */

#include "servo.h"
#include "pwm.h"

class PWMServoProtocol : public IServoProtocol
{
public:
    PWM_Handle& hpwm;

    /* 器件脉宽极限(µs)：默认值即本项目 270° 舵机（500~2500µs） */
    uint16_t min_pulse;
    uint16_t max_pulse;

    PWMServoProtocol(PWM_Handle& handle, uint16_t min_pulse = 500, uint16_t max_pulse = 2500);
    void set_position(uint16_t rate_0E3) override;
    void start() override;
    void stop() override;
};

#endif
