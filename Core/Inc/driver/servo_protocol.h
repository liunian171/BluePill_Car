#ifndef __SERVO_PROTOCOL_H__
#define __SERVO_PROTOCOL_H__
#include "servo.h"
#include "pwm.h"
class PWMServoProtocol: public IServoProtocol
{
public:
     // 该协议实现需要的属性以及方法
     //下层传入的句柄
     PWM_Handle& hpwm;
     //舵机频率->规定arr,psc

     //脉冲宽度->规定ccr
     uint16_t min_pulse=500;
     uint16_t max_pulse=2500;

     //方法
     //构造函数
     PWMServoProtocol(PWM_Handle& handle);
     void set_position(uint16_t rate_0E3) override;
     void start() override;
     void stop() override;
};

#endif
