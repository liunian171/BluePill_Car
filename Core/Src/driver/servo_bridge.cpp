#include "servo_bridge.h"
#include "servo_protocol.h"          // 需要 PWMServoProtocol 的完整定义
#include <new>                       // placement new
#include <stddef.h>

/* 静态池替代堆 new (heap 仅 512B, 分配失败静默返回 NULL → 硬错误, 见调试总结 §4) */
static uint8_t pwm_protoc_pool[MAX_SERVOS][sizeof(PWMServoProtocol)];
static uint8_t servo_pool[MAX_SERVOS][sizeof(Servo)];
static PWMServoProtocol *pwm_servo_protoc[MAX_SERVOS] = {nullptr};
static Servo* servo[MAX_SERVOS] = {nullptr};

void servo_bridge_init(uint8_t id, ServoProtocol_t protocol, void *handle)
{
    if (id >= MAX_SERVOS) return;
    switch (protocol)
    {
    case SERVO_PROTOCOL_PWM:
        pwm_servo_protoc[id] = new (pwm_protoc_pool[id]) PWMServoProtocol(*(PWM_Handle*)handle);
        servo[id] = new (servo_pool[id]) Servo(*pwm_servo_protoc[id]);
        break;
    default:
        break;
    }
}

void servo_bridge_set_angle(uint8_t id, float angle)
{
    if (id >= MAX_SERVOS || servo[id] == nullptr) return;
    servo[id]->set_angle(angle);
}

void servo_bridge_start(uint8_t id)
{
    if (id >= MAX_SERVOS || servo[id] == nullptr) return;
    servo[id]->start();
}

