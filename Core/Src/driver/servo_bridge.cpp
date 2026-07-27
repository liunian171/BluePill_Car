#include "servo_bridge.h"
#include "servo_protocol.h"          // 需要 PWMServoProtocol 的完整定义

static PWMServoProtocol *pwm_servo_protoc[MAX_SERVOS] = {nullptr};
static Servo* servo[MAX_SERVOS] = {nullptr};

void servo_bridge_init(uint8_t id, ServoProtocol_t protocol, void *handle)
{
    switch (protocol)
    {
    case SERVO_PROTOCOL_PWM:
        pwm_servo_protoc[id] = new PWMServoProtocol(*(PWM_Handle*)handle);
        servo[id] = new Servo(*pwm_servo_protoc[id]);
        break;
    default:
        break;
    }
}

void servo_bridge_set_angle(uint8_t id, float angle)
{
    if (servo[id] == nullptr) return;
    servo[id]->set_angle(angle);
}

void servo_bridge_start(uint8_t id)
{
    if (servo[id] == nullptr) return;
    servo[id]->start();
}

void servo_bridge_stop(uint8_t id)
{
    if (servo[id] == nullptr) return;
    servo[id]->stop();
}