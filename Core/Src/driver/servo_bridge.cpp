/**
 * ============================================================================
 *  桥接层 — servo_bridge 实现（C 门面）
 * ============================================================================
 *
 *  家族契约：见 servo_bridge.h 文件头 / doc/桥接层家族契约.md
 * ============================================================================
 */

#include "servo_bridge.h"
#include "servo_protocol.h"          /* PWMServoProtocol / PWM_Handle 完整定义（已含 pwm.h） */
#include <new>                       /* placement new */
#include <stddef.h>

/* 静态池替代堆 new (heap 仅 512B, 分配失败静默返回 NULL → 硬错误, 见调试总结 §4) */
static uint8_t pwm_protoc_pool[MAX_SERVOS][sizeof(PWMServoProtocol)];
static uint8_t servo_pool[MAX_SERVOS][sizeof(Servo)];
static PWMServoProtocol *pwm_servo_protoc[MAX_SERVOS] = {nullptr};
static Servo *servo[MAX_SERVOS] = {nullptr};

/* 统一入口校验：id 越界 / 未初始化一律显式返回，不静默 */
static bridge_ret_t servo_bridge_chk(uint8_t id)
{
    if (id >= MAX_SERVOS) return BRIDGE_ERR_BAD_ARG;
    if (servo[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return BRIDGE_OK;
}

bridge_ret_t servo_bridge_init(uint8_t id, const servo_bridge_cfg_t *cfg)
{
    if (cfg == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (id >= MAX_SERVOS) return BRIDGE_ERR_BAD_ARG;
    if (cfg->handle == nullptr) return BRIDGE_ERR_BAD_ARG;   /* ← 原实现未查 NULL 即解引用 */

    switch (cfg->protocol)
    {
    case SERVO_PROTOCOL_PWM:
        pwm_servo_protoc[id] = new (pwm_protoc_pool[id])
                               PWMServoProtocol(*(PWM_Handle *)cfg->handle);
        servo[id] = new (servo_pool[id]) Servo(*pwm_servo_protoc[id]);
        return BRIDGE_OK;

    default:
        return BRIDGE_ERR_BAD_CFG;   /* 未实现协议：显式报错，不静默 break */
    }
}

bridge_ret_t servo_bridge_set_angle(uint8_t id, float angle)
{
    bridge_ret_t r = servo_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    servo[id]->set_angle(angle);
    return BRIDGE_OK;
}

bridge_ret_t servo_bridge_start(uint8_t id)
{
    bridge_ret_t r = servo_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    servo[id]->start();
    return BRIDGE_OK;
}
