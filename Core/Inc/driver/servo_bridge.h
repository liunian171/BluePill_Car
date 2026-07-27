#ifndef __SERVO_BRIDGE_H__
#define __SERVO_BRIDGE_H__

#include <stdint.h>    // for uint8_t

#define MAX_SERVOS 8

typedef enum
{
    SERVO_PROTOCOL_PWM = 0,
    SERVO_PROTOCOL_UART = 1,
} ServoProtocol_t;


#ifdef __cplusplus
extern "C" {
#endif

void servo_bridge_init(uint8_t id, ServoProtocol_t protocol, void *handle);         //绑定协议和句柄，创建对应的 Servo 对象
void servo_bridge_set_angle(uint8_t id, float angle);                       //设置角度，内部自动转换为协议需要的参数并调用协议接口  
void servo_bridge_start(uint8_t id);                                        //启动舵机（如 PWM 协议则启动 PWM 输出）
void servo_bridge_stop(uint8_t id);                     //停止舵机（如 PWM 协议则停止 PWM 输出）    //!!!!!!!!!!该函数还未测试是否有用!!!!!!!!!!!!!

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_BRIDGE_H__ */



