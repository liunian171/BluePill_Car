#ifndef MOTOR_BRIDGE_H_
#define MOTOR_BRIDGE_H_

#include <stdint.h>
#include "pwm.h"
#include "usergpio.h"

#define MAX_MOTORS 8

typedef enum
{
    MOTOR_PROTOCOL_TB6612 = 0,
    MOTOR_PROTOCOL_L298N = 1,
} MotorProtocol_t;

#ifdef __cplusplus
extern "C" {
#endif

void motor_bridge_init(uint8_t id,
                       PWM_Handle *pwm,
                       UserGPIO_Handle *ain1_gpio,
                       UserGPIO_Handle *ain2_gpio,
                       UserGPIO_Handle *stby_gpio,
                       float max_rpm,
                       float wheel_radius_mm);

void motor_bridge_set_speed_rpm(uint8_t id, float rpm);
void motor_bridge_set_speed_mps(uint8_t id, float mps);
void motor_bridge_stop(uint8_t id);
void motor_bridge_brake(uint8_t id);
void motor_bridge_set_dead_zone(uint8_t id, float rpm);

#ifdef __cplusplus
}
#endif

#endif
