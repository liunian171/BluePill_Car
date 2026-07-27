#include "motor_bridge.h"
#include "motor_protocol.h"    // TB6612MotorProtocol 完整定义
#include <new>                 // placement new

/* 静态池：编译时分配好内存，不碰堆 */
static uint8_t g_proto_mem[MAX_MOTORS][sizeof(TB6612MotorProtocol)];
static uint8_t g_motor_mem[MAX_MOTORS][sizeof(Motor)];
static TB6612MotorProtocol *tb6612_proto[MAX_MOTORS] = {nullptr};
static Motor *motor[MAX_MOTORS] = {nullptr};

void motor_bridge_init(uint8_t id,
                       PWM_Handle *pwm,
                       UserGPIO_Handle *ain1_gpio,
                       UserGPIO_Handle *ain2_gpio,
                       UserGPIO_Handle *stby_gpio,
                       float max_rpm,
                       float wheel_radius_mm)
{
    /* placement new：在预分配的静态内存上构造对象，不占用堆 */
    tb6612_proto[id] = new (g_proto_mem[id]) TB6612MotorProtocol(pwm, ain1_gpio, ain2_gpio, stby_gpio);
    motor[id] = new (g_motor_mem[id]) Motor(*tb6612_proto[id], max_rpm, wheel_radius_mm);
}

void motor_bridge_set_speed_rpm(uint8_t id, float rpm)
{
    if (motor[id] == nullptr) return;
    if (id == 1) rpm = -rpm;  /* E2 硬件接反，软件取反 */
    motor[id]->set_speed_rpm(rpm);
}

void motor_bridge_set_speed_mps(uint8_t id, float mps)
{
    if (motor[id] == nullptr) return;
    if (id == 1) mps = -mps;  /* E2 硬件接反，软件取反 */
    motor[id]->set_speed_mps(mps);
}

void motor_bridge_stop(uint8_t id)
{
    if (motor[id] == nullptr) return;
    motor[id]->stop();
}

void motor_bridge_brake(uint8_t id)
{
    if (motor[id] == nullptr) return;
    motor[id]->brake();
}

void motor_bridge_set_dead_zone(uint8_t id, float rpm)
{
    if (motor[id] == nullptr) return;
    motor[id]->set_dead_zone(rpm);
}
