#include "motor.h"
#include "tool.h"

/**
 * Motor 功能层实现
 * RPM/线速度 → 千分比 → IMotorProtocol
 */

// ========== 构造函数 ==========
// dead_zone 默认取 max_rpm 的 5%

Motor::Motor(IMotorProtocol &protocol, float max_rpm, float wheel_radius_mm)
    :protocol(protocol), max_rpm(max_rpm), wheel_radius_mm(wheel_radius_mm),
     dead_zone_rpm(max_rpm * 0.05f) {}


// ========== set_speed_rpm ==========
// 死区判断 → map(rpm, ±max_rpm, ±1000) → 输出千分比

void Motor::set_speed_rpm(float rpm)
{
    int16_t rpm_abs = rpm > 0 ? rpm : -rpm;
    if (rpm_abs < dead_zone_rpm) {
        protocol.stop();
        return;
    }
    int16_t rate_0E3 = map(rpm, -max_rpm, max_rpm, -1000, 1000);
    protocol.set_speed_rate_0E3(rate_0E3);
}


// ========== set_speed_mps ==========
// m/s → RPM → 千分比
// RPM = (v × 1000 / r_mm) × 60 / (2π)
// rate_0E3 = RPM / max_rpm × 1000

void Motor::set_speed_mps(float mps)
{
    int16_t rate_0E3 = ((mps * 60 * 1000) * 1000 / wheel_radius_mm)
                       / (2 * 3.14159f) / max_rpm;
    protocol.set_speed_rate_0E3(rate_0E3);
}


// ========== stop / brake ==========

void Motor::stop()
{
    protocol.stop();
}

void Motor::brake()
{
    protocol.brake();
}


// ========== set_dead_zone ==========
// 直流有刷电机启动死区 ≈ 5%~15% 占空比

void Motor::set_dead_zone(float rpm)
{
    dead_zone_rpm = rpm;
}
