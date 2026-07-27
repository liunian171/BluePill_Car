#ifndef __SERVO_H
#define __SERVO_H
#include <stdint.h>
//协议层抽象父类
class IServoProtocol
{
public:
    
    //通用的函数
    virtual void set_position(uint16_t rate_0E3) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual ~IServoProtocol() = default;
};
//功能类,控制一个舵机,处理角度
class Servo
{
    public:
    //通用的参数
    uint8_t min_angle;//角度范围
    uint8_t max_angle;
    IServoProtocol& protocol;   //>>>>>>>>>>>>>>>>引用的父抽象类,但是可以注入子具体类,调用的是父抽象类声明子具体类实现的方法
    Servo(IServoProtocol& proto);
    ~Servo();
    //功能函数
    void set_angle(float angle);
    void start();
    void stop();

};



#endif