#ifndef SERVO_H
#define SERVO_H

/**
 * ============================================================================
 *  执行对象层 — Servo（舵机语义翻译）
 * ============================================================================
 *
 *  层位（自顶向下，详见 doc/代码风格与模块衔接指南 §1.2）：
 *    组装层        main.c              实例创建 / 命令转发 / 接管权仲裁
 *    执行组件      steering            意图(输出域角) → 器件级目标(物理角)
 *    执行对象层    Servo               ← 本文件：物理角 → 千分比
 *    器件层        PWMServoProtocol     千分比 → 脉宽 µs
 *    驱动策略层    pwm_set_pulse_us     脉宽 → CCR
 *    驱动平台层    pwm_platform_ops     寄存器写（唯一碰 HAL）
 *
 *  本层职责（两项）：
 *    1) 物理角(°) → 千分比(0E3) 换算        — 器件级目标 → 器件语言
 *    2) 器件行程极限钳位（默认 0~270°）      — 器件知识，永久兜底、任何路径绕不过
 *
 *  ▸ 单位与坐标域声明（规范 §4.2 义务 8）◂
 *    本层接口角度 = **物理角**，区间 [min_angle, max_angle] = [0°, 270°]
 *    （500µs = 0°，1500µs = 135° 电气中位，2500µs = 270°）
 *    ⚠️ 与命令域/组件域（**输出域绝对角**，直行 = -90）**不同域**：
 *       换算（+135° 安装偏置）由执行组件 steering 负责 —— 安装偏置是"车知识"，
 *       不进本层，否则换车要改执行对象层、"换整车零修改"承诺作废。
 *
 *  依赖白名单：IServoProtocol（构造注入引用）
 *  禁止：直调 HAL / 直调 pwm / 依赖具体协议类实现
 *  可移植性：换 MCU 平台 —— 不动本文件；换舵机器件 —— 不动本文件
 *            （新增一个 IServoProtocol 派生类即可；行程参数经构造注入）
 * ============================================================================
 */

#include <stdint.h>

/* 协议抽象父类（器件层对上暴露的接缝，非独立层） */
class IServoProtocol
{
public:
    /* 设置输出：千分比占空比（0~1000；0 = min_pulse 端） */
    virtual void set_position(uint16_t rate_0E3) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual ~IServoProtocol() = default;
};

/* 执行对象：一个实例控制一个舵机 */
class Servo
{
public:
    /* min_angle / max_angle：器件行程(°)，默认本项目 270° 舵机；
     * 配置非法（max_angle <= min_angle）时 set_angle 拒绝输出（不写 CCR） */
    Servo(IServoProtocol& proto, float min_angle = 0.0f, float max_angle = 270.0f);
    ~Servo();

    void  set_angle(float angle);   /* 物理角(°) → 器件行程钳位 → 千分比 → 协议层 */
    float get_angle() const;        /* 最近一次实际输出角(°)，钳位后 */
    void  start();
    void  stop();

private:
    float min_angle;
    float max_angle;
    float cur_angle;
    IServoProtocol& protocol;
};

#endif
