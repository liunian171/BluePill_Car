/**
 * ============================================================================
 *  Motor 驱动 — 功能层 + 协议接口层（C++）
 * ============================================================================
 *
 *  架构定位（四层模型）：
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │  main.c（应用层）→ motor_bridge_set_speed_rpm(0, 100)        │
 *  │  语言：人的意图                                                │
 *  └──────────────────────┬───────────────────────────────────────┘
 *                         │
 *  ┌──────────────────────▼───────────────────────────────────────┐
 *  │  motor_bridge.h/.cpp（C 桥接层）                              │
 *  │  extern "C" 接口，C→C++ 适配                                  │
 *  └──────────────────────┬───────────────────────────────────────┘
 *                         │
 *  ┌──────────────────────▼───────────────────────────────────────┐
 *  │  本文件 — Motor 类（功能层）                                   │
 *  │  职责：速度限制、死区判断、RPM→千分比换算                      │
 *  │  语言：业务抽象（RPM / m/s）                                   │
 *  │  持有：IMotorProtocol&（接口注入，不依赖具体驱动芯片）          │
 *  └──────────────────────┬───────────────────────────────────────┘
 *                         │
 *  ┌──────────────────────▼───────────────────────────────────────┐
 *  │  IMotorProtocol（协议接口）→ motor_protocol.h 中实现          │
 *  │  TB6612Protocol: 千分比 → 真值表 → GPIO + PWM                │
 *  │  将来可扩展: L298NProtocol, DRV8833Protocol 等               │
 *  └──────────────────────┬───────────────────────────────────────┘
 *                         │
 *  ┌──────────────────────▼───────────────────────────────────────┐
 *  │  PWM 驱动层 + GPIO 驱动层                                      │
 *  │  语言：寄存器                                                  │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  ▸ 设计关键点 ◂
 *  1. IMotorProtocol 必须定义在 Motor 前面（C++ 前向引用规则）：
 *     否则 Motor 中引用 IMotorProtocol 时类型未知，clangd 会
 *     从"Unknown type name"开始产生级联误报，把后续 class 也标红。
 *  2. 协议层只实现接口，不是 TB6612 的全部驱动：
 *     TB6612Protocol 的职责是把功能层的千分比"翻译"成 TB6612
 *     需要的操作序列，不管芯片还能用来驱动步进电机或电磁阀。
 *     这遵循"层按翻译意图分，不按硬件相似性分"的原则。
 *  3. 本文件是 C++ class 声明，不要用 extern "C" 包裹：
 *     extern "C" 只用于桥接层（motor_bridge.h）的 C 接口函数。
 *
 *  ▸ RPM vs 线速度决策 ◂
 *  - Motor 类提供 set_speed_rpm() 作为主接口，RPM 是电机的固有语言。
 *  - 同时提供 set_speed_mps()，需构造时注入 wheel_radius_m。
 *  - RPM 线速度放在 Motor 层而非更上层，因为 Motor 就是驱动链的顶层。
 *  - 换轮子只需改构造参数，不影响协议层和桥接层。
 *
 *  ▸ 开环下 RPM 的准确性 ◂
 *  - 无编码器时，RPM 只是"名义值"，实际速度取决于负载/电压。
 *  - max_rpm_ 是校准点：用户实测 100% 占空比下的空载转速填进来。
 *  - 二期接入编码器 + PID 闭环后，RPM 变成真实值，接口不变。
 *
 *  ▸ 死区 ◂
 *  - 直流电机占空比低于 5%~15% 时可能不转（启动力矩不够）。
 *  - dead_zone_rpm_ 用 RPM 单位而非千分比，语义更清晰。
 *
 * ============================================================================
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <cstdint>

// ========== 协议抽象接口（必须先于 Motor 定义）==========

class IMotorProtocol
{
public:
    /**
     * 设置速度比率（千分比）
     * @param  rate_0E3  千分比，-1000~1000，正=正转，负=反转，0=停止
     *                    正值越大越快，负值越小反方向越快
     *                    绝对值直接对应占空比：500=50%，1000=100%
     */
    virtual void set_speed_rate_0E3(int16_t rate_0E3) = 0;

    /** 惯性停止：IN1=IN2=0，PWM=0，电机自由滑行 */
    virtual void stop() = 0;

    /** 刹车制动：IN1=IN2=1，短接制动，电机快速停转 */
    virtual void brake() = 0;

    virtual ~IMotorProtocol() = default;
};

// ========== 功能层：Motor 类 ==========

class Motor
{
public:
    // ---- 依赖注入：只依赖接口，不依赖具体芯片 ----
    IMotorProtocol &protocol;

    /**
     * 额定转速（RPM）。
     * 开环下为实测值：用户给 100% 占空比时测到的空载 RPM。
     * 闭环下直接使用编码器测得的真实 RPM。
     * 构造后可通过 set_max_rpm() 运行时校准。
     */
    float max_rpm;

    /**
     * 轮子半径（mm），0 = 不启用线速度模式。
     * set_speed_mps() 中通过 v=ω·r → RPM = v*60/(2πr) 换算。
     * 换轮子只需改此值，Motor/Protocol/Bridge 全不动。
     */
    float wheel_radius_mm;

    /** 死区（RPM），绝对速度低于此值时自动 stop，避免占空比太小电机不转 */
    float dead_zone_rpm;

    /**
     * 构造
     * @param protocol       协议层引用（TB6612MotorProtocol / L298NProtocol 等）
     * @param max_rpm        额定转速（默认 200 RPM，建议实测后校准）
     * @param wheel_radius_mm 轮子半径（mm），0 = 不启用线速度模式
     */
    Motor(IMotorProtocol &protocol, float max_rpm = 200.0f, float wheel_radius_mm = 33.1f);

    /**
     * 设置转速（RPM 数值模式）
     * 语义：设置绝对值，不是比例。300 表示目标 300 RPM。
     * 内部做：死区判断 → 钳位到 ±max_rpm → 线性换算千分比 → driver.set_speed()
     */
    void set_speed_rpm(float rpm);

    /**
     * 设置线速度（m/s 模式，需构造时注入 wheel_radius_m > 0）
     * 语义：设置地面线速度。0.5 表示轮子切线速度 0.5 m/s。
     * 内部做：m/s → RPM → 千分比，物理公式：v = ω·r
     */
    void set_speed_mps(float mps);

    /** 停止（透传 IMotorProtocol::stop） */
    void stop();

    /** 刹车（透传 IMotorProtocol::brake） */
    void brake();

    /** 运行时设置死区，单位 RPM */
    void set_dead_zone(float rpm);

    ~Motor() = default;
};

#endif
