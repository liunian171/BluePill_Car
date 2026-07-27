/**
 * ============================================================================
 *  Motor 协议实现层 — TB6612Protocol（C++）
 * ============================================================================
 *
 *  职责：把功能层的千分比"翻译"成 TB6612FNG 需要的硬件信号序列。
 *
 *  ▸ 为什么叫 TB6612Protocol 而不是 TB6612MotorProtocol ◂
 *  这个类实现的是 IMotorProtocol 接口。TB6612 芯片本身还能用在
 *  步进电机/电磁阀等场景——但那些场景需要不同的接口，会建新的类
 *（如 TB6612StepProtocol）。命名 Pattern = 芯片名 + 接口角色。
 *  遵循"层按翻译意图分，不按硬件相似性分"的原则。
 *
 *  ▸ 注入的对象 ◂
 *  构造函数注入 3 样东西（不是 2 个"对象"）：
 *    1. PWM_Handle —— 控制速度（占空比）
 *    2. UserGPIO_Handle —— IN1 方向引脚
 *    3. UserGPIO_Handle —— IN2 方向引脚
 *  每个 pin 独立一个 handle，即使 IN1/IN2 同端口也不用 pin_mask。
 *  原因：跨端口不可合并（如 IN1=GPIOA_PIN6, IN2=GPIOB_PIN3），
 *  且真值表需要独立写两个 pin（正转 IN1=1 IN2=0、反转 IN1=0 IN2=1）。
 *
 *  ▸ TB6612FNG 控制真值表 ◂
 *  ┌─────┬─────┬──────────┬──────────────┐
 *  │ IN1 │ IN2 │   PWM    │    状态      │
 *  ├─────┼─────┼──────────┼──────────────┤
 *  │  0  │  0  │    X     │ 惯性停止     │
 *  │  1  │  0  │ 0~100%   │ 正转（CW）   │
 *  │  0  │  1  │ 0~100%   │ 反转（CCW）  │
 *  │  1  │  1  │    X     │ 刹车制动     │
 *  └─────┴─────┴──────────┴──────────────┘
 *
 *  ▸ GPIO 跨平台抽象 ◂
 *  UserGPIO_Handle 内部存 void *port + pin + ops 表。
 *  TB6612Protocol 只通过 gpio_handle->ops->write() 操作，不直接调
 *  HAL_GPIO_WritePin()。换 MCU 时提供新的 GPIO ops 即可，本类零修改。
 *  — 和 PWM_Handle 的 void *htim + PWM_PlatformOps_t 模式一致。
 *
 *  ▸ GPIO 初始化 ◂
 *  GPIO 的模式配置（推挽输出/上下拉/速度）由各平台的 MX_GPIO_Init()
 *  在 main.c 初始化阶段完成，不在 ops 中。ops 只管运行时 set/reset/write。
 *  和 PWM 的 MX_TIMx_Init() vs pwm_ops->start() 模式一致。
 *
 *  ▸ 注入方式：引用 vs 指针 ◂
 *  协议层持有的是具体类型的引用/句柄（PWM_Handle&, UserGPIO_Handle&）。
 *  功能层持有的是抽象接口引用（IMotorProtocol&），通过虚函数派发
 *  到子类实现。这是"依赖倒置"的核心：上层不知下层具体是谁。
 *
 * ============================================================================
 */

#ifndef MOTOR_PROTOCOL_H_
#define MOTOR_PROTOCOL_H_

#include "motor.h"       // 需要 IMotorProtocol 的完整定义
#include "pwm.h"         // 需要 PWM_Handle
#include "usergpio.h"    // 需要 UserGPIO_Handle

// [代码注意] 类名只叫 TB6612Protocol，不叫 TB6612MotorProtocol。
// 它实现的是 IMotorProtocol，不是"TB6612 的全部驱动"。

class TB6612MotorProtocol : public IMotorProtocol
{
public:
    // ---- 硬件句柄（构造时从 Bridge 层注入）----
    //一个TB6612MotorProtocol对应一个TB6612FNG的其中一个通道,以motor为单位
    PWM_Handle*    hpwm;           // PWM 通道句柄（控制速度）
    UserGPIO_Handle* hain1_gpio;   // IN1 方向引脚
    UserGPIO_Handle* hain2_gpio;   // IN1 方向引脚
    UserGPIO_Handle* hstby_gpio;   // STBY 待机引脚（可选，两路共用）

    /**
     * 构造 — 从 Bridge 层注入所有硬件句柄
     * @param pwm_handle      定时器 PWM 通道句柄
     * @param ain1_gpio       IN1 方向引脚
     * @param ain2_gpio       IN2 方向引脚
     * @param stby_gpio       STBY 引脚
     */
    TB6612MotorProtocol(PWM_Handle *pwm_handle,
                        UserGPIO_Handle *ain1_gpio,
                        UserGPIO_Handle *ain2_gpio,
                        UserGPIO_Handle *stby_gpio);

    ~TB6612MotorProtocol();

    /**
     * 设置速度（实现 IMotorProtocol 接口）
     * 翻译过程：千分比符号 → 查真值表 → 写 GPIO 方向 + 设 PWM 占空比
     *   正 speed → IN1=1, IN2=0 → 正向 PWM
     *   负 speed → IN1=0, IN2=1 → 反向 PWM
     *   0 speed  → IN1=IN2=0     → 惯性停止
     *
     * [代码注意] 参数是 uint16_t，但取绝对值时需要用到 int16_t 的符号信息。
     * 建议接口改为 int16_t speed_0E3（±1000），和 IMotorProtocol 一致。
     */
    void set_speed_rate_0E3(int16_t rate_0E3) override;  //正反转转速比

    /** 惯性停止（IN1=IN2=0, PWM=0）*/
    void stop() override;

    /** 刹车制动（IN1=IN2=1，短接制动）*/
    void brake() override;
};

#endif
