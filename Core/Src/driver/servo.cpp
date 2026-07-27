/**
 * ============================================================================
 *  伺服驱动 — 功能层（C++）
 * ============================================================================
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │  架构总览                                                       │
 *  │                                                                 │
 *  │  app / main.cpp                                                 │
 *  │      │                                                          │
 *  │      ▼                                                          │
 *  │  ┌─ servo_core.cpp/.h ─────────────────────────────────────┐   │
 *  │  │  功能层（C++ 类）                                         │   │
 *  │  │  Servo 类                                                 │   │
 *  │  │  职责：角度换算、限位钳位、斜坡平滑、耗时控制               │   │
 *  │  │  调用下层：protocol->setPosition(id, pulse)               │   │
 *  │  └──────────────────┬───────────────────────────────────────┘   │
 *  │                     │                                           │
 *  │                     ▼                                           │
 *  │  ┌─ servo_protocol.cpp/.h ─────────────────────────────────┐   │
 *  │  │  适配层（C++ 虚基类 + 派生类）                              │   │
 *  │  │  IServoProtocol 虚基类（定义接口契约）                      │   │
 *  │  │  ├─ PWMServoProtocol  → 调 pwm_set_duty_0E3              │   │
 *  │  │  ├─ ServoUART   → 调 HAL_UART_Transmit                   │   │
 *  │  │  └─ ServoI2C    → 调 HAL_I2C_Mem_Write                   │   │
 *  │  └─────────────────────────────────────────────────────────┘   │
 *  │                     │                                           │
 *  │                     ▼                                           │
 *  │  ┌─ pwm.c / stm32f4xx_hal_uart.h / ... ────────────────────┐  │
 *  │  │  驱动底层（C，前提已实现）                                │  │
 *  │  │  pwm.c 的 pwm_set_duty_0E3 / pwm_set_freq               │  │
 *  │  └─────────────────────────────────────────────────────────┘  │
 *  └─────────────────────────────────────────────────────────────────┘
 *
 *  设计原则：
 *    - 功能层不知道适配层有几个派生类，只对着 IServoProtocol* 调用
 *    - 加一个新协议（如 CAN 舵机）只需要新增一个派生类，功能层零修改
 * ============================================================================
 */



/*==============================================================================
 *  待实现：适配层（servo_protocol.h / .cpp）
 *
 *  IServoProtocol 虚基类
 *  ─────────────────
 *  定义所有协议的共同接口，派生类各自实现。
 *
 *  class IServoProtocol {
 *  public:
 *      // 设置脉宽（us 级别）
 *      virtual void setPosition(uint8_t id, uint16_t pulse_us) = 0;
 *
 *      // 读取脉宽（带反馈舵机使用，PWM 舵机可返回 0）
 *      virtual uint16_t getPosition(uint8_t id) { return 0; }
 *
 *      // 虚析构（确保派生类正确析构）
 *      virtual ~IServoProtocol() = default;
 *  };
 *
 *  ServoPWM（派生类）
 *  ─────────
 *  实现 IServoProtocol，把脉宽映射到 PWM 驱动的 duty。
 *  构造时传入 PWM_Handle* 指针。
 *
 *  思路：
 *    - setPosition(id, pulse_us):
 *          1. 脉宽 us → 千分比 duty = pulse_us * 1000 / 20000
 *          2. 调 pwm_set_duty_0E3(hpwm, duty)
 *
 *  ServoUART（派生类，预留）
 *  ──────────
 *  实现 IServoProtocol，把脉宽通过串口协议发送给舵机。
 *  构造时传入 UART_HandleTypeDef* 和协议类型（Dynamixel / S.Bus / 自定义）。
 *
 *  ServoI2C（派生类，预留）
 *  ─────────
 *  实现 IServoProtocol，把脉宽通过 I²C 写入 PWM 驱动器（如 PCA9685）。
 *==============================================================================*/



/*==============================================================================
 *  待实现：功能层（servo_core.h / .cpp）
 *
 *  Servo 类
 *  ─────────
 *  面向应用层的主要接口。
 *
 *  构造参数：
 *    - IServoProtocol *proto   ← 协议适配层指针（构造时注入）
 *    - uint8_t id               ← 总线上的舵机 ID / 通道号
 *    - uint16_t min_pulse_us    ← 0° 对应的脉宽 (us)
 *    - uint16_t max_pulse_us    ← 180° 对应的脉宽 (us)
 *
 *  对外 API（按实现优先级排列）：
 *
 *  【P0 — 核心功能】
 *  void setAngle(uint16_t deg);
 *      - 立即设置舵机角度（无延时）
 *      - 内部做角度钳位（0~180）
 *      - 实现：
 *          1. pulse = min_pulse + deg * (max_pulse - min_pulse) / 180
 *          2. proto->setPosition(id, pulse)
 *
 *  uint16_t getAngle();
 *      - 读取当前角度（返回缓存值）
 *
 *  void setAngleLimit(uint16_t min, uint16_t max);
 *      - 设置软件限位（默认 0~180）
 *
 *  【P1 — 耗时控制】
 *  void setAngleWithDuration(uint16_t deg, uint16_t duration_ms);
 *      - 在指定毫秒内从当前角度平滑转到目标角度
 *      - 实现思路：内部启动一个定时分片状态机
 *          1. 计算角度差值
 *          2. 以某个时间间隔（如 PWM 周期 20ms）为步长
 *          3. 每步匀插一个中间角度，调一次 setAngle
 *          4. 步进到目标后停止
 *
 *  【P2 — 进阶功能（可选）】
 *  void setAngleSync(uint16_t deg, uint16_t duration_ms, uint32_t trigger_time);
 *      - 多舵机同步：先缓存目标，统一时间戳触发执行
 *
 *  void setDeadBand(uint16_t deg);
 *      - 设置死区补偿（默认 0），小角度变化在死区内忽略
 *
 *  ErrorState getErrorState();
 *      - 获取当前错误状态（过载 / 限位触发等）
 *
 *  void powerOff();
 *      - 紧急停止（停止 PWM 输出）
 *
 *  内部状态：
 *    - 当前角度缓存
 *    - 目标角度缓存（耗时控制时用）
 *    - 限位范围
 *    - 死区阈值
 *    - 错误状态标志
 *==============================================================================*/



/*==============================================================================
 *  设计备忘录
 *
 *  1. ISO C++ 与 HAL 混编
 *     servo_core.cpp 和 servo_protocol.cpp 都编译为 C++，
 *     pwm.c / pwm.h 保持 C。extern "C" 由 pwm.h 自己处理
 *     （检查 pwm.h 是否有 __cplusplus 守卫，若无则需在 servo_protocol.cpp 中
 *      用 extern "C" {} 包裹 #include "pwm.h"）。
 *
 *  2. 斜坡状态的定时触发方式（setAngleWithDuration 的关键决策）
 *     不推荐在主循环里死等 HAL_Delay。可选方案：
 *       a) SysTick 中断（最简单）：每 1ms 标记一次，Servo 内部轮询判断
 *       b) 硬件定时器中断：单独启动一个 TIM，中断里步进
 *       c) 裸机主循环非阻塞：Servo 提供 tick() 函数，主循环每次调用
 *      建议先用 (c)，最灵活且不依赖中断。
 *
 *  3. 文件名规划
 *     servo.h            → 当前文件，放 IServoProtocol 和 Servo 的声明
 *     servo.cpp          → 当前文件，放 Servo 功能层实现
 *     servo_protocol.cpp → 新建，放 ServoPWM 等派生类实现
 *
 *  4. 实现步骤建议
 *     ① 先写 IServoProtocol 基类 + ServoPWM 派生类 + Servo 类（setAngle 和 getAngle）
 *     ② main.cpp 里接入现有的 pwm_tim5_ch4，验证舵机能转
 *     ③ 加限位保护
 *     ④ 加 setAngleWithDuration
 *     ⑤ 后续再考虑同步 / 死区补偿等
 *==============================================================================*/
#include "tool.h"
#include "servo.h"
Servo::Servo(IServoProtocol& proto)
   :min_angle(0),
    max_angle(180),
    protocol(proto)
    {}

void Servo::set_angle(float angle)
{ 
    uint16_t rate_0E3=(angle-min_angle)*1000/(max_angle-min_angle);
    protocol.set_position(rate_0E3);   
}

void Servo::start()
{
    protocol.start();
}

void Servo::stop()
{
    protocol.stop();
}

Servo::~Servo()
{
}
