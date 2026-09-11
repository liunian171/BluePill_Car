/**
 * ============================================================================
 *  执行对象层 — Servo（舵机语义翻译，C++）
 * ============================================================================
 *
 *  层位（自顶向下，详见 doc/代码风格与模块衔接指南 §1.2）：
 *    组装层        main.c             实例创建 / 命令转发 / 接管权仲裁
 *    执行组件      steering           意图(输出域角) → 器件级目标(物理角)
 *    执行对象层    Servo              ← 本文件：物理角 → 千分比
 *    器件层        PWMServoProtocol    千分比 → 脉宽 µs
 *    驱动策略层    pwm_set_pulse_us    脉宽 → CCR（1µs 分辨率）
 *    驱动平台层    pwm_platform_ops    寄存器写（唯一碰 HAL）
 *
 *  本文件职责：物理角 → 千分比换算 + 器件行程极限钳位（单一职责，两项）。
 *  依赖白名单：IServoProtocol（构造注入引用）
 *  禁止：直调 HAL / 直调 pwm / 依赖具体协议类实现
 *
 *  可移植性：换 MCU 平台 —— 不动本文件；换舵机器件 —— 不动本文件
 *            （行程参数经构造注入，新增 IServoProtocol 派生类即可）
 *
 *  ▸ 钳位为什么在本层 ◂
 *    0~270° 是**舵机自身**的物理行程（手册数据），换车/换机构/改标定都不变
 *    → 属"器件知识"，且 set_angle 是通往器件的必经入口，放这里任何调用方都绕不过。
 *    对比：本车可用行程 [-115,-65]（输出域）属"车知识"，归执行组件 steering。
 *
 *  ▸ 历史 ◂
 *    本文件头部原为上游驱动工程（A_board_pwm_driver_test，STM32F427）的设计草案
 *    （servo_core / ServoUART / ServoI2C / PCA9685 等，均未实现），随移植整文件复制；
 *    2026-09-11 剥离至 doc/舵机旧设计稿_归档_上游A板.md 保留备查。
 *    同日补：器件行程极限钳位、get_angle()、行程参数构造注入（原为 public 裸字段）。
 * ============================================================================
 */

#include "servo.h"

Servo::Servo(IServoProtocol& proto, float min_angle, float max_angle)
   : min_angle(min_angle),
     max_angle(max_angle),
     cur_angle(min_angle),
     protocol(proto)
{
}

void Servo::set_angle(float angle)
{
    /* 配置合法性：非法行程直接拒绝输出（不写 CCR），避免除零与语义错乱 */
    if (max_angle <= min_angle) return;

    /* 器件行程极限钳位（本层兜底：任何调用路径都绕不过） */
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;

    cur_angle = angle;

    /* 物理角 → 千分比：rate = (angle - min) / 行程 × 1000
     * 钳位后 rate 必然落在 [0,1000]，不存在越界千分比 */
    uint16_t rate_0E3 = (uint16_t)((angle - min_angle) * 1000.0f / (max_angle - min_angle));

    protocol.set_position(rate_0E3);
}

float Servo::get_angle() const
{
    return cur_angle;
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
