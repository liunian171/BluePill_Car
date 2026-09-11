/**
 * ============================================================================
 *  执行对象层 — Servo（舵机语义翻译，C++）
 * ============================================================================
 *
 *  层位（自顶向下，详见 doc/代码风格与模块衔接指南 §1.2）：
 *    组装层        main.c             实例创建 / 命令转发 / 接管权仲裁
 *    执行组件      steering（待建）   钳位 + 域换算 + 标定限位      ← 见 S2/S3/S4
 *    执行对象层    Servo              ← 本文件：角度 → 千分比（器件语言）
 *    器件层        PWMServoProtocol    千分比 → 脉宽 µs
 *    驱动策略层    pwm_set_pulse_us    脉宽 → CCR（1µs 分辨率）
 *    驱动平台层    pwm_platform_ops    寄存器写（唯一碰 HAL）
 *
 *  本文件职责：角度域 → 千分比换算（单一职责）。
 *  依赖白名单：IServoProtocol（构造注入引用）、tool.h（map）。
 *  禁止：直调 HAL / 直调 pwm / 依赖具体协议类实现。
 *
 *  可移植性：换 MCU 平台 —— 不动本文件；换舵机器件 —— 不动本文件
 *            （协议变化只需新增一个 IServoProtocol 派生类）。
 *
 *  ⚠️ 已知偏差（待治理，编号见 doc/舵机代码结构对照分析.md）：
 *    S2  钳位职责缺失——本层定位应含"钳位"，当前无 clamp，
 *        实际 clamp 重复出现在 main.c 的 servo_set/servo_apply
 *    S3  坐标域未声明——本层按物理角 [min_angle, max_angle] 语义；
 *        与命令域（输出域绝对角，直行 = -90）的换算（+135）硬编码在 main.c，
 *        待域契约确定后归位
 *    S10 min_angle / max_angle 为 public 字段，无注入入口
 *        （应改为构造参数或配置注入）
 *
 *  历史：本文件头部原为上游驱动工程（A_board_pwm_driver_test，STM32F427）
 *        的设计草案（servo_core / ServoUART / ServoI2C / PCA9685 等，
 *        均未实现），随移植整文件复制；2026-09-11 剥离至
 *        doc/舵机旧设计稿_归档_上游A板.md 保留备查。
 * ============================================================================
 */
#include "tool.h"
#include "servo.h"
Servo::Servo(IServoProtocol& proto)
   :min_angle(0),
    max_angle(270),   /* 本项目舵机: 270° 行程 (500~2500µs), 0°=500µs, 135°=1500µs 中位 */
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
