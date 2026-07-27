/**
 * ============================================================================
 *  UserGPIO 驱动 — 抽象层（跨平台 GPIO 封装）
 * ============================================================================
 *
 *  架构对标 pwm.h，采用 ops 表模式实现跨平台 GPIO 操作：
 *
 *  ┌─ usergpio.c ────────────────────────────────────────────────┐
 *  │  策略层（跨平台通用）                                         │
 *  │  职责：从 Handle 提取 port + pin，传给 ops                    │
 *  └──────────────────────┬──────────────────────────────────────┘
 *                         │ ops->xxx(void *port, uint16_t pin, ...)
 *                         ▼
 *  ┌─ usergpio_platform.c ───────────────────────────────────────┐
 *  │  平台实现层（每 MCU 一套）                                    │
 *  │  职责：接收裸参数，直接调 HAL 库操作寄存器                    │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  ▸ 一个 handle 管一个 pin ◂
 *  不用 port + pin_mask 结构同时管多个 pin。原因：
 *    1. IN1/IN2 可能跨端口，mask 模型直接失效
 *    2. 真值表场景下两个 pin 需要独立写（IN1=1 IN2=0）
 *    3. 一个 pin 一个 handle 语义最清晰，分组由上层协议类管理
 *
 *  ▸ GPIO 初始化不入 ops ◂
 *  各平台的 MX_GPIO_Init() 一次性配置模式/上下拉/速度。
 *  ops 只管运行时：set/reset/write/toggle/read。
 *  对标 PWM：MX_TIMx_Init() 初始化，pwm_ops 管运行。
 *
 * ============================================================================
 */

#ifndef USERGPIO_H
#define USERGPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ========== 平台操作表（对标 PWM_PlatformOps_t）==========

typedef struct UserGPIO_PlatformOps_t
{
    void (*write)(void *hgpio_port, uint16_t gpio_pin, uint8_t state);
    void (*toggle)(void *hgpio_port, uint16_t gpio_pin);
    uint8_t (*read)(void *hgpio_port, uint16_t gpio_pin);

} UserGPIO_PlatformOps_t;

// ========== GPIO 句柄（对标 PWM_Handle）==========

typedef struct
{
    void *hgpio_port;                     // STM32: GPIO_TypeDef*
    uint16_t gpio_pin;                    // 引脚掩码（如 GPIO_PIN_13 = 0x2000）
    const UserGPIO_PlatformOps_t *ops;    // 平台操作表指针

} UserGPIO_Handle;

// ========== 策略层函数（对标 pwm_set_duty_0E3）==========

void usergpio_write(UserGPIO_Handle *hGPIO, uint8_t state);
void usergpio_toggle(UserGPIO_Handle *hGPIO);
uint8_t usergpio_read(UserGPIO_Handle *hGPIO);

#ifdef __cplusplus
}
#endif

#endif
