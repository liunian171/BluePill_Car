/**
 * ============================================================================
 *  app_wiring.h — 装配模块（组装层的"唯一装配点"）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主的装配点。⚠️ 全工程唯一允许 include car_config.h
 *        的 app 模块（其余 app_* 与组件/桥接层一律禁止, §4.5）。
 *  职责: ① 把 car_config.h 的 20 个标定宏翻译成各 cfg 结构体初值
 *        ② 填好全部 io 函数指针表（判活 HAL 读绑定也在此）
 *        ③ 按依赖序调用全部组件/桥/app init —— 任一失败 → 返回错误码
 *           （装载层收到后 Error_Handler,"init 失败即停"语义不变, 调试总结 §22）
 *  换平台: 不动; 换整车: 只改 car_config.h。
 * ============================================================================
 */

#ifndef APP_WIRING_H
#define APP_WIRING_H

#include <stdint.h>
#include "driver/bridge_ret.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 一次性装配: cfg 表初始化 + io 表填充 + init 链（顺序 = 现 main.c 装载顺序:
 * ringbuf → 编码器 → PWM → 电机桥 → 速度环 → 舵机桥+转向 → OLED → 巡线(stub) →
 * IMU → odom → 仲裁 → app_link/control/display/tx）。
 * 返回 OK 之外任何码: 装载层必须进 Error_Handler, 不得带病运行。 */
bridge_ret_t app_wiring_load(void);

/* init 失败定位（Error_Handler 前可查; 未失败返回 ""） */
const char *app_wiring_failed_module(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_WIRING_H */
