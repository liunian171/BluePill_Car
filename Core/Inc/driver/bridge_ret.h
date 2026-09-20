#ifndef BRIDGE_RET_H
#define BRIDGE_RET_H

/**
 * ============================================================================
 *  桥接层家族 — 统一返回码（C1 家族契约，2026-09-13）
 * ============================================================================
 *
 *  层位：桥接层是 C 世界 ↔ C++ 对象的**门面**，函数体只做三件事：
 *        查 id → 校验参数 → 转发。零 I/O、零业务逻辑、零 HAL。
 *
 *  ▸ 家族契约（正文见 doc/桥接层家族契约.md）◂
 *    1) 形状     init(id, const xxx_cfg_t*) → bridge_ret_t；运行时同前缀族名
 *    2) 错误通道 **返回码**（不做串口/OLED 输出 → 桥保持零依赖，PC 桩可编译）
 *    3) 动作类   init / set_xxx / start / stop 等 → 返回 bridge_ret_t
 *    4) 取值类   get_xxx / ready 等 → 保持值返回；越界/未初始化必须返回
 *                **头文件注明的安全默认值**（不得返回未定义值）
 *    5) 内存     静态池 + placement new；容量用 XXX_MAX 宏；禁堆 new
 *    6) 校验     init 与运行时都必须查 id；handle 必须查 NULL；
 *                未初始化调用返回 ERR_NOT_INIT，不得无声 return
 *    7) 知识归属 方向/极性/标定等"硬件与整车知识"必须**配置注入**，
 *                禁止在桥内用 id 做特例判断（如 `if (id == 1) rpm = -rpm;`）
 *    8) 单例豁免 仅 oled_bridge（单屏是硬事实）；运行时接口不带 id，
 *                但 init 仍用配置结构 + 返回码，形状与家族一致
 *
 *  ▸ 与模块自有返回码的关系 ◂
 *    steering_ret_t / speed_loop_ret_t 等**组件**返回码保持模块自有，
 *    但语义与取值顺序对齐本表（OK=0 优先，其后为错误）。
 *    桥家族共用本表（family 内一致），因为四座桥是同一层、同一职责。
 *
 *  ▸ 消费约定 ◂
 *    · init/配置类：组装层必须判断（失败 → Error_Handler，先例：steering_init）
 *    · 运行时类：调用方按需判断；50ms 热路径可显式 `(void)` 忽略（**有意的忽略**
 *      必须写成 `(void)`，不要靠"返回值没人接"来表达）
 * ============================================================================
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BRIDGE_OK           = 0,   /* 成功 */
    BRIDGE_ERR_NOT_INIT = 1,   /* 该 id 未初始化（init 未调用或未通过） */
    BRIDGE_ERR_BAD_ARG  = 2,   /* 入参非法：id 越界 / handle 为 NULL / 值为非有限数 */
    BRIDGE_ERR_BAD_CFG  = 3,   /* 配置非法：协议/类型枚举不支持、标定值区间不成立 */
    BRIDGE_ERR_IO       = 4,   /* 下层器件/总线返回错误（器件层已报告，桥只做透传） */
    BRIDGE_ERR_BUSY     = 5,   /* 资源占用（如阶跃测试激活期; 2026-09-20 app_control 引入） */
} bridge_ret_t;

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_RET_H */
