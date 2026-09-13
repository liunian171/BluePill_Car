#ifndef SERVO_BRIDGE_H
#define SERVO_BRIDGE_H

/**
 * ============================================================================
 *  桥接层 — servo_bridge（舵机 C 门面）
 * ============================================================================
 *
 *  层位：组装层 → 执行组件(steering) → **本文件** → 执行对象层(Servo) → 器件层
 *  职责：查 id → 校验 → 转发。零业务逻辑、零 I/O、零 HAL。
 *
 *  ▸ 家族契约（正文 doc/桥接层家族契约.md，返回码 bridge_ret.h）◂
 *    · init 用配置结构注入：`servo_bridge_init(id, const servo_bridge_cfg_t*)`
 *    · 动作类返回 bridge_ret_t；id / handle / 协议枚举全部校验，不静默失败
 *    · 静态池 + placement new，禁堆 new
 *
 *  ▸ 单位与坐标域 ▸ 角度 = **物理角 [0, 270]°**（本器件行程）
 *    ⚠️ 与命令域（输出域绝对角，直行 = -90）**不同域**；换算归执行组件 steering，
 *       本文件不参与域换算（跨模块接口声明义务，指南 §4.2 义务 8）
 * ============================================================================
 */

#include <stdint.h>
#include "bridge_ret.h"

#define MAX_SERVOS 8

typedef enum
{
    SERVO_PROTOCOL_PWM  = 0,   /* 已实现 */
    SERVO_PROTOCOL_UART = 1,   /* 未实现：init 会返回 BRIDGE_ERR_BAD_CFG，不静默 */
} ServoProtocol_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（组装层注入；handle 指向协议所需的句柄，如 PWM_Handle*）---- */
typedef struct {
    ServoProtocol_t protocol;
    void           *handle;   /* 必填（NULL → BAD_ARG）。本工程为 PWM_Handle* */
} servo_bridge_cfg_t;

/**
 * @brief  初始化一个舵机通道（静态池 + placement new）
 * @retval BRIDGE_OK / BAD_ARG（id 越界、cfg/handle 为 NULL）/ BAD_CFG（协议未实现）
 */
bridge_ret_t servo_bridge_init(uint8_t id, const servo_bridge_cfg_t *cfg);

/* ---- 动作类：全部返回 bridge_ret_t；未初始化 → ERR_NOT_INIT ---- */
bridge_ret_t servo_bridge_set_angle(uint8_t id, float angle);  /* 物理角 ° */
bridge_ret_t servo_bridge_start(uint8_t id);                   /* 启动输出（PWM 起振） */

#ifdef __cplusplus
}
#endif

#endif /* SERVO_BRIDGE_H */
