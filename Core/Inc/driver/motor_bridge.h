#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

/**
 * ============================================================================
 *  桥接层 — motor_bridge（电机 C 门面）
 * ============================================================================
 *
 *  层位：组装层 → 执行组件(speed_loop) → **本文件** → 执行对象层(Motor) → 器件层
 *  职责：查 id → 校验 → 转发。零业务逻辑、零 I/O、零 HAL。
 *
 *  ▸ 家族契约（正文 doc/桥接层家族契约.md，返回码 bridge_ret.h）◂
 *    · init 用配置结构注入：`motor_bridge_init(id, const motor_bridge_cfg_t*)`
 *    · 动作类返回 bridge_ret_t；id/handle/区间全部校验，不静默失败
 *    · 静态池 + placement new，禁堆 new
 *    · **方向符号 drive_sign 由配置注入**（不在本文件按 id 做特例判断）
 *
 *  ▸ 单位 ▸  转速 RPM / 线速度 m·s⁻¹ / 千分比 0E3（±1000 = ±100.0%）
 *  ▸ 坐标域 ▸ 全部为**带符号**量：正 = 该轮"正转"（与 drive_sign 相乘后的物理方向）
 *
 *  ▸ ⚠️ drive_sign 与速度环的反馈符号 fb_sign 必须镜像 ◂
 *    驱动侧（本文件 cfg.drive_sign）与反馈侧（speed_loop 标定表 cfg.fb_sign）
 *    是同一个硬件事实的两面：两侧不一致 → **正反馈飞车**（2026-09-13 真机实证）。
 *    故两者要求在组装层**相邻声明**，改动时必须成对。
 * ============================================================================
 */

#include <stdint.h>
#include "bridge_ret.h"
#include "pwm.h"
#include "usergpio.h"

#define MAX_MOTORS 8

typedef enum
{
    MOTOR_PROTOCOL_TB6612 = 0,   /* 已实现 */
    MOTOR_PROTOCOL_L298N = 1,    /* 未实现：init 会返回 BRIDGE_ERR_BAD_CFG，不静默 */
} MotorProtocol_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（"硬件接线知识"，由组装层注入）---- */
typedef struct {
    MotorProtocol_t  protocol;        /* 目前仅 TB6612 可用 */
    PWM_Handle      *pwm;             /* 必填 */
    UserGPIO_Handle *ain1;            /* 必填 */
    UserGPIO_Handle *ain2;            /* 必填 */
    UserGPIO_Handle *stby;            /* 可为 NULL（模块无 STBY 引脚） */
    float            max_rpm;         /* 必填，>0 */
    float            wheel_radius_mm; /* 必填，>0 */
    int8_t           drive_sign;      /* 驱动侧方向：+1 同相 / -1 硬件接反 */
} motor_bridge_cfg_t;

/**
 * @brief  初始化一个电机通道（静态池 + placement new）
 * @retval BRIDGE_OK / BAD_ARG（id 越界、cfg/handle 为 NULL、标定值非正）
 *         / BAD_CFG（协议未实现）
 */
bridge_ret_t motor_bridge_init(uint8_t id, const motor_bridge_cfg_t *cfg);

/* ---- 动作类：全部返回 bridge_ret_t；未初始化 → ERR_NOT_INIT ---- */
bridge_ret_t motor_bridge_set_speed_rpm(uint8_t id, float rpm);
bridge_ret_t motor_bridge_set_speed_mps(uint8_t id, float mps);
bridge_ret_t motor_bridge_set_rate_0E3(uint8_t id, int16_t rate_0E3); /* 千分比直通（阶跃测试用） */
bridge_ret_t motor_bridge_brake(uint8_t id);
/* 2026-09-20 P1-6: motor_bridge_stop / motor_bridge_set_dead_zone 按 §7 判死删除
 * （全仓零调用; 停车语义由 speed_loop_stop + brake 承担）。git 历史可取回。 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BRIDGE_H */
