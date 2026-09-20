/**
 * ============================================================================
 *  桥接层 — motor_bridge 实现（C 门面）
 * ============================================================================
 *
 *  家族契约：见 motor_bridge.h 文件头 / doc/桥接层家族契约.md
 *  本文件只做：查 id → 校验 → 按配置符号换算方向 → 转发给执行对象层。
 * ============================================================================
 */

#include "motor_bridge.h"
#include "motor_protocol.h"    /* TB6612MotorProtocol 完整定义 */
#include <new>                 /* placement new */
#include <stddef.h>

/* 静态池：编译时分配好内存，不碰堆 */
static uint8_t g_proto_mem[MAX_MOTORS][sizeof(TB6612MotorProtocol)];
static uint8_t g_motor_mem[MAX_MOTORS][sizeof(Motor)];
static TB6612MotorProtocol *tb6612_proto[MAX_MOTORS] = {nullptr};
static Motor *motor[MAX_MOTORS] = {nullptr};

/* 驱动侧方向符号（配置注入；两侧符号必须镜像，见 motor_bridge.h 注释） */
static int8_t g_drive_sign[MAX_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1};

/* 统一入口校验：id 越界 / 未初始化一律显式返回，不静默 */
static bridge_ret_t motor_bridge_chk(uint8_t id)
{
    if (id >= MAX_MOTORS) return BRIDGE_ERR_BAD_ARG;
    if (motor[id] == nullptr) return BRIDGE_ERR_NOT_INIT;
    return BRIDGE_OK;
}

bridge_ret_t motor_bridge_init(uint8_t id, const motor_bridge_cfg_t *cfg)
{
    if (cfg == nullptr) return BRIDGE_ERR_BAD_ARG;
    if (id >= MAX_MOTORS) return BRIDGE_ERR_BAD_ARG;      /* ← 原实现缺此检查，会越界写静态池 */
    if (cfg->pwm == nullptr || cfg->ain1 == nullptr || cfg->ain2 == nullptr)
        return BRIDGE_ERR_BAD_ARG;
    /* 用 !(x > 0) 写法同时挡掉 0 与 NaN */
    if (!(cfg->max_rpm > 0.0f) || !(cfg->wheel_radius_mm > 0.0f))
        return BRIDGE_ERR_BAD_ARG;
    if (cfg->protocol != MOTOR_PROTOCOL_TB6612)
        return BRIDGE_ERR_BAD_CFG;                        /* 未实现协议：显式报错，不静默 */

    g_drive_sign[id] = (cfg->drive_sign < 0) ? -1 : 1;

    /* placement new：在预分配的静态内存上构造对象，不占用堆 */
    tb6612_proto[id] = new (g_proto_mem[id]) TB6612MotorProtocol(cfg->pwm, cfg->ain1,
                                                                cfg->ain2, cfg->stby);
    motor[id] = new (g_motor_mem[id]) Motor(*tb6612_proto[id],
                                           cfg->max_rpm, cfg->wheel_radius_mm);
    return BRIDGE_OK;
}

bridge_ret_t motor_bridge_set_speed_rpm(uint8_t id, float rpm)
{
    bridge_ret_t r = motor_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    if (g_drive_sign[id] < 0) rpm = -rpm;   /* 方向符号：配置注入，不按 id 特例判断 */
    motor[id]->set_speed_rpm(rpm);
    return BRIDGE_OK;
}

bridge_ret_t motor_bridge_set_speed_mps(uint8_t id, float mps)
{
    bridge_ret_t r = motor_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    if (g_drive_sign[id] < 0) mps = -mps;
    motor[id]->set_speed_mps(mps);
    return BRIDGE_OK;
}

bridge_ret_t motor_bridge_set_rate_0E3(uint8_t id, int16_t rate_0E3)
{
    bridge_ret_t r = motor_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    if (g_drive_sign[id] < 0) rate_0E3 = (int16_t)(-rate_0E3);
    tb6612_proto[id]->set_speed_rate_0E3(rate_0E3);
    return BRIDGE_OK;
}

/* 2026-09-20 P1-6: motor_bridge_stop / motor_bridge_set_dead_zone 判死删除（§7），
 * 停车语义由 speed_loop_stop + brake 承担。 */

bridge_ret_t motor_bridge_brake(uint8_t id)
{
    bridge_ret_t r = motor_bridge_chk(id);
    if (r != BRIDGE_OK) return r;
    motor[id]->brake();
    return BRIDGE_OK;
}
