/**
 * ============================================================================
 *  app_control.h — 50ms 控制节拍宿主（组装层）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主。零 HAL（判活信号经 io 注入, 保 PC 桩）。
 *  职责: 判活喂入仲裁 / 命令看门狗 / 调参工具链状态机 / 闭环搬运 / 感知喂入。
 *  换平台: 不动; 换整车: 改 cfg（值来自 car_config.h, 经 app_wiring 注入）。
 *
 *  吞掉的 main.c 现块: 主循环 ② 每 50ms 控制块（L986-1145）。
 *  消除的跨块共享: g_step_* / g_tel_* / g_rec_* / g_odom_on / g_itel_mode 6 组标志
 *  全部收编为私有状态, 经下方公开 API 被 app_link 命令驱动。
 * ============================================================================
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "driver/bridge_ret.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置（值由 app_wiring 从 car_config.h 翻译注入） ---- */
typedef struct {
    uint32_t ctrl_period_ms;      /* 控制节拍 (CAR_CTRL_PERIOD_MS, 现 50) */
    uint16_t rec_max;             /* 机内记录容量 (现 REC_MAX=128) */
    uint16_t imu_period_ms;       /* IMU 更新周期初值 (CAR_IMU_PERIOD_MS; IRATE 可改) */
    uint8_t  both_links;          /* 双链路并存才喂判活 (P3: 单链路不喂伪判活) */
    uint8_t  wd_default_off;      /* 看门狗默认关 (现 g_wd_ms=0; WD 命令可开) */
} app_control_cfg_t;

/* ---- 依赖注入（全部由 app_wiring 填充; 指针不得为 NULL） ---- */
typedef struct {
    uint8_t  (*usb_alive)(void *ctx);             /* 判活三信号: dev_state+pClassData+DTR
                                                     (HAL 读收敛在 app_wiring 绑定) */
    uint32_t (*last_rx_tick)(void *ctx);          /* owner 链最近下行字节时刻 (WD 基准;
                                                     消费侧写入, 归 app_link) */
    int32_t  (*enc_count)(uint8_t id, void *ctx); /* 编码器读 (id 0/1; 绑 encoder 句柄) */
    void (*send_line)(const char *s, int len, void *ctx); /* 裸发送 (tx_raw 等价; 不进页4)。
                                                     ⚠️ len 必须显式给: ODOM/ATT 为含 0x00 的二进制帧, 严禁 strlen */
    void (*note_cmd)(const char *s, void *ctx);   /* 页6 命令记录 (cmd_note 等价) */
    void (*resp)(const char *s, void *ctx);       /* 应答+页4 (ack 等价) */
    void (*delay_ms)(uint32_t ms, void *ctx);     /* 阻塞延时 (DUMP 重放用; 绑 HAL_Delay) */
    void *ctx;
} app_control_io_t;

/* ---- 生命周期 ---- */
bridge_ret_t app_control_init(const app_control_cfg_t *cfg, const app_control_io_t *io);

/* 节拍入口: 每 ctrl_period_ms 到期执行一次（含节拍门判断, now 由调用方传入）。
 * 内部: 判活喂入 link_arbiter → WD 判定 → STEP(激活时独占) →
 *       line_follower_update → speed_loop_update → odom_update/上行组帧。 */
void app_control_task(uint32_t now);

/* ---- 调参工具链（app_link 命令入口; 默认全关, STOP 联动全关） ----
 * 带返回码的入口含数值域校验（非法 → BAD_ARG, 不改状态）——校验单一来源在本模块 */
bridge_ret_t app_control_step_start(uint8_t motor_id, int16_t rate_0E3,
                                    uint32_t dur_ms, uint8_t tel_div, uint32_t now);
void         app_control_tel_set(uint8_t on);        /* 闭环遥测 10Hz CSV */
void         app_control_rec_set(uint8_t on);        /* 机内记录开关 (开启即清缓冲) */
void         app_control_rec_dump(void);             /* 20ms 匀速重放 (阻塞, 会话锁定) */
bridge_ret_t app_control_itel_set(uint8_t mode);     /* IMU 遥测 0关/1CSV/2VOFA+ */
bridge_ret_t app_control_imu_rate_set(uint16_t ms);  /* IMU 更新周期 20~1000ms */
void         app_control_odom_set(uint8_t on);       /* 上位机位姿帧 ODOM/ATT 开关 */
bridge_ret_t app_control_wd_set(uint32_t timeout_ms);/* 命令看门狗 0~60000ms, 0=关 */
void         app_control_wd_rearm(void);             /* owner 字节到达解除闩锁 (每字节调) */

/* ---- 查询 / 安全 ---- */
uint8_t app_control_session_active(void);   /* 仲裁会话锁判据 (BUSY:SESSION) */
void app_control_stop_all(void);            /* STOP/WD 共用急停收口: 全部工具链关闭 */

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
