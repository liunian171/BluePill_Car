/**
 * ============================================================================
 *  cmd_exec.h — 命令执行器（组装层 · 纯逻辑 · 全函数指针注入）
 * ============================================================================
 *
 *  层位: 组装层命令执行模块。零 HAL、零组件 include（全部经 io 注入），
 *        保 PC 桩可编译。
 *  职责: ① 文本命令表(exec_text_cmd 32 case) ② 二进制帧 7 分支
 *        ③ 应答编码(ack/tx_raw/cmd_note/dec1/spd_to_int)
 *  搬迁自: app_link.c 的 exec_text_cmd + 消费循环内 7 个二进制 if-else 分支
 *          + 出口助手(ack/tx_raw/cmd_note/dec1/spd_to_int)。
 *  语义承诺: 与搬迁前逐行为等价（应答字符串逐字节一致、命令级交织不变）。
 *
 *  ▸ R2 决策 (2026-09-21 用户拍板: 全函数指针注入) ◂
 *    组件/桥/宿主调用全部经 cmd_exec_io_t 函数指针注入 → PC 桩可链接纯 C 测试
 *    (host_cmd_exec_test.c), mock 记录调用序列 + 可控返回值精确断言应答字节。
 *    注入面 = 命令执行依赖的**全部**外部调用 (speed/line/steer/ctl/imu/mtr/arb)。
 *
 *  ▸ 门控归属 ◂
 *    指挥权门控(BUSY 判定)仍留 app_link 消费循环; 本模块只执行"已放行的命令"。
 *    LINK 命令内部仍用 io.arb_owner/arb_request 查询/切换指挥权 (安全例外已在
 *    app_link 放行)——故 io 注满 arb_owner/request 与 uart/usb_enabled 标志。
 * ============================================================================
 */

#ifndef CMD_EXEC_H
#define CMD_EXEC_H

#include <stdint.h>
#include "app/app_iface.h"
#include "driver/txt_cmd.h"
#include "driver/bridge_ret.h"
#include "driver/speed_loop.h"
#include "driver/steering.h"
#include "driver/link_arbiter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 各域回调 typedef（对齐组件/桥头签名）---- */

/* 出口（来自 app_link_io_t，原样迁入） */
typedef void (*cmd_sink_fn)(const char *buf, int n, void *ctx);              /* sink.send */
typedef void (*cmd_note_fn)(const char *s, void *ctx);                       /* note_cmd/note_resp */

/* speed_loop */
typedef speed_loop_ret_t (*cmd_spd_ret1_fn)(uint8_t id, void *ctx);          /* stop */
typedef speed_loop_ret_t (*cmd_spd_tgt_fn)(uint8_t id, float rpm, void *ctx);/* set_target */
typedef speed_loop_ret_t (*cmd_spd_setg_fn)(uint8_t id, float kp, float ki, float kd, void *ctx);
typedef speed_loop_ret_t (*cmd_spd_getg_fn)(uint8_t id, float *kp, float *ki, float *kd, void *ctx);
typedef speed_loop_ret_t (*cmd_spd_setff_fn)(uint8_t id, float gain, void *ctx);

/* line_follower */
typedef void (*cmd_line_u8_fn)(uint8_t v, void *ctx);                        /* enable/set_auto */
typedef void (*cmd_line_f_fn)(float v, void *ctx);                           /* set_kp/kd/speed */
typedef void (*cmd_line_void_fn)(void *ctx);                                 /* invert */
typedef int8_t (*cmd_line_inv_fn)(void *ctx);                                /* inverted */

/* steering */
typedef steering_ret_t (*cmd_steer_set_fn)(float v, void *ctx);
typedef float (*cmd_steer_get_fn)(void *ctx);
typedef steering_ret_t (*cmd_steer_nudge_fn)(float d, void *ctx);
typedef steering_ret_t (*cmd_steer_limit_fn)(float v, void *ctx);            /* set_limit_min/max/center */
typedef steering_ret_t (*cmd_steer_center_fn)(void *ctx);

/* app_control 命令入口 */
typedef void (*cmd_ctl_set_fn)(uint8_t v, void *ctx);                        /* tel/rec/odom */
typedef void (*cmd_ctl_void_fn)(void *ctx);                                  /* stop_all/rec_dump */
typedef bridge_ret_t (*cmd_ctl_step_fn)(uint8_t id, int16_t rate, uint32_t dur,
                                        uint8_t div, uint32_t now, void *ctx);
typedef bridge_ret_t (*cmd_ctl_wd_fn)(uint32_t ms, void *ctx);
typedef bridge_ret_t (*cmd_ctl_itel_fn)(uint8_t mode, void *ctx);
typedef bridge_ret_t (*cmd_ctl_irate_fn)(uint16_t ms, void *ctx);

/* imu_bridge */
typedef bridge_ret_t (*cmd_imu2_fn)(uint8_t id, uint8_t v, void *ctx);       /* set_drift_enable */
typedef bridge_ret_t (*cmd_imu_gain_fn)(uint8_t id, float kp, float ki, void *ctx);
typedef bridge_ret_t (*cmd_imu_void_fn)(uint8_t id, void *ctx);              /* recalibrate */

/* motor_bridge（二进制帧） */
typedef bridge_ret_t (*cmd_mtr_mps_fn)(uint8_t id, float mps, void *ctx);
typedef bridge_ret_t (*cmd_mtr_rate_fn)(uint8_t id, int16_t rate, void *ctx);

/* link_arbiter */
typedef LinkArbRet (*cmd_arb_req_fn)(LinkArbOwner target, void *ctx);
typedef LinkArbOwner (*cmd_arb_owner_fn)(void *ctx);

/* ---- 依赖注入（全部由 app_wiring 填充; 指针不得为 NULL） ---- */
typedef struct {
    /* --- 出口 --- */
    cmd_sink_fn  send;             /* sink.send 等价 (tx_raw 底层) */
    cmd_note_fn  note_cmd;         /* OLED 页6 命令记录 (cmd_note 等价) */
    cmd_note_fn  note_resp;        /* OLED 页4 应答记录 (ack 的页记录) */
    void (*override_manual)(void *ctx);   /* 手动指令优先: 退出巡线接管 */
    void (*wd_feed)(void *ctx);           /* WD 使能即刷新喂狗基准 */
    void (*ppr_set)(uint8_t id, uint16_t ppr, void *ctx); /* E 命令在线改 PPR */

    /* --- 链路标志（LINK 命令单链路判据; 由 wiring 从 cfg 翻译注入） --- */
    uint8_t uart_enabled;
    uint8_t usb_enabled;

    /* --- 组件/宿主/桥 (全注入) --- */
    /* speed_loop */
    cmd_spd_ret1_fn   spd_stop;
    cmd_spd_tgt_fn    spd_set_target;
    cmd_spd_setg_fn   spd_set_gains;
    cmd_spd_getg_fn   spd_get_gains;
    cmd_spd_setff_fn  spd_set_ff_gain;
    /* line_follower */
    cmd_line_u8_fn    line_enable;
    cmd_line_u8_fn    line_set_auto;
    cmd_line_f_fn     line_set_kp;
    cmd_line_f_fn     line_set_kd;
    cmd_line_f_fn     line_set_speed;
    cmd_line_void_fn  line_invert;
    cmd_line_inv_fn   line_inverted;
    void (*line_set_straight_cnt)(int32_t v, void *ctx);
    void (*line_set_turn_cnt)(int32_t v, void *ctx);
    /* steering */
    cmd_steer_set_fn  steer_set;
    cmd_steer_get_fn  steer_get;
    cmd_steer_nudge_fn steer_nudge;
    cmd_steer_limit_fn steer_set_limit_min;
    cmd_steer_limit_fn steer_set_limit_max;
    cmd_steer_limit_fn steer_set_center;
    cmd_steer_center_fn steer_center;
    /* app_control */
    cmd_ctl_void_fn   ctl_stop_all;
    cmd_ctl_step_fn   ctl_step_start;
    cmd_ctl_set_fn    ctl_tel_set;
    cmd_ctl_set_fn    ctl_rec_set;
    cmd_ctl_void_fn   ctl_rec_dump;
    cmd_ctl_set_fn    ctl_odom_set;
    cmd_ctl_wd_fn     ctl_wd_set;
    cmd_ctl_itel_fn   ctl_itel_set;
    cmd_ctl_irate_fn  ctl_imu_rate_set;
    /* imu */
    cmd_imu_gain_fn   imu_set_mahony_gains;
    cmd_imu2_fn       imu_set_drift_enable;
    cmd_imu_void_fn   imu_recalibrate;
    /* motor_bridge */
    cmd_mtr_mps_fn    mtr_set_speed_mps;
    cmd_mtr_rate_fn   mtr_set_rate_0E3;
    /* link_arbiter */
    cmd_arb_req_fn    arb_request;
    cmd_arb_owner_fn  arb_owner;

    void *ctx;
} cmd_exec_io_t;

/* ---- 入口（纯执行，不判门控） ---- */
/* 执行一条文本命令（txt_cmd_parse 已识别成功, app_link 已放行门控）。
 * tc 为解析结果; now = ms 时间基准 (STEP 时长内部用)。 */
void cmd_exec_text(const cmd_exec_io_t *io, const TxtCmd *tc, uint32_t now);

/* 执行一个二进制帧命令（app_link 已解析帧/校验长度/放行门控; 0xF0 心跳亦在此）。
 * cmd = 帧命令字节; data/len = 帧参数字节（不含 AA/CMD/FF/FF）；
 * now 保留（当前二进制分支未用, 为签名统一传入）。 */
void cmd_exec_bin(const cmd_exec_io_t *io, uint8_t cmd, const uint8_t *data, uint8_t len, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* CMD_EXEC_H */