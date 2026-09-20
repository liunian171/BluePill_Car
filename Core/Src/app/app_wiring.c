/**
 * ============================================================================
 *  app_wiring.c — 装配模块（组装层的"唯一装配点"）
 * ============================================================================
 *
 *  层位: 组装层 app_* 宿主的装配点。⚠️ 全工程唯一 include car_config.h 的
 *        app 模块（其余 app_* 与组件/桥接层一律禁止, 风格指南 §4.5）。
 *  职责: ① 把 car_config.h 的 20 个标定宏翻译成各 cfg 结构体初值
 *        ② 填好全部 io 函数指针表（判活 HAL 读也在此绑定）
 *        ③ 按依赖序调用全部组件/桥/app init —— 任一失败即停
 *           (装载层收到非 OK → Error_Handler, "init 失败即停"语义不变, §22)
 *  搬迁自: main.c PV 区配置表 + USER CODE 2 init 链 + 全部 ctl_* 与 link_* 与 disp_* 绑定。
 *  换平台: 不动; 换整车: 只改 car_config.h。
 * ============================================================================
 */

#include "app/app_wiring.h"
#include "car_config.h"
#include "usart.h"               /* huart2 (UART 通道绑定) */
#include "i2c.h"                 /* hi2c2 (IMU/OLED 总线绑定) */
#include "tim.h"                 /* htim2/3 (编码器基座) */
#include "driver/uart_platform_ops.h"  /* uart_debug ops 表 */              /* ⚠️ 唯一 include 者 (组装层配置单) */
#include "driver/pwm.h"
#include "driver/uart.h"
#include "driver/motor_bridge.h"
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"
#include "driver/encoder.h"
#include "driver/oled_bridge.h"
#include "driver/oled_platform_ops.h"
#include "driver/imu_bridge.h"
#include "driver/i2c_hardware_ops.h"
#include "driver/line_follower.h"
#include "driver/odom.h"
#include "driver/servo_bridge.h"
#include "driver/steering.h"
#include "driver/speed_loop.h"
#include "driver/link_arbiter.h"
#include "app/app_tx.h"
#include "app/app_control.h"
#include "app/app_display.h"
#include "app/app_link.h"
#include "usbd_cdc_if.h"             /* usbd_cdc_if_register_rx_sink (P1-8) */
#include <string.h>

/* ---- 硬件句柄（原 main.c PV 区; 仅本模块与装载层 ISR 共享知识） ---- */

/* uart_debug 句柄归装载层 main.c (ISR 重挂 + bring-up 在彼处) */
extern USBD_HandleTypeDef hUsbDeviceFS;          /* usb_device.c 定义 */
extern volatile uint8_t  g_usb_dtr;              /* usbd_cdc_if.c DTR 捕获 */

static UserGPIO_Handle motor_a_in1 = { GPIOB, GPIO_PIN_13, &usergpio_platform_ops_stm32 };
static UserGPIO_Handle motor_a_in2 = { GPIOB, GPIO_PIN_12, &usergpio_platform_ops_stm32 };
static UserGPIO_Handle motor_b_in1 = { GPIOB, GPIO_PIN_14, &usergpio_platform_ops_stm32 };
static UserGPIO_Handle motor_b_in2 = { GPIOB, GPIO_PIN_15, &usergpio_platform_ops_stm32 };

static Encoder_Handle henc1, henc2;

static I2C_Handle imu_i2c = {
    .i2c_context = &hi2c2,
    .ops         = &i2c_hardware_platform_ops_stm32,
};

/* owner 链最近下行字节 (app_link io.wd_feed 回写, app_control io 读出) */
static uint32_t g_last_rx_tick = 0;

/* ---- 桥接层配置表（C1 家族契约：init 一律"配置结构注入"）----
 * ⚠️ drive_sign（驱动侧方向）与速度环标定表的 fb_sign（反馈侧符号）**必须镜像**：
 *    同一硬件事实的两面（MOTOR B 驱动与编码器接线均反相）；失配 = 正反馈飞车
 *    （2026-09-13 真机实证）。数值真值源 = car_config.h 的
 *    CAR_M1_DRIVE_SIGN / CAR_M1_FB_SIGN —— 两表同源引用, 相邻可查。 */

static const motor_bridge_cfg_t g_motor_cfg[2] = {
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch1, .ain1 = &motor_a_in1, .ain2 = &motor_a_in2, .stby = NULL,
      .max_rpm = CAR_MOTOR_MAX_RPM, .wheel_radius_mm = CAR_WHEEL_RADIUS_MM,
      .drive_sign =  1 },
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch2, .ain1 = &motor_b_in1, .ain2 = &motor_b_in2, .stby = NULL,
      .max_rpm = CAR_MOTOR_MAX_RPM, .wheel_radius_mm = CAR_WHEEL_RADIUS_MM,
      .drive_sign = CAR_M1_DRIVE_SIGN },   /* ← 必须与 g_spd_cfg.ch[1].fb_sign 同源 */
};

static const servo_bridge_cfg_t g_servo_cfg = {
    .protocol = SERVO_PROTOCOL_PWM,
    .handle   = &pwm_tim4_ch3,
};

static const oled_cfg_t g_oled_cfg = {
    .i2c_context = &hi2c2,
    .write       = oled_platform_write_stm32,   /* 平台绑定（器件层零 HAL） */
    .addr7       = 0x3C,
};

static const imu_bridge_cfg_t g_imu_cfg = {
    .type       = IMU_MPU6050,
    .i2c_handle = &imu_i2c,
};

/* ---- 速度环执行组件标定表（数值真值源 = car_config.h, 2026-09-19 标定集中）---- */

static speed_loop_cfg_t g_spd_cfg = {
    .ch_count = 2,
    .ch = {
        { .ppr = (float)CAR_PPR, .enc_span = CAR_ENC_SPAN, .fb_sign =  1,
          .ff_gain = CAR_SPD_FF_GAIN, .out_min = -CAR_MOTOR_MAX_RPM, .out_max = CAR_MOTOR_MAX_RPM,
          .kp = CAR_SPD_KP, .ki = CAR_SPD_KI, .kd = CAR_SPD_KD },
        { .ppr = (float)CAR_PPR, .enc_span = CAR_ENC_SPAN, .fb_sign = CAR_M1_FB_SIGN,
          .ff_gain = CAR_SPD_FF_GAIN, .out_min = -CAR_MOTOR_MAX_RPM, .out_max = CAR_MOTOR_MAX_RPM,
          .kp = CAR_SPD_KP, .ki = CAR_SPD_KI, .kd = CAR_SPD_KD },
    }
};

static void spd_io_set_rpm(uint8_t id, float rpm) { (void)motor_bridge_set_speed_rpm(id, rpm); }
static void spd_io_brake  (uint8_t id)            { (void)motor_bridge_brake(id); }
static int32_t spd_io_read_enc(uint8_t id)        { return (id == 0) ? encoder_get_count(&henc1)
                                                                      : encoder_get_count(&henc2); }
static const speed_loop_io_t g_spd_io = {
    .set_rpm  = spd_io_set_rpm,
    .brake    = spd_io_brake,
    .read_enc = spd_io_read_enc,
};

/* ---- 里程计感知组件标定表 + IO（yaw_sign = MPU6050 安装方向适配）---- */

static const odom_cfg_t g_odom_cfg = {
    .ppr    = { (float)CAR_PPR, (float)CAR_PPR },
    .wheel_circ_mm   = CAR_WHEEL_CIRC_MM,
    .wheel_track_mm  = CAR_WHEEL_TRACK_MM,          /* ⚠️ 未标定 — 本车走 IMU yaw 差分 */
    .enc_span        = (float)CAR_ENC_SPAN,
    .sign            = { 1, CAR_M1_FB_SIGN },       /* ⚠️ 与 g_spd_cfg 同源 (E2 接反) */
    .yaw_sign        = CAR_YAW_SIGN,
};
static int32_t odom_io_read_enc(uint8_t id)
{
    return (id == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
}
static void odom_io_get_yaw(float *yaw_deg)
{
    *yaw_deg = imu_bridge_get_yaw(0);        /* ° , 逆时针为正 (Mahony 融合输出) */
}
static const odom_io_t g_odom_io = {
    .read_enc = odom_io_read_enc,
    .get_yaw  = odom_io_get_yaw,
};

/* ---- 转向执行组件配置 + 输出注入（标定来源 = SS2 实车探边, 详舵机设计文档 §2）---- */

static const steering_cfg_t g_steering_cfg = {
    .lim_min     = CAR_SERVO_LIM_MIN,
    .lim_max     = CAR_SERVO_LIM_MAX,
    .center      = CAR_SERVO_CENTER,
    .phys_offset = CAR_SERVO_PHYS_OFFSET,
    .lim_abs     = CAR_SERVO_LIM_ABS,
    .servo_id    = 0,
};
static void steering_output_to_servo(uint8_t id, float phys_angle)
{
    (void)servo_bridge_set_angle(id, phys_angle);   /* 显示/控制链不处理桥错误码（契约 §2.3） */
}

/* ---- app_control / app_display / app_link 的 io 绑定（自 main.c 迁入）---- */

static uint8_t ctl_usb_alive(void *ctx)
{
    (void)ctx;
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
            hUsbDeviceFS.pClassData != NULL && g_usb_dtr) ? 1u : 0u;
}
static uint32_t ctl_last_rx_tick(void *ctx) { (void)ctx; return g_last_rx_tick; }
static int32_t  ctl_enc_count(uint8_t id, void *ctx)
{
    (void)ctx;
    return (id == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
}
static void ctl_send_line(const char *s, int len, void *ctx)
{   /* 发送出口经 app_tx (active 由 app_link 消费时登记) */
    (void)ctx; app_tx_send_by_link(app_tx_active(), s, len);
}
static void ctl_note_cmd(const char *s, void *ctx)  { (void)ctx; app_display_note_cmd(s, NULL); }
static void ctl_resp(const char *s, void *ctx)
{   /* ack 等价: 发送 + 页4 */
    (void)ctx;
    app_tx_send_by_link(app_tx_active(), s, (int)strlen(s));
    app_display_note_resp(s, NULL);
}
static void ctl_delay_ms(uint32_t ms, void *ctx)    { (void)ctx; HAL_Delay(ms); }

static const char *disp_owner_str(void *ctx)        { (void)ctx; return app_tx_owner_str(); }
static uint8_t  disp_usb_on(void *ctx)              { (void)ctx; return app_tx_usb_on(); }
static uint16_t disp_rx_overflow(void *ctx)         { (void)ctx; return app_link_rx_overflow(); }
static uint32_t disp_uart_silence_s(void *ctx)      { (void)ctx; return app_link_uart_silence_s(); }
static uint8_t disp_gray_read(uint8_t idx, void *ctx)
{   /* [P2-4 收敛点] 页 3 灰度读唯一 HAL 直读处 (原 main.c 与 line_follower 各一份) */
    (void)ctx;
    static const uint32_t ports[5] = {
        (uint32_t)OUT1_GPIO_Port, (uint32_t)OUT2_GPIO_Port, (uint32_t)OUT3_GPIO_Port,
        (uint32_t)OUT4_GPIO_Port, (uint32_t)OUT5_GPIO_Port };
    static const uint16_t pins[5] = {
        OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, OUT5_Pin };
    if (idx > 4) return 0;
    return (uint8_t)HAL_GPIO_ReadPin((GPIO_TypeDef *)ports[idx], pins[idx]);
}

/* io.sink 复用 ctl_send_line (与控制链同源出口) */
static void link_set_active(uint8_t link, void *ctx) { (void)ctx; app_tx_set_active(link); }
static uint8_t link_owner_link(void *ctx)            { (void)ctx; return app_tx_owner_main_link(); }
static void link_override_manual(void *ctx)          { (void)ctx; line_follower_enable(0); }
static void link_wd_feed(void *ctx)
{   /* owner 字节算"活" + 解除 WD 闩锁 */
    (void)ctx;
    g_last_rx_tick = HAL_GetTick();
    app_control_wd_rearm();
}
static uint8_t link_session_active(void *ctx)        { (void)ctx; return app_control_session_active(); }
/* LinkArbCfg 回调签名 (无 ctx 指针): 适配两处 io 函数 */
static uint8_t arb_session_active_cb(void) { return app_control_session_active(); }
static void arb_broadcast_cb(const char *msg)
{   /* 仲裁广播: 必须到达指挥权方 (send_to_owner 内做编号换算) */
    app_tx_send_to_owner(msg, (int)strlen(msg));
}
static void link_ppr_set(uint8_t id, uint16_t ppr, void *ctx)
{   /* E 命令在线改 PPR (编码器句柄属装配层) */
    (void)ctx;
    if (id == 0) henc1.ppr = ppr; else henc2.ppr = ppr;
}

/* ---- init 失败定位 ---- */
static const char *s_failed = "";

/* ---- 装配入口 ---- */

bridge_ret_t app_wiring_load(void)
{
    /* ---- 编码器 (PPR = car_config 单一真值源) ---- */
    henc1.htim  = &htim2;  henc1.ops = encoder_platform_get_ops();
    henc1.ppr   = CAR_PPR; henc1.position = 0;
    encoder_start(&henc1);
    henc2.htim  = &htim3;  henc2.ops = encoder_platform_get_ops();
    henc2.ppr   = CAR_PPR; henc2.position = 0;
    encoder_start(&henc2);

    /* ---- PWM 电机 20kHz 初始 0% ---- */
    pwm_set_freq(&pwm_tim1_ch1, CAR_MOTOR_PWM_HZ); pwm_set_duty_0E3(&pwm_tim1_ch1, 0); pwm_start(&pwm_tim1_ch1);
    pwm_set_freq(&pwm_tim1_ch2, CAR_MOTOR_PWM_HZ); pwm_set_duty_0E3(&pwm_tim1_ch2, 0); pwm_start(&pwm_tim1_ch2);

    /* ---- 电机桥（C1：配置结构注入 + init 失败即停）---- */
    s_failed = "motor";
    if (motor_bridge_init(0, &g_motor_cfg[0]) != BRIDGE_OK ||
        motor_bridge_init(1, &g_motor_cfg[1]) != BRIDGE_OK)
        return BRIDGE_ERR_IO;

    /* ---- 速度环（ppr 以编码器实测为准; 参数 = SIMC 平衡档, 2026-09-13）---- */
    s_failed = "speed_loop";
    g_spd_cfg.ch[0].ppr = (float)henc1.ppr;
    g_spd_cfg.ch[1].ppr = (float)henc2.ppr;
    if (speed_loop_init(&g_spd_cfg, &g_spd_io) != SPEED_LOOP_OK)
        return BRIDGE_ERR_IO;

    /* ---- 舵机 50Hz + 桥 + 转向: 上电回直行位 = 安全铁律 ---- */
    s_failed = "servo";
    pwm_set_freq(&pwm_tim4_ch3, CAR_SERVO_PWM_HZ); pwm_start(&pwm_tim4_ch3);
    if (servo_bridge_init(0, &g_servo_cfg) != BRIDGE_OK)
        return BRIDGE_ERR_IO;
    (void)servo_bridge_start(0);
    s_failed = "steering";
    if (steering_init(&g_steering_cfg, steering_output_to_servo) != STEERING_OK)
        return BRIDGE_ERR_IO;
    steering_center();

    /* ---- OLED ---- */
    s_failed = "oled";
    if (oled_bridge_init(&g_oled_cfg) != BRIDGE_OK)
        return BRIDGE_ERR_IO;
    oled_bridge_show_string_small(0,0,"BLUEPILL PID OK");
    oled_bridge_show_string_small(2,0,"Boot...");

    /* ---- 寻迹 (诊断回调注册归 app_link_init) ---- */
    s_failed = "line_follower";
    line_follower_init(CAR_LF_BASE_SPD, CAR_LF_KP);

    /* ---- IMU ---- */
    s_failed = "imu";
    if (imu_bridge_init(0, &g_imu_cfg) != BRIDGE_OK)
        return BRIDGE_ERR_IO;

    /* ---- 里程计 (C1 故障注入已真机验证: bad cfg → 整机拒运行, §22) ---- */
    s_failed = "odom";
    if (odom_init(&g_odom_cfg, &g_odom_io) != ODOM_OK)
        return BRIDGE_ERR_IO;

    /* ---- app_tx 发送出口 (先于任何发送: 仲裁广播/横幅) ---- */
    s_failed = "app_tx";
    {
        app_tx_cfg_t tx_cfg = {
            .uart_enabled          = CAR_FEATURE_UART,
            .usb_enabled           = CAR_FEATURE_USB,
            .usb_busy_timeout_ms   = 10,     /* 原直调口径 */
            .uart_block_timeout_ms = 100,
        };
        if (app_tx_init(&tx_cfg) != BRIDGE_OK) return BRIDGE_ERR_IO;
    }

    /* ---- app_control 50ms 节拍宿主 ---- */
    s_failed = "app_control";
    {
        app_control_cfg_t ctl_cfg = {
            .ctrl_period_ms = CAR_CTRL_PERIOD_MS,
            .rec_max        = 128,               /* 原 REC_MAX */
            .imu_period_ms  = CAR_IMU_PERIOD_MS,
            .both_links     = (uint8_t)(CAR_FEATURE_UART && CAR_FEATURE_USB),
            .wd_default_off = 1,                 /* WD 命令可开, 默认关 */
        };
        app_control_io_t ctl_io = {
            .usb_alive    = ctl_usb_alive,
            .last_rx_tick = ctl_last_rx_tick,
            .enc_count    = ctl_enc_count,
            .send_line    = ctl_send_line,
            .note_cmd     = ctl_note_cmd,
            .resp         = ctl_resp,
            .delay_ms     = ctl_delay_ms,
            .ctx          = NULL,
        };
        if (app_control_init(&ctl_cfg, &ctl_io) != BRIDGE_OK) return BRIDGE_ERR_IO;
    }

    /* ---- app_display 100ms 显示节拍宿主 ---- */
    s_failed = "app_display";
    {
        app_display_cfg_t disp_cfg = {
            .disp_period_ms = CAR_DISP_PERIOD_MS,
            .usb_enabled    = CAR_FEATURE_USB,   /* P3: 下线时页 7 不宣称在线 */
        };
        app_display_io_t disp_io = {
            .owner_str      = disp_owner_str,
            .usb_on         = disp_usb_on,
            .rx_overflow    = disp_rx_overflow,
            .uart_silence_s = disp_uart_silence_s,
            .enc_count      = ctl_enc_count,     /* 复用编码器绑定 */
            .gray_read      = disp_gray_read,
            .ctx            = NULL,
        };
        if (app_display_init(&disp_cfg, &disp_io) != BRIDGE_OK) return BRIDGE_ERR_IO;
    }

    /* ---- 指挥权仲裁初始化 (原 main.c init 块; 拆分时曾遗失 → 空指针广播死机) ----
     * alive_init: 双链路=未知(-1) 宽限观望; USB=0 判死(固定 UART); UART=0 判活(固定 USB) */
    s_failed = "link_arbiter";
    {
        LinkArbCfg ac = {
            .session_active = arb_session_active_cb,
            .broadcast      = arb_broadcast_cb,
            .boot_grace_ms  = 3000,
        };
#if !CAR_FEATURE_USB
        const int8_t alive_init = 0;    /* 无 USB 链路 → 固定 UART */
#elif !CAR_FEATURE_UART
        const int8_t alive_init = 1;    /* 无 UART 链路 → 固定 USB */
#else
        const int8_t alive_init = -1;   /* 未知, 待主循环喂判别活 */
#endif
        link_arb_init(&ac, alive_init, HAL_GetTick());
    }

    /* ---- app_link 命令链宿主 + P1-8 收包 sink 注册 ---- */
    s_failed = "app_link";
    {
        app_link_cfg_t link_cfg = {
            .uart_enabled     = CAR_FEATURE_UART,
            .usb_enabled      = CAR_FEATURE_USB,
            .byte_budget      = 64,     /* 原 LINK_BYTE_BUDGET (时间片封顶) */
            .frame_timeout_ms = 100,    /* 二进制帧超时重同步 */
        };
        app_link_io_t link_io = {
            .sink            = { ctl_send_line, NULL },   /* 出口与控制链同源 */
            .page_note       = { app_display_note_cmd, app_display_note_resp, NULL },
            .set_active      = link_set_active,
            .owner_link      = link_owner_link,
            .override_manual = link_override_manual,
            .wd_feed         = link_wd_feed,
            .session_active  = link_session_active,
            .ppr_set         = link_ppr_set,
            .ctx             = NULL,
        };
        if (app_link_init(&link_cfg, &link_io) != BRIDGE_OK) return BRIDGE_ERR_IO;
        usbd_cdc_if_register_rx_sink(app_link_usb_rx_isr);   /* [P1-8] */
    }

    s_failed = "";
    return BRIDGE_OK;
}

const char *app_wiring_failed_module(void)
{
    return s_failed;
}
