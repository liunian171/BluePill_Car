/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : 完整版 — 串口命令 + PWM + OLED + IMU + 编码器 + 灰度
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"

/* USER CODE BEGIN Includes */
#include "driver/pwm.h"
#include "driver/uart.h"
#include "driver/uart_platform_ops.h"
#include "driver/motor_bridge.h"
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"
#include "driver/encoder.h"
#include "driver/oled_bridge.h"
#include "driver/oled_platform_ops.h"
#include "driver/imu_bridge.h"
#include "driver/i2c_hardware_ops.h"
#include "driver/line_follower.h"
#include "driver/txt_cmd.h"
#include "driver/odom.h"
#include "driver/servo_bridge.h"
#include "driver/steering.h"
#include "driver/speed_loop.h"
#include "common/ringbuf.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
UART_Handle    uart_debug = {
    .huart      = &huart2,
    .ops        = &uart_platform_ops_stm32,
};
static RingBuffer g_ringbuf_debug;

UserGPIO_Handle motor_a_in1 = { GPIOB, GPIO_PIN_13, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_a_in2 = { GPIOB, GPIO_PIN_12, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in1 = { GPIOB, GPIO_PIN_14, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in2 = { GPIOB, GPIO_PIN_15, &usergpio_platform_ops_stm32 };

Encoder_Handle henc1, henc2;
static I2C_Handle imu_i2c = {
    .i2c_context = &hi2c2,
    .ops         = &i2c_hardware_platform_ops_stm32,
};

/* ==== 桥接层配置表（C1 家族契约：init 一律"配置结构注入"）====
 * 桥只做"查 id → 校验 → 转发"，所有硬件接线知识（引脚/句柄/方向/地址）在此注入。 */

/* ⚠️ drive_sign（驱动侧方向）与下方速度环标定表的 fb_sign（反馈侧符号）**必须镜像**。
 *    两者是同一个硬件事实的两面（MOTOR B 的驱动与编码器接线均反相）；
 *    只改一侧 = 正反馈飞车（2026-09-13 真机实证：两侧失配时 M1 上电持续加速）。
 *    故两者必须相邻声明，改动时成对修改。 */
static const motor_bridge_cfg_t g_motor_cfg[2] = {
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch1, .ain1 = &motor_a_in1, .ain2 = &motor_a_in2, .stby = NULL,
      .max_rpm = 319.0f, .wheel_radius_mm = 32.5f,
      .drive_sign =  1 },
    { .protocol = MOTOR_PROTOCOL_TB6612,
      .pwm = &pwm_tim1_ch2, .ain1 = &motor_b_in1, .ain2 = &motor_b_in2, .stby = NULL,
      .max_rpm = 319.0f, .wheel_radius_mm = 32.5f,
      .drive_sign = -1 },   /* ← 必须与 g_spd_cfg.ch[1].fb_sign 同号 */
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

/* ==== 速度环执行组件 (speed_loop) 的标定表 —— 组装层只留"配置 + 接线" ====
 * PID / 前馈 / 测速 / 限幅 / 停车语义 / 状态保持 已全部归位到组件内部（C2, 2026-09-13）。
 * 下列数值为"车知识/硬件知识"，全部经本表注入；换车/换电机只改本表，不改组件。
 *   ppr        : 编码器每转脉冲数（init 时用 henc 实测值覆盖，此处为缺省回退）
 *   enc_span   : 16 位定时器计数模值（回绕校正用）；换 32 位编码器置 0
 *   fb_sign    : 反馈符号，必须镜像驱动侧取反（E2 硬件接反 → id1 = -1）
 *   ff_gain    : 前馈系数（FF 命令在线整定，辨识值 1/K ≈ 1.0）
 *   out ±319   : 输出限幅 = 电机 max_rpm 实测
 *   kp/ki/kd   : SIMC 阶跃辨识值（2026-09-13，见 tools/step_ident.py） */
static speed_loop_cfg_t g_spd_cfg = {
    .ch_count = 2,
    .ch = {
        /* id0 (MOTOR A, E1 同向) */
        { .ppr = 1466.0f, .enc_span = 65536, .fb_sign =  1,
          .ff_gain = 1.0f, .out_min = -319.0f, .out_max = 319.0f,
          .kp = 0.87f, .ki = 0.40f, .kd = 0.0f },
        /* id1 (MOTOR B, E2 硬件接反 → 反馈同号取反) */
        { .ppr = 1466.0f, .enc_span = 65536, .fb_sign = -1,
          .ff_gain = 1.0f, .out_min = -319.0f, .out_max = 319.0f,
          .kp = 0.87f, .ki = 0.40f, .kd = 0.0f },
    }
};

/* ==== 决策组件的输出缓冲（巡线写意图 → 组装层推给执行组件）====
 * 与执行组件解耦：巡线仍按"目标幅值 + 方向"两数组接口输出（结构保留不动），
 * 组装层负责合成带符号目标；巡线未启用时该缓冲不参与（命令下发的意图保持不变）。 */
static float        g_lf_tgt[2]     = {0.0f, 0.0f};
static int8_t       g_lf_dir[2]     = {0, 0};

static char         s_last_cmd[20]  = "NONE";  /* 最近执行的命令 (OLED 页6 显示) */static char         s_last_resp[20] = "-";     /* 最近应答/回复 (OLED 页4 显示) */
static uint32_t     t_frame         = 0;       /* 二进制帧最近字节时间戳 (超时重同步) */

/* ==== 调参工具链 (工程支持层, 可开关, 默认关 — STOP 急停自动关闭) ==== */
static uint8_t      g_step_active   = 0;       /* 开环阶跃测试进行中 (激活时独占 50ms 控制帧) */
static uint8_t      g_step_m        = 0;       /* 测试电机 id */
static int16_t      g_step_rate     = 0;       /* 阶跃千分比 (±1000) */
static uint32_t     g_step_t0       = 0;       /* 阶跃起始 tick (遥测 t 基准) */
static uint32_t     g_step_end      = 0;       /* 阶跃结束 tick */
static uint8_t      g_step_first    = 1;       /* 首帧测速基准初始化标志 (建基准并丢弃该帧) */
/* 注: 阶跃测速基准/前馈系数已归位到 speed_loop 组件（原 g_step_prev_enc/tick、g_ff_gain）*/

/* 机内记录: 20Hz 写 RAM, DUMP 重放 — 对抗无线链路丢行 (丢行可重试补全) */
#define REC_MAX 128
static uint8_t      g_rec_on        = 0;
static uint16_t     g_rec_cnt       = 0;
static uint32_t     rec_tick[REC_MAX];
static int16_t      rec_t0[REC_MAX], rec_r0[REC_MAX];
static int16_t      rec_t1[REC_MAX], rec_r1[REC_MAX];
static uint8_t      g_step_div      = 1;       /* 遥测分频: 1=20Hz 2=10Hz (弱链路降频防丢行) */
static uint8_t      g_step_cnt      = 0;       /* 遥测分频计数 */
static uint8_t      g_tel_on        = 0;       /* 闭环遥测开关 (10Hz CSV, 阻塞发送, 仅整定会话开启) */
static uint8_t      g_tel_div       = 0;       /* 50ms→100ms 分频 */

/* ==== 上位机对接 (2026-09-14): 里程计上报 + 命令看门狗 ====
 * 上行二进制帧 (复用 0xAA...FFFF 帧体系, 无校验 — 需求方确认只要增量+时间戳):
 *   ODOM 0x51: 帧头|ΔX|ΔY|Δθ|ts_ms|FFFF = 20B @20Hz (50ms, 与控制环同拍)
 *   ATT  0x52: 帧头|roll|pitch|yaw|ts_ms|FFFF = 20B @10Hz
 * 带宽: 全开 ≈ 600B/s ≈ 62%@9600 — 仅对接会话开启; 20ms 周期(50Hz)需先提速(见 README §3.9) */
#define ODOM_FRAME_CMD   0x51
#define ATT_FRAME_CMD    0x52
static uint8_t      g_odom_on       = 0;       /* 里程计上报开关 (ODOM 命令 / STOP 关闭) */
static uint8_t      g_att_div       = 0;       /* ATT 10Hz 分频 */

/* 命令看门狗: WD <ms> 使能; 超时无任何下行字节 → 急停 (上位机断链兜底, PDF 安全机制条款) */
static uint32_t     g_wd_ms         = 0;       /* 0 = 关 (默认, 保持现有人机行为) */
static uint32_t     g_last_rx_tick  = 0;       /* 最近下行字节时刻 (主循环每字节刷新) */
static uint8_t      g_wd_fired      = 0;       /* 触发闩锁: 防止超时期间反复急停刷屏 */

/* ==== 里程计感知组件 (odom) 的标定表 + IO 绑定 — 车知识集中注入 ==== */
static const odom_cfg_t g_odom_cfg = {
    .ppr    = { 1466.0f, 1466.0f },          /* 编码器实测 (README §1) */
    .wheel_circ_mm   = 205.0f,               /* 轮周长 (README 权威 20.5cm) */
    .wheel_track_mm  = 0.0f,                 /* 轮距 ⚠️ 待标定 — 本车走 IMU yaw 差分, 不参与 */
    .enc_span        = 65536.0f,
    .sign            = { 1, -1 },            /* ⚠️ 必须镜像 g_spd_cfg.ch[].fb_sign (E2 接反) */
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
    .get_yaw  = odom_io_get_yaw,             /* NULL 则回退编码器差分(需轮距) */
};
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* ==== 速度环组件的 IO 绑定（组装层职责：把组件接口接到器件/传感器）====
 * 组件只持函数指针（不 include 桥接层头），故执行栈向上不反向依赖。
 * 换驱动芯片 / 换编码器接口只改这三个绑定，组件与上层不动。 */
static void spd_io_set_rpm(uint8_t id, float rpm) { (void)motor_bridge_set_speed_rpm(id, rpm); }
static void spd_io_brake  (uint8_t id)            { (void)motor_bridge_brake(id); }
static int32_t spd_io_read_enc(uint8_t id)        { return (id == 0) ? encoder_get_count(&henc1)
                                                                      : encoder_get_count(&henc2); }
static const speed_loop_io_t g_spd_io = {
    .set_rpm  = spd_io_set_rpm,
    .brake    = spd_io_brake,
    .read_enc = spd_io_read_enc,
};

/* 状态读出小工具: 带符号浮点 → 四舍五入到 int（遥测/显示共用，避免各处重复写） */
static int spd_to_int(float v)
{
    return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* 中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hal_huart)
{
    if (hal_huart->Instance != USART2) return;
    ringbuf_write(&g_ringbuf_debug, uart_debug.rx_byte);
    HAL_UART_Receive_IT(hal_huart, &uart_debug.rx_byte, 1);
}

/* 裸发送: 不记录 OLED 应答页 (寻迹诊断等高频事件用) */
static void tx_raw(const char *buf, int n)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
}

/* 文本回应: 发送 + 记录到 OLED 页4 (指令接收后的回复, 无线调试确认) */
static void ack(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        tx_raw(buf, n);
        /* 手动截断复制, 消除 -Wformat-truncation (截断是预期行为) */
        size_t rn = ((size_t)n < sizeof(s_last_resp) - 1) ? (size_t)n : sizeof(s_last_resp) - 1;
        memcpy(s_last_resp, buf, rn);
        s_last_resp[rn] = '\0';
    }
}

/* 寻迹诊断代理：转发 line_follower 事件到串口 (高频, 不占用应答页) */
static void line_diag(const char *msg)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", msg);
    if (n > 0) tx_raw(buf, n);
}

/* float → 1位小数的十分位数字 (替代 %.1f: newlib-nano 缺 _printf_float) */
static int dec1(float v)
{
    if (v < 0) v = -v;
    int d = (int)((v - (float)(int)v) * 10.0f + 0.5f);
    return (d > 9) ? 9 : d;
}

/* ---- 转向执行组件 (steering) 配置 + 输出绑定 ----
 * 坐标系: 输出域绝对角(°) — 直行 = -90, 右满舵 = -115(下限), 左满舵 = -65(上限), 行程 ±25
 * 标定来源: SS2 实车手动探边 (2026-09-09), 详见 doc/舵机转向设计文档.md §2
 * 物理角 = 输出角 + SERVO_PHYS_OFFSET (135° 电气中位 = 前轮正前, 摆臂安装偏置)
 * ⚠️ 方向备忘: 指令正向(朝 -65) = 用户标"左"——摆臂偏装 ~90° 且方向与常规相反,
 *   SS3 转向模型统一符号; 直行位 ≠ 0 由实测决定, SV 0 会 clamp 到左满舵
 * 注: 标定值集中于此由组装层注入; 待 car_config 表建立后迁入(结构优化顺序分析 §8.2 序 7) */
#define SERVO_PHYS_OFFSET   135.0f    /* 输出域 → 物理角 安装偏置(°) */
#define SERVO_LIM_MIN     (-115.0f)   /* 输出域下限 = 右满舵(实测) */
#define SERVO_LIM_MAX     ( -65.0f)   /* 输出域上限 = 左满舵(实测) */
#define SERVO_CENTER      ( -90.0f)   /* 直行位(°) (实测) — 上电/STOP 目标 */
#define SERVO_LIM_ABS       135.0f    /* 输出域绝对值上限(配置校验用) */

static const steering_cfg_t g_steering_cfg = {
    .lim_min     = SERVO_LIM_MIN,
    .lim_max     = SERVO_LIM_MAX,
    .center      = SERVO_CENTER,
    .phys_offset = SERVO_PHYS_OFFSET,
    .lim_abs     = SERVO_LIM_ABS,
    .servo_id    = 0,
};

/* steering → 桥接层 输出注入
 * (组件不 include 桥接层头, 保证 steering.c 零依赖、PC 桩可编译 — 指南 §3.2) */
static void steering_output_to_servo(uint8_t id, float phys_angle)
{
    (void)servo_bridge_set_angle(id, phys_angle);   /* 显示/控制链不处理桥错误码（契约 §2.3） */
}

/* 记录最近执行的命令, OLED 页6 显示, 用于无线命令执行确认 */
static void cmd_note(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_last_cmd, sizeof(s_last_cmd), fmt, ap);
    va_end(ap);
}

/* OLED 整行写入: 补齐/截断到 21 字符, 定宽覆盖无残留, 无需先清屏 */
static void oled_line(uint8_t page, const char *s)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%-21.21s", s);
    oled_bridge_show_line_small(page, buf);
}
/* USER CODE END 0 */

int main(void)
{

  HAL_Init();
    SystemClock_Config();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_I2C2_Init();

    /* USER CODE BEGIN 2 */
    /* ---- UART (BT04 蓝牙串口) ----
     * PA2/PA3 现接 BT04, 透明传输波特率 9600, 覆盖 CubeMX 默认 115200 */
    huart2.Init.BaudRate = 9600;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    ringbuf_init(&g_ringbuf_debug);
    HAL_UART_Receive_IT(&huart2, &uart_debug.rx_byte, 1);
    HAL_UART_Transmit(&huart2, (uint8_t *)"UART2 Ready\r\n", 13, 100);

    /* ---- 编码器 ---- */
    henc1.htim  = &htim2;  henc1.ops = encoder_platform_get_ops();
    henc1.ppr   = 1466;    henc1.position = 0;
    encoder_start(&henc1);
    henc2.htim  = &htim3;  henc2.ops = encoder_platform_get_ops();
    henc2.ppr   = 1466;    henc2.position = 0;
    encoder_start(&henc2);

    /* ---- PWM 20kHz 初始 0% ---- */
    pwm_set_freq(&pwm_tim1_ch1, 20000); pwm_set_duty_0E3(&pwm_tim1_ch1, 0); pwm_start(&pwm_tim1_ch1);
    pwm_set_freq(&pwm_tim1_ch2, 20000); pwm_set_duty_0E3(&pwm_tim1_ch2, 0); pwm_start(&pwm_tim1_ch2);

    /* ---- 电机桥（C1：配置结构注入 + init 失败即停）---- */
    if (motor_bridge_init(0, &g_motor_cfg[0]) != BRIDGE_OK ||
        motor_bridge_init(1, &g_motor_cfg[1]) != BRIDGE_OK)
        Error_Handler();

    /* ---- 速度环执行组件 (参数 = SIMC 辨识建议[平衡]档, 2026-09-13 阶跃辨识,
     *      A/B 实测验证: 上升 90% 150ms/零超调 vs 旧参 >3s; 详见 tools/step_ident.py) ----
     * ppr 一律以编码器实测值为准（单一数据源），标定表里的字面量只作缺省回退 */
    g_spd_cfg.ch[0].ppr = (float)henc1.ppr;
    g_spd_cfg.ch[1].ppr = (float)henc2.ppr;
    if (speed_loop_init(&g_spd_cfg, &g_spd_io) != SPEED_LOOP_OK)
        Error_Handler();

    /* ---- 舵机 50Hz ---- */
    pwm_set_freq(&pwm_tim4_ch3, 50); pwm_start(&pwm_tim4_ch3);

    /* ---- 舵机桥 (PB8) + 转向执行组件: 上电回直行位 = 安全铁律 ---- */
    if (servo_bridge_init(0, &g_servo_cfg) != BRIDGE_OK)
        Error_Handler();
    (void)servo_bridge_start(0);
    if (steering_init(&g_steering_cfg, steering_output_to_servo) != STEERING_OK)
        Error_Handler();
    steering_center();

    /* ---- OLED ---- */
    if (oled_bridge_init(&g_oled_cfg) != BRIDGE_OK)
        Error_Handler();
    oled_bridge_show_string_small(0,0,"BLUEPILL PID OK");
    oled_bridge_show_string_small(2,0,"Boot...");

    /* ---- 寻迹 ---- */
    line_follower_init(30.0f, 10.0f);
    line_follower_set_event_cb(line_diag);

    /* ---- IMU ---- */
    if (imu_bridge_init(0, &g_imu_cfg) != BRIDGE_OK)
        Error_Handler();

    /* ---- 里程计感知组件 (ΔX/ΔY/Δθ 增量位姿, 上位机 ODOM 上报数据源) ----
     * 依赖 imu_bridge 已 init（yaw 注入）。基准在主循环首拍由 update 自动建立。 */
    if (odom_init(&g_odom_cfg, &g_odom_io) != ODOM_OK)
        Error_Handler();
    /* USER CODE END 2 */

    /* ---- 主循环 ---- */
    uint8_t  frame[32];
    uint8_t  f_len = 0;
    uint8_t  in_frame = 0;
    uint32_t t_pid = 0;
    uint32_t t_disp = 0;

    while (1)
    {
        /* ① 串口命令解析 — 一次读完 ringbuf 所有积压字节 */
        static uint8_t txt[20];
        static uint8_t txt_len = 0;
        uint8_t b;
        while (ringbuf_read(&g_ringbuf_debug, &b) == 0) {
            g_last_rx_tick = HAL_GetTick();   /* 看门狗: 任何下行字节都算"活" */
            g_wd_fired = 0;                   /* 解除触发闩锁 (断链恢复即重新武装) */
            if (b == 0xAA && !in_frame) { f_len = 0; in_frame = 1; t_frame = HAL_GetTick(); }
            if (in_frame) {
                if (f_len >= sizeof(frame)) { in_frame = 0; continue; }
                frame[f_len++] = b;
                t_frame = HAL_GetTick();
                if (f_len >= 2 && frame[f_len-2] == 0xFF && frame[f_len-1] == 0xFF) {
                    uint8_t cmd = frame[1], flen = f_len - 2;
                    if      (cmd == 0x01 && flen >= 7 && frame[2] < 2) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); line_follower_enable(0); speed_loop_set_target(id, v); cmd_note("M%d=%dRPM", id, (int)((v>0)?v:-v)); ack("M%d:%dRPM\r\n",id,(int)(v+0.5f)); }
                    else if (cmd == 0x02 && flen >= 7 && frame[2] < 2) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); line_follower_enable(0); (void)motor_bridge_set_speed_mps(id,v); cmd_note("M%d=%dcm/s", id, (int)(v*100)); ack("M%d:%dcm/s\r\n",id,(int)(v*100+0.5f)); }
                    else if (cmd == 0x03 && flen >= 3 && frame[2] < 2) { uint8_t id = frame[2]; line_follower_enable(0); speed_loop_stop(id); cmd_note("BRK%d", id); ack("M%d:BRAKE\r\n",id); }
                    else if (cmd == 0x10 && flen >= 7 && frame[2] < 1) { float v; memcpy(&v,&frame[3],4); steering_set(v); float a = steering_get(); cmd_note("SV=%d.%d", (int)a, dec1(a)); ack("SV:%d.%d\r\n", (int)a, dec1(a)); }
                    else if (cmd == 0xE0 && flen >= 15 && frame[2] < 2) { uint8_t id = frame[2]; float kp,ki,kd; memcpy(&kp,&frame[3],4); memcpy(&ki,&frame[7],4); memcpy(&kd,&frame[11],4); speed_loop_set_gains(id,kp,ki,kd); cmd_note("PID%d", id); ack("OK\r\n"); }
                    else if (cmd == 0x20 && flen >= 7 && frame[2] < 2) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); if (v >= -100.0f && v <= 100.0f) { line_follower_enable(0); (void)motor_bridge_set_rate_0E3(id, (int16_t)(v * 10.0f)); cmd_note("DUTY%d", id); ack("D%d:%d.%d\r\n", id, (int)v, dec1(v)); } else { cmd_note("DUTY BAD"); ack("?\r\n"); } }
                    else if (cmd == 0xF0) { cmd_note("PING->PONG"); ack("PONG\r\n"); }
                    else { cmd_note("ERR:%02X", cmd); ack("?\r\n"); }
                    in_frame = 0; f_len = 0;
                }
                continue;
            }
            if (b == '\r') continue;  /* 兼容手机APP "\r\n" 行尾: 否则 "PING\r" 匹配失败 */
            if (b == '\n') txt[txt_len] = 0;
            else if (txt_len < 19) { txt[txt_len++] = b; continue; }
            if (txt_len > 0) {
                /* 文本命令解析已模块化到 txt_cmd.c (PC 测试桩覆盖 41 用例),
                 * 手写数值解析替代 sscanf %f (newlib-nano 缺 _scanf_float 静默失败) */
                TxtCmd tc;
                if (txt_cmd_parse((char *)txt, &tc)) {
                    switch (tc.type) {
                    case TXTCMD_PING:
                        cmd_note("PING->PONG");
                        ack("PONG\r\n");
                        break;
                    case TXTCMD_STOP:
                        for (int i = 0; i < 2; i++) speed_loop_stop(i);
                        /* 急停语义 = 彻底停: 必须同步关巡线+自动启动,
                         * 否则 50ms 后状态机重写目标, 车会"复活"(实测踩坑) */
                        line_follower_enable(0);
                        line_follower_set_auto(0);
                        steering_center();  /* 前轮回直行位 (实测 -90, 非 0) */
                        /* 调参工具链随急停关闭 (阶跃测试中途=安全终止, 遥测/记录回默认关) */
                        g_step_active = 0; g_tel_on = 0; g_rec_on = 0;
                        g_odom_on = 0;    /* 上位机对接上报同样关闭 (看门狗保持武装) */
                        cmd_note("STOP ALL");
                        ack("STOP OK LINE OFF\r\n");
                        break;
                    case TXTCMD_MOTOR: {
                        int id = tc.i0;
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先: 退出巡线接管, 否则 50ms 后被覆盖 */
                        speed_loop_set_target(id, v);   /* 带符号目标: 方向由符号统一表达 */
                        cmd_note("M%d=%dRPM", id, (int)((v >= 0) ? v : -v));
                        ack("M%d:%dRPM\r\n", id, (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_MOTOR_BOTH: {
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        speed_loop_set_target(0, v);
                        speed_loop_set_target(1, v);
                        cmd_note("MS=%dRPM", (int)((v >= 0) ? v : -v));
                        ack("MS:%dRPM\r\n", (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_BRAKE: {
                        int id = tc.i0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        speed_loop_stop(id);
                        cmd_note("BRK%d", id);
                        ack("M%d:BRAKE\r\n", id);
                        break;
                    }
                    case TXTCMD_PID_SET: {
                        int lo = (tc.i0 < 0) ? 0 : tc.i0, hi = (tc.i0 < 0) ? 1 : tc.i0;
                        for (int i = lo; i <= hi; i++) {
                            speed_loop_set_gains(i, tc.i1*0.01f, tc.i2*0.01f, tc.i3*0.01f);
                        }
                        if (tc.i0 < 0) ack("PID ALL:%d %d %d\r\n", tc.i1, tc.i2, tc.i3);
                        else           ack("PID%d:%d %d %d\r\n", tc.i0, tc.i1, tc.i2, tc.i3);
                        cmd_note("PID SET");
                        break;
                    }
                    case TXTCMD_PID_SWAP: {
                        /* 增益真值从组件读（不再维护 ×100 镜像数组 → 消除"二进制改参后镜像失效"的漂移） */
                        float kp[2], ki[2], kd[2];
                        speed_loop_get_gains(0, &kp[0], &ki[0], &kd[0]);
                        speed_loop_get_gains(1, &kp[1], &ki[1], &kd[1]);
                        speed_loop_set_gains(0, kp[1], ki[1], kd[1]);
                        speed_loop_set_gains(1, kp[0], ki[0], kd[0]);
                        cmd_note("PID SWAP");
                        ack("SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
                            spd_to_int(kp[1]*100.0f), spd_to_int(ki[1]*100.0f), spd_to_int(kd[1]*100.0f),
                            spd_to_int(kp[0]*100.0f), spd_to_int(ki[0]*100.0f), spd_to_int(kd[0]*100.0f));
                        break;
                    }
                    case TXTCMD_LINE_EN: {
                        int en = tc.i0 ? 1 : 0;
                        line_follower_enable(en);
                        if (!en) { speed_loop_set_target(0, 0.0f); speed_loop_set_target(1, 0.0f); }
                        cmd_note("LINE %s", en ? "ON" : "OFF");
                        ack("LINE:%s\r\n", en ? "ON" : "OFF");
                        break;
                    }
                    case TXTCMD_LINE_AUTO:
                        line_follower_set_auto(tc.i0 ? 1 : 0);
                        cmd_note("AUTO %d", tc.i0 ? 1 : 0);
                        ack("LA:%d\r\n", tc.i0 ? 1 : 0);
                        break;
                    case TXTCMD_GK:
                        line_follower_set_kp(tc.f0);
                        cmd_note("GK=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GK:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GD:
                        line_follower_set_kd(tc.f0);
                        cmd_note("GD=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GD:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GS:
                        line_follower_set_speed(tc.f0);
                        cmd_note("GS=%d.%d", (int)tc.f0, dec1(tc.f0));
                        ack("GS:%d.%d\r\n", (int)tc.f0, dec1(tc.f0));
                        break;
                    case TXTCMD_GI:
                        line_follower_invert();
                        cmd_note("GI");
                        ack("GI:%d\r\n", line_follower_inverted());
                        break;
                    case TXTCMD_GC:
                        line_follower_set_straight_cnt(tc.i0);
                        cmd_note("GC=%d", tc.i0);
                        ack("GC:%d\r\n", tc.i0);
                        break;
                    case TXTCMD_GT:
                        line_follower_set_turn_cnt(tc.i0);
                        cmd_note("GT=%d", tc.i0);
                        ack("GT:%d\r\n", tc.i0);
                        break;
                    case TXTCMD_PPR:
                        if (tc.i0 == 0) henc1.ppr = (uint16_t)tc.i1;
                        else            henc2.ppr = (uint16_t)tc.i1;
                        cmd_note("PPR%d=%d", tc.i0, tc.i1);
                        ack("PPR%d:%d\r\n", tc.i0, tc.i1);
                        break;
                    case TXTCMD_SERVO: {
                        steering_set(tc.f0);
                        float a = steering_get();            /* 回显实际生效角(钳位后) */
                        cmd_note("SV=%d.%d", (int)a, dec1(a));
                        ack("SV:%d.%d\r\n", (int)a, dec1(a));
                        break;
                    }
                    case TXTCMD_SERVO_NUDGE: {
                        steering_nudge((float)tc.i0);
                        float a = steering_get();
                        cmd_note("SV=%d.%d", (int)a, dec1(a));
                        ack("SV:%d.%d\r\n", (int)a, dec1(a));
                        break;
                    }
                    case TXTCMD_SERVO_LIM: {
                        /* 限位=配置态: 由 steering 组件唯一持有并归一化
                         * (下限<上限自动交换、限幅到 ±lim_abs、直行位拉回区间内) */
                        steering_ret_t r;
                        if      (tc.i0 == 0) r = steering_set_limit_min(tc.f0);
                        else if (tc.i0 == 1) r = steering_set_limit_max(tc.f0);
                        else                 r = steering_set_center(tc.f0);
                        /* 非法值: 组件已钳位保底, 此处回显请求值并标记 BAD (义务 6: 不静默) */
                        cmd_note("S%c=%d.%d%s", "LRC"[tc.i0], (int)tc.f0, dec1(tc.f0),
                                 (r == STEERING_OK) ? "" : "!");
                        ack("S%c:%d.%d%s\r\n", "LRC"[tc.i0], (int)tc.f0, dec1(tc.f0),
                            (r == STEERING_OK) ? "" : " BAD");
                        break;
                    }
                    /* ==== 调参工具链命令 (工程支持层, 可开关默认关, STOP 随时安全终止) ====
                     * [对未完善部分的假设] ① 接管权仲裁(C4)/桥接层契约(C1)尚未实施, 本工具暂按
                     *   "手动命令同模式" 取接管权: line_follower_enable(0) + speed_loop_stop()
                     *   (清目标 + 释放 PID + 物理刹停); 框架完善后应改为注册制接管, 对接点仅此一处;
                     * ② 开环输出经 motor_bridge_set_rate_0E3 直通(桥接层稳定边界), 千分比域
                     *   与 TB6612Protocol 一致, 换驱动芯片时仅桥内适配 */
                    case TXTCMD_STEP: {
                        if (g_step_active) { ack("STEP BUSY\r\n"); cmd_note("STEP BUSY"); break; }
                        if (tc.i0 < 0 || tc.i0 > 1 || tc.i1 < -1000 || tc.i1 > 1000 ||
                            tc.i2 < 100 || tc.i2 > 5000 ||
                            tc.i3 < 1 || tc.i3 > 4) { ack("STEP BAD\r\n"); cmd_note("STEP BAD"); break; }
                        /* 台架安全前置: 关巡线+关自动, 双轮清目标+PID释放+物理刹停 */
                        line_follower_enable(0);
                        line_follower_set_auto(0);
                        for (int i = 0; i < 2; i++) speed_loop_stop(i);
                        g_step_m      = (uint8_t)tc.i0;
                        g_step_rate   = (int16_t)tc.i1;
                        g_step_div    = (uint8_t)tc.i3;
                        g_step_cnt    = 0;
                        g_step_t0     = HAL_GetTick();
                        g_step_end    = g_step_t0 + (uint32_t)tc.i2;
                        g_step_first  = 1;
                        g_step_active = 1;
                        cmd_note("STEP%d %d", g_step_m, g_step_rate);
                        ack("STEP GO %d %d %d %d\r\n", tc.i0, tc.i1, tc.i2, tc.i3);
                        break;
                    }
                    case TXTCMD_TEL:
                        g_tel_on = tc.i0 ? 1 : 0;
                        cmd_note("TEL %d", g_tel_on);
                        ack("TEL:%d\r\n", g_tel_on);
                        break;
                    case TXTCMD_FF:
                        if (tc.i0 < 0 || tc.i0 > 500) { ack("FF BAD\r\n"); cmd_note("FF BAD"); break; }
                        for (int i = 0; i < 2; i++) speed_loop_set_ff_gain(i, tc.i0 / 100.0f);
                        cmd_note("FF %d", tc.i0);
                        ack("FF:%d.%02d\r\n", tc.i0 / 100, tc.i0 % 100);
                        break;
                    case TXTCMD_REC:
                        g_rec_on = tc.i0 ? 1 : 0;
                        if (g_rec_on) g_rec_cnt = 0;   /* 开启即清空缓冲 */
                        cmd_note("REC %d", g_rec_on);
                        ack("REC:%d\r\n", g_rec_on);
                        break;
                    case TXTCMD_DUMP: {
                        /* 重放机内记录: 首行报条数, 末行报结束 — PC 端按条数校验补全
                         * [已知限制] 逐行 20ms 匀速发送 (~1.3s 阻塞主循环) — 爆发式连发会
                         * 触发 BT04 FIFO 溢出丢行 (实测 9:1), 换 DMA 后可移除延时 */
                        g_rec_on = 0;
                        ack("DUMP %d\r\n", g_rec_cnt);
                        for (uint16_t i = 0; i < g_rec_cnt; i++) {
                            char tb[44];
                            int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d,%d,%d\r\n",
                                              (unsigned long)rec_tick[i],
                                              rec_t0[i], rec_r0[i], rec_t1[i], rec_r1[i]);
                            if (tn > 0) tx_raw(tb, tn);
                            HAL_Delay(20);
                        }
                        ack("DUMP END\r\n");
                        cmd_note("DUMP");
                        break;
                    }
                    case TXTCMD_ODOM:
                        g_odom_on = tc.i0 ? 1 : 0;
                        cmd_note("ODOM %d", g_odom_on);
                        ack("ODOM:%d\r\n", g_odom_on);
                        break;
                    case TXTCMD_WD:
                        /* 范围 0~60000ms, 0=关; 使能时刷新喂狗时刻并解除闩锁 */
                        if (tc.i0 < 0 || tc.i0 > 60000) { ack("WD BAD\r\n"); cmd_note("WD BAD"); break; }
                        g_wd_ms = (uint32_t)tc.i0;
                        g_last_rx_tick = HAL_GetTick();
                        g_wd_fired = 0;
                        cmd_note("WD %dms", tc.i0);
                        ack("WD:%d\r\n", tc.i0);
                        break;
                    default:
                        break;
                    }
                } else {
                    /* 识别失败的文本回 '?', 无线调试时确认"收到了但没看懂" */
                    cmd_note("ERR TXT");
                    ack("?\r\n");
                }
            }
            txt_len = 0;
        }

        /* 无线丢包保护: 二进制帧不完整超过 100ms 丢弃, 重新同步
         * (无线链路可能吞掉帧尾 0xFF 0xFF, 不复位会卡住后续解析) */
        if (in_frame && (HAL_GetTick() - t_frame) > 100) { in_frame = 0; f_len = 0; }

        /* ② PID 独立运行 — 每 50ms，不受 OLED 拖累 */
        uint32_t now = HAL_GetTick();
        if (now - t_pid >= 50) {
            t_pid = now;

            /* ═══ 命令看门狗: 超时无任何下行字节 → 急停一次 (闩锁防刷屏) ═══
             * 断链恢复 = 收到任意字节自动解除闩锁 (主循环喂狗处), 无需重新配置 */
            if (g_wd_ms > 0 && !g_wd_fired &&
                (now - g_last_rx_tick) > g_wd_ms) {
                for (int i = 0; i < 2; i++) speed_loop_stop(i);
                line_follower_enable(0);
                line_follower_set_auto(0);
                steering_center();
                g_wd_fired = 1;
                cmd_note("WD TIMEOUT");
                tx_raw("WD TIMEOUT STOP\r\n", 17);
            }

            /* ═══ 调参工具链: 开环阶跃测试 (激活时独占执行链, 20Hz CSV 遥测) ═══
             * 测速复用执行组件 speed_loop_measure()（回绕校正 + 反馈符号与闭环同一份实现，
             * 杜绝"两处测速逻辑靠注释维持同源"的分叉隐患），
             * 遥测行 "t_ms,rate_0E3,rpm_x10" — 与 PC 端辨识程序/VOFA+ 的接口契约,
             * [对未完善部分的假设] 帧格式即 B3 接入契约 v0, 改动须升版本并同步 PC 工具 */
            if (g_step_active) {
                if (now < g_step_end) {
                    (void)motor_bridge_set_rate_0E3(g_step_m, g_step_rate);
                } else {
                    g_step_active = 0;
                    (void)motor_bridge_brake(g_step_m);
                    speed_loop_reset(g_step_m);     /* 释放 PID + 测速基准作废 */
                    ack("STEP END\r\n");
                }
                float step_rpm;
                if (g_step_first) {
                    speed_loop_resync(g_step_m, now);   /* 建基准并丢弃该帧（与原实现一致） */
                    g_step_first = 0;
                } else if (speed_loop_measure(g_step_m, now, &step_rpm) == SPEED_LOOP_OK) {
                    /* 遥测分频发射: 弱链路(如 BLE 桥)20Hz 会系统性丢行, div=2 → 10Hz */
                    if ((++g_step_cnt % g_step_div) == 0) {
                        int rpm10 = spd_to_int(step_rpm * 10.0f);
                        char tb[36];
                        int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d\r\n",
                                          (unsigned long)(now - g_step_t0), (int)g_step_rate, rpm10);
                        if (tn > 0) tx_raw(tb, tn);  /* [已知限制] 阻塞发送 ~16ms@9600, 开环测试可容忍; DMA 化见 P1-6 */
                    }
                }
            }
            /* ═══ 正常闭环控制 (阶跃激活时整段旁路) ═══ */
            else {
            /* ═══ 寻迹控制（启用时把运动意图推给执行组件） ═══ */
            line_follower_update(now, g_lf_tgt, g_lf_dir,
                                 encoder_get_count(&henc1),
                                 encoder_get_count(&henc2));
            if (line_follower_enabled()) {  /* 未启用时巡线不写缓冲 → 保留命令下发的意图 */
                for (uint8_t m = 0; m < 2; m++)
                    speed_loop_set_target(m, g_lf_tgt[m] * (float)g_lf_dir[m]);
            }
            /* ═══ 速度环执行组件: 测速 → PID+前馈 → 限幅 → 下发 ═══
             * "0 速必停"与"内轮停车"语义已在组件内强制（原 main.c 两处停车分支合并） */
            speed_loop_update(now);

            /* ═══ 里程计感知组件: 增量位姿解算 (50ms 与控制环同拍) ═══
             * 恒解算保持位姿新鲜; 仅 g_odom_on 时组帧发送 (ODOM 20Hz + ATT 10Hz)
             * 帧格式见 PV 区注释 — 上位机按"帧头+CMD"分发, 无校验(需求方确认) */
            odom_delta_t od;
            if (odom_update(now, &od) == ODOM_OK && g_odom_on) {
                uint8_t ob[20];
                float   th = od.dtheta_deg;
                ob[0] = 0xAA; ob[1] = ODOM_FRAME_CMD;
                memcpy(&ob[2],  &od.dx_mm, 4);
                memcpy(&ob[6],  &od.dy_mm, 4);
                memcpy(&ob[10], &th,       4);
                memcpy(&ob[14], &now,      4);
                ob[18] = 0xFF; ob[19] = 0xFF;
                tx_raw((const char *)ob, 20);
                if ((++g_att_div & 1) == 0) {   /* ATT 10Hz: 欧拉角透传 (IMU Mahony) */
                    uint8_t ab[20];
                    float r = imu_bridge_get_roll(0);
                    float p = imu_bridge_get_pitch(0);
                    float y = imu_bridge_get_yaw(0);
                    ab[0] = 0xAA; ab[1] = ATT_FRAME_CMD;
                    memcpy(&ab[2],  &r,   4);
                    memcpy(&ab[6],  &p,   4);
                    memcpy(&ab[10], &y,   4);
                    memcpy(&ab[14], &now, 4);
                    ab[18] = 0xFF; ab[19] = 0xFF;
                    tx_raw((const char *)ab, 20);
                }
            }

            /* ═══ 调参工具链: 闭环遥测 (10Hz CSV, 仅 TEL 1 时发射) ═══
             * 行 "tick,tgt0,rpm0,tgt1,rpm1" — B3 接入契约 v0, 同帧快照
             * [已知限制] 阻塞发送 ~21ms@9600, 仅整定会话开启; DMA 化后可提频 */
            if (g_tel_on && (++g_tel_div & 1) == 0) {
                speed_loop_state_t s0, s1;
                speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
                char tb[44];
                int tn = snprintf(tb, sizeof(tb), "%lu,%d,%d,%d,%d\r\n",
                                  (unsigned long)now,
                                  (int)s0.target_rpm, spd_to_int(s0.actual_rpm),
                                  (int)s1.target_rpm, spd_to_int(s1.actual_rpm));
                if (tn > 0) tx_raw(tb, tn);
            }

            /* ═══ 调参工具链: 机内记录 (20Hz 写 RAM, DUMP 重放) — 对抗无线丢行 ═══ */
            if (g_rec_on && g_rec_cnt < REC_MAX) {
                speed_loop_state_t s0, s1;
                speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
                rec_tick[g_rec_cnt] = now;
                rec_t0[g_rec_cnt] = (int16_t)s0.target_rpm;
                rec_r0[g_rec_cnt] = (int16_t)spd_to_int(s0.actual_rpm);
                rec_t1[g_rec_cnt] = (int16_t)s1.target_rpm;
                rec_r1[g_rec_cnt] = (int16_t)spd_to_int(s1.actual_rpm);
                g_rec_cnt++;
            }
            } /* 正常闭环控制 else 段结束 */
        }

        /* ③ 每 100ms 刷新显示 + 传感器
         * [OLED 分页轮转] 每 100ms 只刷 1 页 (8 页 800ms 轮完) — I2C2 异常时每页
         * ~10ms 超时, 8 页连刷曾阻塞主循环 ~850ms/圈 → 控制环掉到 1Hz (2026-09-13
         * 调参实验实测踩坑, 见调试总结 §14 优化方向); 分页后最坏阻塞 ≤1 页 */
        if (now - t_disp >= 100) {
            t_disp = HAL_GetTick();

            /* 读传感器 (IMU 事务带 10ms 超时, 可接受)；时间基准传入（义务 5） */
            (void)imu_bridge_update_filter(0, now);
            float roll  = imu_bridge_get_roll(0);
            float pitch = imu_bridge_get_pitch(0);
            float yaw   = imu_bridge_get_yaw(0);
            int32_t enc1 = encoder_get_count(&henc1);
            int32_t enc2 = encoder_get_count(&henc2);

            /* 拆分 float → int.dec */
            int ri=(int)roll,  rd=(int)((roll -ri)*10.0f); if(rd<0)rd=-rd;
            int pi=(int)pitch, pd=(int)((pitch-pi)*10.0f); if(pd<0)pd=-pd;
            int yi=(int)yaw,   yd=(int)((yaw  -yi)*10.0f); if(yd<0)yd=-yd;
            uint8_t prog = imu_bridge_cal_progress(0);

            /* IMU 校准完成后自动启动寻迹 */
            line_follower_try_auto_start(prog >= 100, now);

            /* OLED 分页轮转: 单页单事务写入, 定宽补齐无残留, 免清屏 */
            char b[26];
            static uint8_t s_disp_page = 0;
            switch (s_disp_page) {
            case 0: /* IMU 欧拉角 */
                snprintf(b,26,"R:%d.%d P:%d.%d Y:%d.%d",ri,rd,pi,pd,yi,yd);
                oled_line(0,b); break;
            case 1: { /* 速度环 实际->目标 (带符号, 由执行组件状态读出) */
                speed_loop_state_t s0, s1;
                speed_loop_get_state(0, &s0); speed_loop_get_state(1, &s1);
                snprintf(b,26,"M0:%d->%d  M1:%d->%d",
                         spd_to_int(s0.actual_rpm), (int)s0.target_rpm,
                         spd_to_int(s1.actual_rpm), (int)s1.target_rpm);
                oled_line(1,b); break;
            }
            case 2: /* 编码器计数（标定转弯/直行距离用） */
                snprintf(b,26,"ENC0:%d  ENC1:%d", (int)enc1, (int)enc2);
                oled_line(2,b); break;
            case 3: { /* 灰度 + IMU 状态 */
                uint8_t g1=HAL_GPIO_ReadPin(OUT1_GPIO_Port,OUT1_Pin);
                uint8_t g2=HAL_GPIO_ReadPin(OUT2_GPIO_Port,OUT2_Pin);
                uint8_t g3=HAL_GPIO_ReadPin(OUT3_GPIO_Port,OUT3_Pin);
                uint8_t g4=HAL_GPIO_ReadPin(OUT4_GPIO_Port,OUT4_Pin);
                uint8_t g5=HAL_GPIO_ReadPin(OUT5_GPIO_Port,OUT5_Pin);
                snprintf(b,26,"G:%d%d%d%d%d  %s",
                         g1?1:0,g2?1:0,g3?1:0,g4?1:0,g5?1:0, (prog<100)?"CAL":"OK");
                oled_line(3,b); break;
            }
            case 4: /* 最近应答 (指令接收后的回复, 无线调试确认) */
                snprintf(b,26,"RSP:%-17s", s_last_resp);
                oled_line(4,b); break;
            case 5: { /* PID 参数 M0/M1 (合并一行, 腾出命令显示页); 真值从组件读, ×100 显示 */
                float kp[2], ki[2], kd[2];
                speed_loop_get_gains(0, &kp[0], &ki[0], &kd[0]);
                speed_loop_get_gains(1, &kp[1], &ki[1], &kd[1]);
                snprintf(b,26,"P%d,%d I%d,%d D%d,%d",
                         spd_to_int(kp[0]*100.0f), spd_to_int(kp[1]*100.0f),
                         spd_to_int(ki[0]*100.0f), spd_to_int(ki[1]*100.0f),
                         spd_to_int(kd[0]*100.0f), spd_to_int(kd[1]*100.0f));
                oled_line(5,b); break;
            }
            case 6: /* 最近执行的命令 (无线命令执行确认) */
                snprintf(b,26,"CMD:%-16s", s_last_cmd);
                oled_line(6,b); break;
            default: /* 页7: 寻迹 + 校准 + 转弯状态 */
                if (prog < 100) {
                    snprintf(b,26,"CAL:%d%%  AUTO:%d", prog, line_follower_auto());
                } else if (!line_follower_enabled()) {
                    snprintf(b,26,"LINE:OFF SPD:%d", (int)line_follower_base_spd());
                } else {
                    const char *st[] = {"FOLLOW","?","TURNING","EXIT","SEARCH"};
                    uint8_t ls = line_follower_state();
                    snprintf(b,26,"LINE %s SPD:%d", st[ls>4?0:ls], (int)line_follower_base_spd());
                }
                oled_line(7,b); break;
            }
            s_disp_page = (s_disp_page + 1) & 7;
        }
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) { } }
