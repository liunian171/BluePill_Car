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
#include "driver/imu_bridge.h"
#include "driver/i2c_hardware_ops.h"
#include "driver/line_follower.h"
#include "driver/txt_cmd.h"
#include "driver/servo_bridge.h"
#include "driver/steering.h"
#include "common/ringbuf.h"
#include "common/pid.h"
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

/* PID 速度闭环 */
static PID_Handle   pid_spd[2];
static float        spd_target[2]   = {0, 0};
static int8_t       spd_dir[2]      = {1, 1};
static int32_t      spd_prev_enc[2] = {0, 0};
static uint32_t     spd_prev_tick[2]= {0, 0};
static int          pid_kp100[2]    = {24, 24}; /* Kp×100 */
static int          pid_ki100[2]    = {13, 13};
static int          pid_kd100[2]    = {20, 20};
static int16_t      rpm_disp[2]     = {0, 0};
static char         s_last_cmd[20]  = "NONE";  /* 最近执行的命令 (OLED 页6 显示) */
static char         s_last_resp[20] = "-";     /* 最近应答/回复 (OLED 页4 显示) */
static uint32_t     t_frame         = 0;       /* 二进制帧最近字节时间戳 (超时重同步) */
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* VOFA+ FireWater: 发送 N 个 float（自动转大端+加校验） */
static void firewater_send(float *data, uint8_t n)
{
    uint8_t buf[128];
    uint8_t len = 1 + n * 4;  /* 类型ID + n个float */
    buf[0] = 0x55; buf[1] = 0x55;  /* 帧头 */
    buf[2] = len;                    /* 长度 */
    buf[3] = 0x01;                   /* float类型 */
    for (uint8_t i = 0; i < n; i++) {
        uint32_t v;
        memcpy(&v, &data[i], 4);
        /* 小端→大端 */
        buf[4+i*4+0] = (uint8_t)(v >> 24);
        buf[4+i*4+1] = (uint8_t)(v >> 16);
        buf[4+i*4+2] = (uint8_t)(v >> 8);
        buf[4+i*4+3] = (uint8_t)(v);
    }
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 2 + len; i++) sum += buf[i];
    buf[4 + n*4] = sum;
    HAL_UART_Transmit(&huart2, buf, 5 + n*4, 100);
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
    servo_bridge_set_angle(id, phys_angle);
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

    /* ---- 电机桥 ---- */
    motor_bridge_init(0, &pwm_tim1_ch1, &motor_a_in1, &motor_a_in2, NULL, 319.0f, 32.5f);
    motor_bridge_init(1, &pwm_tim1_ch2, &motor_b_in1, &motor_b_in2, NULL, 319.0f, 32.5f);

    /* ---- PID 速度闭环 ---- */
    PID_Params_t spd_param = { .kp=0.24f, .ki=0.13f, .kd=0.0f,
                               .out_min=-319.0f, .out_max=319.0f, .integral_limit=60.0f };
    pid_init(&pid_spd[0], PID_MODE_INCREMENTAL, &spd_param);
    pid_init(&pid_spd[1], PID_MODE_INCREMENTAL, &spd_param);

    /* ---- 舵机 50Hz ---- */
    pwm_set_freq(&pwm_tim4_ch3, 50); pwm_start(&pwm_tim4_ch3);

    /* ---- 舵机桥 (PB8) + 转向执行组件: 上电回直行位 = 安全铁律 ---- */
    servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim4_ch3);
    servo_bridge_start(0);
    if (steering_init(&g_steering_cfg, steering_output_to_servo) != STEERING_OK)
        Error_Handler();
    steering_center();

    /* ---- OLED ---- */
    oled_bridge_init();
    oled_bridge_show_string_small(0,0,"BLUEPILL PID OK");
    oled_bridge_show_string_small(2,0,"Boot...");

    /* ---- 寻迹 ---- */
    line_follower_init(30.0f, 10.0f);
    line_follower_set_event_cb(line_diag);

    /* ---- IMU ---- */
    imu_bridge_init(0, IMU_MPU6050, &imu_i2c);
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
            if (b == 0xAA && !in_frame) { f_len = 0; in_frame = 1; t_frame = HAL_GetTick(); }
            if (in_frame) {
                if (f_len >= sizeof(frame)) { in_frame = 0; continue; }
                frame[f_len++] = b;
                t_frame = HAL_GetTick();
                if (f_len >= 2 && frame[f_len-2] == 0xFF && frame[f_len-1] == 0xFF) {
                    uint8_t cmd = frame[1], flen = f_len - 2;
                    if      (cmd == 0x01 && flen >= 7 && frame[2] < 2) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); line_follower_enable(0); spd_target[id] = (v>0)?v:(-v); spd_dir[id] = (v>=0)?1:-1; cmd_note("M%d=%dRPM", id, (int)(v>0?v:-v)); ack("M%d:%dRPM\r\n",id,(int)(v+0.5f)); }
                    else if (cmd == 0x02 && flen >= 7 && frame[2] < 2) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); line_follower_enable(0); motor_bridge_set_speed_mps(id,v); cmd_note("M%d=%dcm/s", id, (int)(v*100)); ack("M%d:%dcm/s\r\n",id,(int)(v*100+0.5f)); }
                    else if (cmd == 0x03 && flen >= 3 && frame[2] < 2) { uint8_t id = frame[2]; line_follower_enable(0); spd_target[id]=0; pid_reset(&pid_spd[id]); motor_bridge_brake(id); cmd_note("BRK%d", id); ack("M%d:BRAKE\r\n",id); }
                    else if (cmd == 0x10 && flen >= 7 && frame[2] < 1) { float v; memcpy(&v,&frame[3],4); steering_set(v); float a = steering_get(); cmd_note("SV=%d.%d", (int)a, dec1(a)); ack("SV:%d.%d\r\n", (int)a, dec1(a)); }
                    else if (cmd == 0xE0 && flen >= 15 && frame[2] < 2) { uint8_t id = frame[2]; float kp,ki,kd; memcpy(&kp,&frame[3],4); memcpy(&ki,&frame[7],4); memcpy(&kd,&frame[11],4); pid_set_gains(&pid_spd[id],kp,ki,kd); cmd_note("PID%d", id); ack("OK\r\n"); }
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
                        for (int i = 0; i < 2; i++) { spd_target[i]=0; pid_reset(&pid_spd[i]); motor_bridge_brake(i); }
                        /* 急停语义 = 彻底停: 必须同步关巡线+自动启动,
                         * 否则 50ms 后状态机重写 spd_target, 车会"复活"(实测踩坑) */
                        line_follower_enable(0);
                        line_follower_set_auto(0);
                        steering_center();  /* 前轮回直行位 (实测 -90, 非 0) */
                        cmd_note("STOP ALL");
                        ack("STOP OK LINE OFF\r\n");
                        break;
                    case TXTCMD_MOTOR: {
                        int id = tc.i0;
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先: 退出巡线接管, 否则 50ms 后被覆盖 */
                        spd_target[id] = (v >= 0) ? v : -v;
                        spd_dir[id]    = (v >= 0) ? 1 : -1;
                        cmd_note("M%d=%dRPM", id, (int)((v >= 0) ? v : -v));
                        ack("M%d:%dRPM\r\n", id, (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_MOTOR_BOTH: {
                        float v = tc.f0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        spd_target[0] = spd_target[1] = (v >= 0) ? v : -v;
                        spd_dir[0]    = spd_dir[1]    = (v >= 0) ? 1 : -1;
                        cmd_note("MS=%dRPM", (int)((v >= 0) ? v : -v));
                        ack("MS:%dRPM\r\n", (int)(v + ((v < 0) ? -0.5f : 0.5f)));
                        break;
                    }
                    case TXTCMD_BRAKE: {
                        int id = tc.i0;
                        line_follower_enable(0);  /* 手动指令优先 */
                        spd_target[id] = 0; pid_reset(&pid_spd[id]); motor_bridge_brake(id);
                        cmd_note("BRK%d", id);
                        ack("M%d:BRAKE\r\n", id);
                        break;
                    }
                    case TXTCMD_PID_SET: {
                        int lo = (tc.i0 < 0) ? 0 : tc.i0, hi = (tc.i0 < 0) ? 1 : tc.i0;
                        for (int i = lo; i <= hi; i++) {
                            pid_kp100[i] = tc.i1; pid_ki100[i] = tc.i2; pid_kd100[i] = tc.i3;
                            pid_set_gains(&pid_spd[i], tc.i1*0.01f, tc.i2*0.01f, tc.i3*0.01f);
                        }
                        if (tc.i0 < 0) ack("PID ALL:%d %d %d\r\n", tc.i1, tc.i2, tc.i3);
                        else           ack("PID%d:%d %d %d\r\n", tc.i0, tc.i1, tc.i2, tc.i3);
                        cmd_note("PID SET");
                        break;
                    }
                    case TXTCMD_PID_SWAP: {
                        int t;
                        t=pid_kp100[0]; pid_kp100[0]=pid_kp100[1]; pid_kp100[1]=t;
                        t=pid_ki100[0]; pid_ki100[0]=pid_ki100[1]; pid_ki100[1]=t;
                        t=pid_kd100[0]; pid_kd100[0]=pid_kd100[1]; pid_kd100[1]=t;
                        for (int i=0;i<2;i++) pid_set_gains(&pid_spd[i], pid_kp100[i]*0.01f, pid_ki100[i]*0.01f, pid_kd100[i]*0.01f);
                        cmd_note("PID SWAP");
                        ack("SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
                            pid_kp100[0],pid_ki100[0],pid_kd100[0],
                            pid_kp100[1],pid_ki100[1],pid_kd100[1]);
                        break;
                    }
                    case TXTCMD_LINE_EN: {
                        int en = tc.i0 ? 1 : 0;
                        line_follower_enable(en);
                        if (!en) { spd_target[0]=0; spd_target[1]=0; }
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

            /* ═══ 寻迹控制（启用时覆盖 spd_target[2] / spd_dir[2]） ═══ */
            line_follower_update(now, spd_target, spd_dir,
                                 encoder_get_count(&henc1),
                                 encoder_get_count(&henc2));

            for (uint8_t m = 0; m < 2; m++) {
                if (spd_target[m] < 1.0f) {
                    /* 0速语义 = 停车: 释放PID + 物理刹停 (否则PWM保持旧值, 无线发 M0 0 车不停) */
                    pid_reset(&pid_spd[m]);
                    motor_bridge_brake(m);
                    rpm_disp[m] = 0;
                    continue;
                }
                /* 内侧轮停车：冻结 PID + 物理刹停，退出转弯时从零起步 */
                if (spd_dir[m] == 0) {
                    pid_reset(&pid_spd[m]);
                    motor_bridge_brake(m);  /* 必须物理刹停，否则 PWM 保持旧值电机不停 */
                    spd_prev_enc[m] = (m == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
                    spd_prev_tick[m] = now;
                    rpm_disp[m] = 0;
                    continue;
                }
                int32_t enc = (m == 0) ? encoder_get_count(&henc1) : encoder_get_count(&henc2);
                float dt = (float)(now - spd_prev_tick[m]) * 0.001f;
                if (dt < 0.01f || dt > 2.0f) { spd_prev_tick[m] = now; spd_prev_enc[m] = enc; continue; }

                int32_t delta = (int32_t)enc - (int32_t)spd_prev_enc[m];
                if (delta >  30000) delta -= 65536;
                if (delta < -30000) delta += 65536;
                int32_t abs_delta = (delta < 0) ? -delta : delta;
                uint16_t ppr = (m == 0) ? henc1.ppr : henc2.ppr;
                float actual_rpm = (float)abs_delta * 60.0f / (dt * (float)ppr);

                /* 前馈: 目标转速直接作为基础输出，PID 只做修正 */
                float ff = spd_target[m] * 0.3f;  /* 前馈系数 0.3 */
                pid_spd[m].params.out_min = -319.0f;
                pid_spd[m].params.out_max = 319.0f;
                float pid_out = pid_update(&pid_spd[m], spd_target[m], actual_rpm, dt) + ff;
                if (pid_out < 0.0f) pid_out = 0.0f;
                if (pid_out > 319.0f) pid_out = 319.0f;

                motor_bridge_set_speed_rpm(m, pid_out * spd_dir[m]);
                rpm_disp[m] = (int16_t)(actual_rpm + 0.5f);
                spd_prev_enc[m] = enc; spd_prev_tick[m] = now;
            }
        }

        /* ③ 每 100ms 刷新显示 + 传感器 */
        if (now - t_disp >= 100) {
            t_disp = HAL_GetTick();

            /* 读传感器 */
            imu_bridge_update_filter(0);
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

            /* OLED 刷新（6x8 小字体, 整行单事务写入; 定宽补齐无残留, 免清屏） */
            char b[26];
            /* 页0: IMU 欧拉角 */
            snprintf(b,26,"R:%d.%d P:%d.%d Y:%d.%d",ri,rd,pi,pd,yi,yd);
            oled_line(0,b);
            /* 页1: PID 速度 实际->目标 */
            snprintf(b,26,"M0:%d->%d  M1:%d->%d",
                     rpm_disp[0],(int)spd_target[0],rpm_disp[1],(int)spd_target[1]);
            oled_line(1,b);
            /* 页2: 编码器计数（标定转弯/直行距离用） */
            snprintf(b,26,"ENC0:%d  ENC1:%d",
                     (int)enc1, (int)enc2);
            oled_line(2,b);
            /* 页3: 灰度 + IMU 状态 */
            uint8_t g1=HAL_GPIO_ReadPin(OUT1_GPIO_Port,OUT1_Pin);
            uint8_t g2=HAL_GPIO_ReadPin(OUT2_GPIO_Port,OUT2_Pin);
            uint8_t g3=HAL_GPIO_ReadPin(OUT3_GPIO_Port,OUT3_Pin);
            uint8_t g4=HAL_GPIO_ReadPin(OUT4_GPIO_Port,OUT4_Pin);
            uint8_t g5=HAL_GPIO_ReadPin(OUT5_GPIO_Port,OUT5_Pin);
            snprintf(b,26,"G:%d%d%d%d%d  %s",
                     g1?1:0,g2?1:0,g3?1:0,g4?1:0,g5?1:0, (prog<100)?"CAL":"OK");
            oled_line(3,b);
            /* 页4: 最近应答 (指令接收后的回复, 无线调试确认) */
            snprintf(b,26,"RSP:%-17s", s_last_resp);
            oled_line(4,b);
            /* 页5: PID 参数 M0/M1 (合并一行, 腾出命令显示页) */
            snprintf(b,26,"P%d,%d I%d,%d D%d,%d",
                     pid_kp100[0], pid_kp100[1],
                     pid_ki100[0], pid_ki100[1],
                     pid_kd100[0], pid_kd100[1]);
            oled_line(5,b);
            /* 页6: 最近执行的命令 (无线命令执行确认) */
            snprintf(b,26,"CMD:%-16s", s_last_cmd);
            oled_line(6,b);
            /* 页7: 寻迹 + 校准 + 转弯状态 */
            if (prog < 100) {
                snprintf(b,26,"CAL:%d%%  AUTO:%d", prog, line_follower_auto());
            } else if (!line_follower_enabled()) {
                snprintf(b,26,"LINE:OFF SPD:%d", (int)line_follower_base_spd());
            } else {
                const char *st[] = {"FOLLOW","?","TURNING","EXIT","SEARCH"};
                uint8_t ls = line_follower_state();
                snprintf(b,26,"LINE %s SPD:%d", st[ls>4?0:ls], (int)line_follower_base_spd());
            }
            oled_line(7,b);
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
