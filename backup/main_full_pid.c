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

/* 文本回应 */
static void ack(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
}

/* 寻迹诊断代理：转发 line_follower 事件到串口 */
static void line_diag(const char *msg) { ack("%s\r\n", msg); }
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
    /* ---- UART ---- */
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
            if (b == 0xAA && !in_frame) { f_len = 0; in_frame = 1; }
            if (in_frame) {
                if (f_len >= sizeof(frame)) { in_frame = 0; continue; }
                frame[f_len++] = b;
                if (f_len >= 2 && frame[f_len-2] == 0xFF && frame[f_len-1] == 0xFF) {
                    uint8_t cmd = frame[1], flen = f_len - 2;
                    if      (cmd == 0x01 && flen >= 7) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); spd_target[id] = (v>0)?v:(-v); spd_dir[id] = (v>=0)?1:-1; ack("M%d:%dRPM\r\n",id,(int)(v+0.5f)); }
                    else if (cmd == 0x02 && flen >= 7) { uint8_t id = frame[2]; float v; memcpy(&v,&frame[3],4); motor_bridge_set_speed_mps(id,v); ack("M%d:%dcm/s\r\n",id,(int)(v*100+0.5f)); }
                    else if (cmd == 0x03 && flen >= 3) { uint8_t id = frame[2]; spd_target[id]=0; pid_reset(&pid_spd[id]); motor_bridge_brake(id); ack("M%d:BRAKE\r\n",id); }
                    else if (cmd == 0xE0 && flen >= 15) { uint8_t id = frame[2]; float kp,ki,kd; memcpy(&kp,&frame[3],4); memcpy(&ki,&frame[7],4); memcpy(&kd,&frame[11],4); pid_set_gains(&pid_spd[id],kp,ki,kd); ack("OK\r\n"); }
                    else if (cmd == 0xF0) { ack("PONG\r\n"); }
                    else { ack("?\r\n"); }
                    in_frame = 0; f_len = 0;
                }
                continue;
            }
            if (b == '\n') txt[txt_len] = 0;
            else if (txt_len < 19) { txt[txt_len++] = b; continue; }
            if (txt_len > 0 && txt[0] == 'P') {
                int id, a, c, d;
                if (sscanf((char*)txt, "P%d %d %d %d", &id, &a, &c, &d) == 4 && id < 2) {
                    pid_kp100[id]=a; pid_ki100[id]=c; pid_kd100[id]=d;
                    pid_set_gains(&pid_spd[id], a*0.01f, c*0.01f, d*0.01f);
                    ack("PID%d:%d %d %d\r\n", id, a, c, d);
                }
            } else if (txt_len > 0 && txt[0] == 'S') {
                int t;
                t=pid_kp100[0]; pid_kp100[0]=pid_kp100[1]; pid_kp100[1]=t;
                t=pid_ki100[0]; pid_ki100[0]=pid_ki100[1]; pid_ki100[1]=t;
                t=pid_kd100[0]; pid_kd100[0]=pid_kd100[1]; pid_kd100[1]=t;
                for (int i=0;i<2;i++) pid_set_gains(&pid_spd[i], pid_kp100[i]*0.01f, pid_ki100[i]*0.01f, pid_kd100[i]*0.01f);
                ack("SWAP:M0 %d %d %d  M1 %d %d %d\r\n",
                    pid_kp100[0],pid_ki100[0],pid_kd100[0],
                    pid_kp100[1],pid_ki100[1],pid_kd100[1]);
            } else if (txt_len > 0 && txt[0] == 'L') {
                int en;
                if (sscanf((char*)txt, "L %d", &en) == 1) {
                    line_follower_enable(en ? 1 : 0);
                    if (!en) { spd_target[0]=0; spd_target[1]=0; }
                    ack("LINE:%s\r\n", en ? "ON" : "OFF");
                }
            } else if (txt_len > 0 && txt[0] == 'L' && txt[1] == 'A') {
                int en;
                if (sscanf((char*)txt, "LA %d", &en) == 1) { line_follower_set_auto(en); ack("LA:%d\r\n", en); }
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'K') {
                float kp;
                if (sscanf((char*)txt, "GK %f", &kp) == 1) { line_follower_set_kp(kp); ack("GK:%.1f\r\n", kp); }
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'D') {
                float kd;
                if (sscanf((char*)txt, "GD %f", &kd) == 1) { line_follower_set_kd(kd); ack("GD:%.1f\r\n", kd); }
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'S') {
                float spd;
                if (sscanf((char*)txt, "GS %f", &spd) == 1) { line_follower_set_speed(spd); ack("GS:%.1f\r\n", spd); }
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'I') {
                line_follower_invert();
                ack("GI:%d\r\n", line_follower_inverted());
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'C') {
                int c;
                if (sscanf((char*)txt, "GC %d", &c) == 1 && c > 0) { line_follower_set_straight_cnt(c); ack("GC:%d\r\n", c); }
            } else if (txt_len > 0 && txt[0] == 'G' && txt[1] == 'T') {
                int c;
                if (sscanf((char*)txt, "GT %d", &c) == 1 && c > 0) { line_follower_set_turn_cnt(c); ack("GT:%d\r\n", c); }
            } else if (txt_len > 0 && txt[0] == 'E') {
                int id, ppr;
                if (sscanf((char*)txt, "E%d %d", &id, &ppr) == 2 && id < 2 && ppr > 0) {
                    if (id == 0) henc1.ppr = (uint16_t)ppr;
                    else         henc2.ppr = (uint16_t)ppr;
                    ack("PPR%d:%d\r\n", id, ppr);
                }
            } else if (txt_len > 0) {
                int a, c, d;
                if (sscanf((char*)txt, "%d %d %d", &a, &c, &d) == 3) {
                    for (int i = 0; i < 2; i++) {
                        pid_kp100[i]=a; pid_ki100[i]=c; pid_kd100[i]=d;
                        pid_set_gains(&pid_spd[i], a*0.01f, c*0.01f, d*0.01f);
                    }
                    ack("PID ALL:%d %d %d\r\n", a, c, d);
                }
            }
            txt_len = 0;
        }

        /* ② PID 独立运行 — 每 50ms，不受 OLED 拖累 */
        uint32_t now = HAL_GetTick();
        if (now - t_pid >= 50) {
            t_pid = now;

            /* ═══ 寻迹控制（启用时覆盖 spd_target[2] / spd_dir[2]） ═══ */
            line_follower_update(now, spd_target, spd_dir,
                                 encoder_get_count(&henc1),
                                 encoder_get_count(&henc2));

            for (uint8_t m = 0; m < 2; m++) {
                if (spd_target[m] < 1.0f) continue;
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

            /* VOFA+: M0_act,M0_tgt,M1_act,M1_tgt,Roll*10,Yaw*10 */
            char u[64];
            int len = snprintf(u,sizeof(u),"channels: %d,%d,%d,%d,%d,%d\n",
                     rpm_disp[0],(int)spd_target[0],rpm_disp[1],(int)spd_target[1],
                     ri*10+(ri>=0?rd:-rd), yi*10+(yi>=0?yd:-yd));
            if(len>0) HAL_UART_Transmit(&huart2, (uint8_t *)u, len, 100);

            /* OLED 刷新（6x8 小字体，8 页 × 21 列） */
            for (int p=0;p<8;p++) oled_bridge_show_string_small(p,0,"                     ");

            char b[22];
            /* 页0: IMU 欧拉角 */
            snprintf(b,22,"R:%d.%d P:%d.%d Y:%d.%d",ri,rd,pi,pd,yi,yd);
            oled_bridge_show_string_small(0,0,b);
            /* 页1: PID 速度 实际->目标 */
            snprintf(b,22,"M0:%d->%d  M1:%d->%d",
                     rpm_disp[0],(int)spd_target[0],rpm_disp[1],(int)spd_target[1]);
            oled_bridge_show_string_small(1,0,b);
            /* 页2: 编码器计数（标定转弯/直行距离用） */
            snprintf(b,22,"ENC0:%d  ENC1:%d",
                     (int)enc1, (int)enc2);
            oled_bridge_show_string_small(2,0,b);
            /* 页3: 灰度 + IMU 状态 */
            uint8_t g1=HAL_GPIO_ReadPin(OUT1_GPIO_Port,OUT1_Pin);
            uint8_t g2=HAL_GPIO_ReadPin(OUT2_GPIO_Port,OUT2_Pin);
            uint8_t g3=HAL_GPIO_ReadPin(OUT3_GPIO_Port,OUT3_Pin);
            uint8_t g4=HAL_GPIO_ReadPin(OUT4_GPIO_Port,OUT4_Pin);
            uint8_t g5=HAL_GPIO_ReadPin(OUT5_GPIO_Port,OUT5_Pin);
            snprintf(b,22,"G:%d%d%d%d%d  %s",
                     g1?1:0,g2?1:0,g3?1:0,g4?1:0,g5?1:0, (prog<100)?"CAL":"OK");
            oled_bridge_show_string_small(3,0,b);
            /* 页4: 方向 */
            snprintf(b,22,"DIR M0:%c M1:%c",
                     spd_dir[0]>0?'+':'-', spd_dir[1]>0?'+':'-');
            oled_bridge_show_string_small(4,0,b);
            /* 页5: PID 参数 M0 */
            snprintf(b,22,"M0 P:%d I:%d D:%d",
                     pid_kp100[0], pid_ki100[0], pid_kd100[0]);
            oled_bridge_show_string_small(5,0,b);
            /* 页6: PID 参数 M1 */
            snprintf(b,22,"M1 P:%d I:%d D:%d",
                     pid_kp100[1], pid_ki100[1], pid_kd100[1]);
            oled_bridge_show_string_small(6,0,b);
            /* 页7: 寻迹 + 校准 + 转弯状态 */
            if (prog < 100) {
                snprintf(b,22,"CAL:%d%%  AUTO:%d", prog, line_follower_auto());
            } else if (!line_follower_enabled()) {
                snprintf(b,22,"LINE:OFF SPD:%d", (int)line_follower_base_spd());
            } else {
                const char *st[] = {"FOLLOW","?","TURNING","EXIT","SEARCH"};
                uint8_t ls = line_follower_state();
                snprintf(b,22,"LINE %s SPD:%d", st[ls>4?0:ls], (int)line_follower_base_spd());
            }
            oled_bridge_show_string_small(7,0,b);
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
