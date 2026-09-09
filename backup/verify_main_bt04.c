/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BT04 蓝牙串口验证版 — OLED 显示接收数据
  ******************************************************************************
  * @note 验证目标:
  *       1. BT04 (USART3 重映射 PC10/PC11, 9600-8N1) 收发链路通断
  *       2. 手机蓝牙串口 APP 发送文本 → OLED 实时显示 + 回发 ACK
  *
  * @note 完整版 (电机/PID/寻迹/IMU/编码器/调试串口) 已备份至
  *       backup/main_full_pid.c, 本验证版确认可用后再恢复接入。
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "gpio.h"
#include "i2c.h"

/* USER CODE BEGIN Includes */
#include "driver/oled_bridge.h"
#include "driver/bt04_uart.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define BT04_LINE_MAX   20   /* 单行最长 20 字符 (OLED 21 列, 6x8 字体) */

static char     s_line_cur[BT04_LINE_MAX + 1];   /* 正在接收的行 (实时) */
static char     s_line_prev[BT04_LINE_MAX + 1];  /* 上一行 (已提交) */
static uint8_t  s_line_idx      = 0;
static uint32_t s_rx_total      = 0;
static uint32_t s_last_rx_tick  = 0;
static uint8_t  s_ack_pending   = 0;   /* 1=有新行待回 ACK (只回一次) */
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* 行提交: 当前行移入上一行, 清空当前行 */
static void line_commit(void)
{
    if (s_line_idx > 0)
    {
        strcpy(s_line_prev, s_line_cur);
        s_line_idx    = 0;
        s_line_cur[0] = '\0';
        s_ack_pending = 1;
    }
}
/* USER CODE END 0 */

int main(void)
{

  HAL_Init();
    SystemClock_Config();

    /* 外设初始化 (验证版仅保留 GPIO / I2C2-OLED / USART3-BT04) */
    MX_GPIO_Init();
    MX_I2C2_Init();

    /* USER CODE BEGIN 2 */
    /* ---- 板载 LED PC13 心跳 (低电平点亮) ---- */
    GPIO_InitTypeDef led = {0};
    led.Pin   = GPIO_PIN_13;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* 初始灭 */

    /* ---- OLED ---- */
    oled_bridge_init();
    oled_bridge_show_string_small(0, 0, "BT04 TEST 9600-8N1");
    oled_bridge_show_string_small(1, 0, "TX:PA2   RX:PA3");
    oled_bridge_show_string_small(2, 0, "WAIT PHONE CONNECT");
    oled_bridge_show_string_small(3, 0, "...");

    /* ---- BT04 蓝牙串口 ---- */
    bt04_uart_init();
    bt04_uart_send_str("BT04 Ready\r\n");
    /* USER CODE END 2 */

    /* ---- 主循环 ---- */
    uint32_t t_disp = 0;
    uint32_t t_led  = 0;

    while (1)
    {
        /* ① 逐字节读 BT04 环形缓冲区, 拼行 (遇 \n 提交, 超长自动提交) */
        uint8_t b;
        while (bt04_uart_read(&b) == 0)
        {
            s_rx_total++;
            s_last_rx_tick = HAL_GetTick();
            if (b == '\r') continue;
            if (b == '\n') { line_commit(); continue; }
            if (b >= 0x20 && b < 0x7F)
            {
                if (s_line_idx >= BT04_LINE_MAX) line_commit();
                s_line_cur[s_line_idx++] = (char)b;
                s_line_cur[s_line_idx]   = '\0';
            }
        }

        uint32_t now = HAL_GetTick();

        /* ② LED 心跳 500ms */
        if (now - t_led >= 500)
        {
            t_led = now;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        /* ③ 每 100ms 刷新 OLED (6x8 小字体, 8 页 x 21 列) */
        if (now - t_disp >= 100)
        {
            t_disp = now;

            for (int p = 0; p < 8; p++) oled_bridge_show_string_small(p, 0, "                     ");

            char buf[24];
            /* 页0: 标题 + 累计接收字节数 */
            snprintf(buf, 22, "BT04 RX:%-10lu", (unsigned long)s_rx_total);
            oled_bridge_show_string_small(0, 0, buf);
            /* 页1: 缓冲区实时内容 (当前正在接收的行) */
            snprintf(buf, 22, "CUR :%-16s", s_line_cur);
            oled_bridge_show_string_small(1, 0, buf);
            /* 页2: 上一行 */
            snprintf(buf, 22, "PREV:%-15s", s_line_prev);
            oled_bridge_show_string_small(2, 0, buf);
            /* 页3: 收到数据距现在的时间 / 未收到提示 */
            if (s_rx_total == 0)
            {
                oled_bridge_show_string_small(3, 0, "NO DATA YET...");
            }
            else
            {
                snprintf(buf, 22, "LAST RX:%lus", (unsigned long)((now - s_last_rx_tick) / 1000));
                oled_bridge_show_string_small(3, 0, buf);
            }
            /* 页4: 回发 ACK (每收到一行只回一次, 不重复) */
            if (s_ack_pending)
            {
                s_ack_pending = 0;
                bt04_uart_send_str("ACK:");
                bt04_uart_send_str(s_line_prev);
                bt04_uart_send_str("\r\n");
            }
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
