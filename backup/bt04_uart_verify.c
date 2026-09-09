/**
 * @file    bt04_uart.c
 * @brief   BT04 蓝牙串口模块驱动 — USART2 (PA2=TX, PA3=RX)
 *
 *  接收链路: RX 中断 → ringbuf → 主循环 bt04_uart_read()
 *  (与原调试串口相同的 "中断 + ringbuf" 模式, 不丢字节)
 *
 *  @note 复用 CubeMX 生成的 huart2 (usart.c), 其 MspInit 自动完成
 *        PA2/PA3 GPIO 配置与 NVIC 使能, 此处只需设置波特率并启动接收。
 */

#include "main.h"
#include "usart.h"
#include "driver/bt04_uart.h"
#include "common/ringbuf.h"

static RingBuffer g_rb_bt;
static uint8_t    s_rx_byte;
static uint32_t   s_rx_count = 0;

/*==============================================================================
 *  初始化
 *==============================================================================*/
void bt04_uart_init(void)
{
    /* huart2 由 CubeMX usart.c 定义; MspInit 负责 PA2/PA3 + NVIC */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = BT04_BAUDRATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    ringbuf_init(&g_rb_bt);
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

/*==============================================================================
 *  收发 API
 *==============================================================================*/
int8_t bt04_uart_read(uint8_t *byte)
{
    return ringbuf_read(&g_rb_bt, byte);
}

uint16_t bt04_uart_available(void)
{
    return ringbuf_num_available(&g_rb_bt);
}

void bt04_uart_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

void bt04_uart_send_str(const char *str)
{
    uint16_t len = 0;
    while (str[len] != '\0') len++;
    bt04_uart_send((const uint8_t *)str, len);
}

uint32_t bt04_uart_rx_count(void)
{
    return s_rx_count;
}

/*==============================================================================
 *  中断回调 (USART2_IRQHandler 由 stm32f1xx_it.c 提供)
 *==============================================================================*/
/* 接收完成回调: 写 ringbuf + 重新挂接收中断 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        ringbuf_write(&g_rb_bt, s_rx_byte);
        s_rx_count++;
        HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}

/* 错误回调: ORE/FE 等错误后重新挂接收, 防止接收链路卡死 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}
