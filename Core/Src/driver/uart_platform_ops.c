/**
 * @file    uart_platform_ops.c
 * @brief   STM32 平台 UART 操作实现
 *
 * 基于 STM32 HAL 库实现 UART_PlatformOps_t 的三个函数指针。
 *
 * 前置条件:
 *   · stm32f4xx_hal_conf.h 启用 HAL_UART_MODULE_ENABLED        ✅
 *   · CubeMX 配置 UART7 (Asynchronous, PE7=RX, PE8=TX, 115200 8N1) ✅
 *   · CubeMX NVIC 使能 UART7 global interrupt                  ✅
 *
 * 参考: pwm_platform_ops.c, UART_Serial_Design.md 第三章
 */

#include "uart_platform_ops.h"
#include "usart.h"          /* 提供 huart7 的 extern 声明（CubeMX 生成） */

/* 阻塞收发超时，当前固定 1000ms，如需可配则改为读 hUART->timeout_ms */
#define UART_TIMEOUT_MS  1000

/* ================================================================
 *  uart_stm32_send
 *  阻塞发送 len 字节。
 *  内部调用 HAL_UART_Transmit(huart, data, len, 1000ms)。
 *  HCLK=180MHz 下硬件自动串并转换，不占用 CPU 运算时间。
 *
 *  @retval 0   HAL_OK（发送完成）
 *  @retval -1  HAL_ERROR/HAL_BUSY/HAL_TIMEOUT（超时或错误）
 * ================================================================ */
int8_t uart_stm32_send(void *huart_void, const uint8_t *data, uint16_t len)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)huart_void;
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, data, len, UART_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : -1;
}

/* ================================================================
 *  uart_stm32_receive
 *  阻塞接收 len 字节。
 *  内部调用 HAL_UART_Receive(huart, data, len, 1000ms)。
 *  仅调试用，正常接收走中断版本 (receive_IT)，避免阻塞主循环。
 *
 *  @retval 0   HAL_OK
 *  @retval -1  HAL_ERROR/HAL_BUSY/HAL_TIMEOUT
 * ================================================================ */
int8_t uart_stm32_receive(void *huart_void, uint8_t *data, uint16_t len)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)huart_void;
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, len, UART_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : -1;
}

/* ================================================================
 *  uart_stm32_receive_IT
 *  启动单字节中断接收，非阻塞。
 *  调用 HAL_UART_Receive_IT(huart, p_byte, 1)。
 *
 *  中断链:
 *    RXNE 硬件中断 → UART7_IRQHandler → HAL_UART_IRQHandler
 *      → HAL_UART_RxCpltCallback（在 uart_cmd_parser.c 中实现）
 *         → ringbuf_write 存字节
 *         → 重新调用本函数使能下一个字节接收
 *
 *  注意: HAL_UART_Receive_IT 每次调用只注册一个字节的接收，
 *        收到后自动关闭，需要回调中重新调用才能连续接收。
 *
 *  @retval 0   HAL_OK
 *  @retval -1  HAL_ERROR/HAL_BUSY
 * ================================================================ */
int8_t uart_stm32_receive_IT(void *huart_void, uint8_t *p_byte, uint16_t len)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)huart_void;
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart, p_byte, len);
    return (status == HAL_OK) ? 0 : -1;
}

/* ================================================================
 *  uart_platform_ops_stm32 操作表实例
 *
 *  将三个平台函数打包成 UART_PlatformOps_t 结构体，
 *  main.c 中通过 uart_debug.ops = &uart_platform_ops_stm32 注入。
 *  换芯片时只需替换这个变量指向的实例即可。
 * ================================================================ */
const UART_PlatformOps_t uart_platform_ops_stm32 = {
    .send       = uart_stm32_send,
    .receive    = uart_stm32_receive,
    .receive_IT = uart_stm32_receive_IT,
};
