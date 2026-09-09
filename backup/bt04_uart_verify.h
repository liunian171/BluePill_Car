/**
 * @file    bt04_uart.h
 * @brief   BT04 蓝牙串口模块驱动 (USART2, PA2=TX, PA3=RX)
 *
 * @note    接线 (BT04 → BluePill):
 *          BT04_TX  → PA3 (USART2_RX)
 *          BT04_RX  → PA2 (USART2_TX)
 *          VCC      → 3.3V/5V (按模块规格)
 *          GND      → GND
 *
 * @note    USART2 即原调试串口; 验证阶段专用于 BT04,
 *          完整版恢复时命令解析可切换到 BT04 或再分配独立串口。
 */

#ifndef __BT04_UART_H__
#define __BT04_UART_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief BT04 波特率。多数 BT04 模块出厂默认 9600;
 *         若收不到数据, 依次尝试 115200 / 38400 (用 AT 命令查询亦可) */
#ifndef BT04_BAUDRATE
#define BT04_BAUDRATE   9600
#endif

/** @brief 初始化 USART2 (PA2/PA3, 9600-8N1) + 中断接收 + 环形缓冲区 */
void     bt04_uart_init(void);

/** @brief 读取 1 字节 — 主循环调用; 空返回 -1, 有数据返回 0 */
int8_t   bt04_uart_read(uint8_t *byte);

/** @brief 返回缓冲区中未读字节数 */
uint16_t bt04_uart_available(void);

/** @brief 发送原始数据 (阻塞, 100ms 超时) */
void     bt04_uart_send(const uint8_t *data, uint16_t len);

/** @brief 发送字符串 (阻塞, 100ms 超时) */
void     bt04_uart_send_str(const char *str);

/** @brief 累计接收字节数 (开机以来) */
uint32_t bt04_uart_rx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __BT04_UART_H__ */
