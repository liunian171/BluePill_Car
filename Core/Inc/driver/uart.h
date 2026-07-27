/**
 * ============================================================================
 *  UART 驱动 — 抽象层（与具体芯片无关）
 * ============================================================================
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │  架构总览（三层分离）                                            │
 *  │                                                                 │
 *  │  应用层                                                            │
 *  │    uart_send(hUART, data, len)  /  uart_receive_IT(hUART, ...)  │
 *  │                    │                                            │
 *  │                    ▼                                            │
 *  │  ┌─ uart.c ──────────────────────────────────────────────────┐  │
 *  │  │  策略层（跨平台通用，与芯片无关）                           │  │
 *  │  │  职责：组合调用 ops + 参数转发                             │  │
 *  │  │  换平台时：此文件零修改                                    │  │
 *  │  └──────────────────────┬──────────────────────────────────┘  │
 *  │                         │ hUART->ops->xxx()                   │
 *  │                         ▼                                      │
 *  │  ┌─ uart_platform_ops.c ───────────────────────────────────┐  │
 *  │  │  平台实现层（每款芯片一套）                               │  │
 *  │  │  职责：直接调用 HAL 库操作硬件寄存器                      │  │
 *  │  │  换平台时：新增一套 .c/.h，接口签名不变                   │  │
 *  │  └──────────────────────────────────────────────────────────┘  │
 *  └─────────────────────────────────────────────────────────────────┘
 *
 *  与现有 PWM/GPIO 驱动的区别：
 *    · PWM/GPIO 是"配置型"——写完寄存器硬件自己跑
 *    · UART 接收是"事件驱动型"——必须用中断接收，否则阻塞主循环
 *
 *  核心解耦手段：
 *    1. void *huart     — 隐藏具体芯片的句柄类型
 *    2. 操作表 ops       — 函数指针集合，多态的实现方式
 *    3. UART_Handle 句柄 — 串联 huart + ops + rx_byte 的统一结构
 *
 *  参考: UART_Serial_Design.md 第三章
 * ============================================================================
 */

#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 *  平台操作表类型定义 — 接口抽象核心
 *
 *  用途：定义一组与硬件交互的"原语操作"，每个平台各自实现一套
 *  所有函数指针的第一个参数统一为 void *huart，不暴露具体芯片类型
 *
 *  添加新操作时需注意：
 *    · 操作必须是与硬件寄存器打交道的"原子操作"
 *    · 组合逻辑（如 flush 的实现）应放在 uart.c 的策略层，而非 ops 内
 *==============================================================================*/
typedef struct UART_PlatformOps_t
{
    /* 阻塞发送 len 字节，返回 0 成功 / -1 失败 */
    int8_t (*send)(void *huart, const uint8_t *data, uint16_t len);

    /* 阻塞接收 len 字节，一般仅调试用 */
    int8_t (*receive)(void *huart, uint8_t *data, uint16_t len);

    /* 启动中断接收，单字节到达后触发 HAL_UART_RxCpltCallback */
    int8_t (*receive_IT)(void *huart, uint8_t *p_byte, uint16_t len);

} UART_PlatformOps_t;


/*==============================================================================
 *  UART 句柄 — 所有操作的中心数据结构
 *  应用层通过此结构体与驱动交互
 *==============================================================================*/
typedef struct UART_Handle
{
    void                      *huart;    /* 平台句柄（STM32: UART_HandleTypeDef*）
                                          *   换 MCU 后此字段对应的类型由平台层自行强转 */

    const UART_PlatformOps_t  *ops;      /* 当前平台的操作表指针 */

    uint8_t    rx_byte;                 /* 中断接收临时落脚点
                                          * HAL 收到字节后写入此处，回调中再转存到
                                          * ringbuf，然后重新使能下一个字节的接收 */

} UART_Handle;


/*==============================================================================
 *  策略层函数声明（实现在 uart.c）
 *  这些函数通过 hUART->ops 调用底层硬件操作，自身只做转发
 *==============================================================================*/

/**
 * @brief 阻塞发送 len 字节
 * @param hUART  UART 句柄
 * @param data   待发送数据缓冲区
 * @param len    发送字节数
 * @retval 0 成功  -1 超时/HAL 错误
 */
int8_t uart_send(UART_Handle *hUART, const uint8_t *data, uint16_t len);

/**
 * @brief 阻塞接收 len 字节（仅调试用，正常接收走中断模式）
 * @note  会阻塞主循环直到收满 len 字节或超时，不要在产品代码中用
 * @retval 0 成功  -1 超时/HAL 错误
 */
int8_t uart_receive(UART_Handle *hUART, uint8_t *data, uint16_t len);

/**
 * @brief 启动中断接收 — 每收到 1 字节触发 RxCpltCallback
 * @param hUART  UART 句柄
 * @param p_byte 存放收到字节的地址（指向 hUART->rx_byte）
 * @note  回调中必须重新调用本函数使能下一个字节，否则只收到一个。
 *        接收到的字节走 HAL_UART_RxCpltCallback → ringbuf_write →
 *        uart_cmd_parser_tick 路径，最终在 cmd_dispatch 中消费。
 * @retval 0 成功  -1 HAL 错误
 */
int8_t uart_receive_IT(UART_Handle *hUART, uint8_t *p_byte);

/**
 * @brief 清空 RX 缓冲区，丢弃未读数据（当前占位，待补）
 * @retval 0 成功
 */
int8_t uart_flush_rx(UART_Handle *hUART);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
