/**
 * @file    uart_cmd_parser.h
 * @brief   串口命令解析层 — PC 与 STM32 共享的命令/状态枚举定义 + 解析器接口
 *
 * 协议说明:
 *   ┌─ 命令帧（PC → STM32）─────┐
 *   │ 0xAA | cmd | [id] | [参数] | 0xFF | 0xFF │
 *   │  帧头    命令  可选id   参数   两字节帧尾    │
 *   └──────────────────────────────────────────┘
 *
 *   ┌─ 响应帧（STM32 → PC）─────┐
 *   │ 0xAA | cmd | id | status | 0xFF | 0xFF │
 *   └─────────────────────────────────────────┘
 *
 *   帧头帧尾两端统一：
 *     · 帧头 = 0xAA
 *     · 帧尾 = 0xFF 0xFF（两字节，减少参数误触发）
 *   所有字段都是枚举/二进制值，无 ASCII 字符串。
 *   PC 侧在需要显示时查映射表转换为文字。
 *
 * 参考: UART_Serial_Design.md 第五章
 */

#ifndef __UART_CMD_PARSER_H__
#define __UART_CMD_PARSER_H__

#include <stdint.h>
#include "uart.h"
#include "ringbuf.h"

/* ================================================================
 *  命令枚举（PC → STM32）
 *  一个 byte 表达一条完整指令，semantic density 最大化
 * ================================================================ */
typedef enum {
    CMD_NOP              = 0x00,

    /* ---- Motor 命令组（0x01 ~ 0x0F） ---- */
    CMD_MOTOR_SET_RPM    = 0x01,   /* 参数: id(1B) + rpm(float,4B) */
    CMD_MOTOR_SET_MPS    = 0x02,   /* 参数: id(1B) + mps(float,4B) */
    CMD_MOTOR_BRAKE      = 0x03,   /* 参数: id(1B) */
    CMD_MOTOR_SET_DEADZ  = 0x04,   /* 参数: id(1B) + rpm(float,4B) */

    /* ---- Servo 命令组（0x10 ~ 0x1F） ---- */
    CMD_SERVO_SET_ANGLE  = 0x10,   /* 参数: id(1B) + angle(float,4B) */
    CMD_SERVO_START      = 0x11,   /* 参数: id(1B) */
    CMD_SERVO_STOP       = 0x12,   /* 参数: id(1B) */

    /* ---- PWM 命令组（0x20 ~ 0x2F） ---- */
    CMD_PWM_SET_DUTY     = 0x20,   /* 参数: id(1B) + duty(uint16,2B) */

    /* ---- IMU 命令组（0x30 ~ 0x3F） ---- */
    CMD_IMU_GET_DATA     = 0x30,   /* 参数: id(1B) → 响应: roll/pitch/yaw (3×float)*/
    CMD_IMU_CALIBRATE    = 0x31,   /* 参数: id(1B) + samples(uint16,2B)*/

    /* ---- 系统命令（0xF0 ~ 0xFF） ---- */
    CMD_PING             = 0xF0,   /* 心跳检测，无参数 */
    CMD_GET_STATUS       = 0xF1,   /* 获取状态，无参数 */
} CmdCode_t;

/* ================================================================
 *  状态枚举（STM32 → PC）
 * ================================================================ */
typedef enum {
    STATUS_OK            = 0xA0,
    STATUS_ERROR         = 0xA1,
    STATUS_INVALID_CMD   = 0xA2,
    STATUS_INVALID_ID    = 0xA3,
    STATUS_INVALID_PARAM = 0xA4,
    STATUS_BUSY          = 0xA5,
} StatusCode_t;

/* ================================================================
 *  帧常量
 *
 *  当前为单例模式（一个串口），ringbuf/tick 直接全局 static。
 *  多串口时：每个实例独立一套 RingBuffer + rx_byte + tick()
 *
 *  帧长度说明（含帧头尾）:
 *    最短帧（无参数） : SOF + cmd +      + EOF2_1 + EOF2_2 = 5B  (PING/NOP)
 *    最短帧（有 id ） : SOF + cmd + id   + EOF2_1 + EOF2_2 = 5B  (Brake/Start/Stop)
 *    int16/uint16 参数: SOF + cmd + id   + 2B       + EOF2    = 7B
 *    float 参数       : SOF + cmd + id   + 4B       + EOF2    = 9B
 *   FRAME_RX_BUF_SIZE = 32B 可容纳所有合法帧，且为溢出保护留足余量
 * ================================================================ */
#define FRAME_SOF         0xAA   /* 帧头 (Start of Frame)                        */
#define FRAME_EOF2_1      0xFF   /* 两字节帧尾第 1 字节                           */
#define FRAME_EOF2_2      0xFF   /* 两字节帧尾第 2 字节（双字节减少参数误触发）  */
#define FRAME_RX_BUF_SIZE 32     /* 接收帧缓冲区容量                              */

/* ================================================================
 *  解析器 API
 *
 *  命名约定:
 *    uart_cmd_parser_xxx — 属于命令解析层的公开 API
 *    uart_cmd_yyy       — 命令解析层内部函数（static，不暴露）
 *    CMD_xxx            — 上位机发来的命令码（输入）
 *    STATUS_xxx         — 下位机返回的状态码（响应）
 *    FRAME_xxx          — 帧协议常量
 *
 *  典型调用流程（main 中）:
 *    uart_cmd_parser_init();          // ① 初始化（一次）
 *    while (1) {
 *        uart_cmd_parser_tick();      // ② 每轮：处理收到的字节
 *    }
 *
 *  底层自动触发的回调链:
 *    HAL_UART_RxCpltCallback          // ③ 硬件中断：字节存入 ringbuf
 *        → ringbuf_write              //    回调在 uart_cmd_parser.c 实现
 *        → receive_IT                 //    重新使能下一个字节接收
 * ================================================================ */

/**
 * @brief 初始化串口命令解析器
 * @param hUART  已配置的 UART 句柄
 * @param ringbuf 已初始化的环形缓冲区
 */
void uart_cmd_parser_init(UART_Handle *hUART, RingBuffer *ringbuf);

/**
 * @brief 主循环 tick — 每次调用处理环形缓冲区中所有积压的字节
 *
 * 内部逻辑:
 *   ① 从环形缓冲区取出一个字节
 *   ② 如果收到 0xAA → 开始新帧
 *   ③ 如果正在收帧中 → 追加到帧缓冲
 *   ④ 如果收到连续两字节 0xFF 0xFF 且帧长度合法 → 帧完整，调 uart_cmd_dispatch
 *   ⑤ 非帧头也非帧内的字节 → 丢弃
 *   ⑥ 回到 ① 直到缓冲区空
 */
void uart_cmd_parser_tick(void);

#endif /* __UART_CMD_PARSER_H__ */
