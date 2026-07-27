/**
 * ============================================================================
 *  串口命令解析层 — 命令解析实现 + HAL_UART_RxCpltCallback 中断回调
 * ============================================================================
 *
 *  本文件包含三部分：
 *    1. 中断回调 —— 收到字节后存入环形缓冲区，重新使能接收
 *    2. 命令解析 —— 从环形缓冲区取字节，按帧头/帧尾拼帧，完整后分发
 *    3. 命令分发 —— switch(cmd) 跳转到对应 bridge 函数执行
 *
 *  数据流向：
 *    HAL_UART_RxCpltCallback → ringbuf_write() → [RingBuffer]
 *      → uart_cmd_parser_tick() → ringbuf_read() → 状态机拼帧
 *      → uart_cmd_dispatch() → bridge 函数执行
 *
 *  参考: UART_Serial_Design.md 第五、六章
 * ============================================================================
 */

#include "uart_cmd_parser.h"
#include <stdint.h>
#include <stdio.h>          /* snprintf, vsnprintf */
#include <stdarg.h>         /* va_list, va_start, va_end */
#include <string.h>         /* memcpy, strlen */
#include "usart.h"          /* 提供 huart2 / UART_HandleTypeDef */
#include "driver/uart.h"          /* uart_send */
#include "common/tool.h"    /* handle_to_id */
#include "motor_bridge.h"
#include "servo_bridge.h"
#include "pwm.h"
#include "imu_uart_handler.h"


/*==============================================================================
 *  文本回应辅助（调试用，发送 ASCII 可见字符串）
 *==============================================================================*/
static void uart_send_text(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

static void uart_send_text_fmt(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
}


/*==============================================================================
 *  平台相关配置宏 — 换平台时只改这两行
 *==============================================================================*/

/* 基准 UART HAL 句柄（取所有 UART 中地址最小的那个） */
#define UART_BASE_HANDLE    huart2

/* HAL 句柄类型名称 */
#define UART_HAL_TYPE       UART_HandleTypeDef


/*==============================================================================
 *  实例映射
 *
 *  通过 handle_to_id (tool.h) 将 HAL 回调传入的 UART_HandleTypeDef* 单射
 *  为数组下标，O(1) 定位到对应的 UART_Handle 和 RingBuffer。
 *
 *  原理：HAL 的各个 UART 句柄（huart1/huart6/huart7 等）在 .bss 段中
 *  连续排列，通过 (目标地址 - 基准地址) / sizeof(type) 直接算出序号。
 *==============================================================================*/

#define UART_INSTANCE_MAX  4

typedef struct
{
    UART_Handle *handle;       /* 自定义句柄（含 rx_byte、ops）;链接下层串口基本句柄 */
    RingBuffer  *ringbuf;      /* 对应的环形缓冲区               */
} UART_Instance;

static UART_Instance   uart_instances[UART_INSTANCE_MAX];


/*==============================================================================
 *  帧接收状态机变量 — 仅 uart_cmd_parser_tick() 使用
 *  中断不碰、main 不碰、bridge 不碰
 *==============================================================================*/
//当前只支持一个串口,后续放到UART_Instance中
static uint8_t   g_rx_frame[FRAME_RX_BUF_SIZE];    /* 正在组装的帧字节数组 */
static uint16_t  g_rx_index;                       /* 当前帧已收字节数       */
static uint8_t   g_in_frame;                       /* 0=等帧头, 1=收帧中   */

/* 前向声明 — static 函数在调用处之后定义时需要 */
static void uart_cmd_dispatch(const uint8_t *frame, uint16_t length);
static void uart_cmd_send_status(CmdCode_t cmd, int8_t id, StatusCode_t status);


/*==============================================================================
 *  初始化
 *==============================================================================*/

/**
 * @brief  绑定 UART 句柄和环形缓冲区，注册到uart_instances实例映射表
 * @param  hUART   已配置的 UART 句柄
 * @param  ringbuf 已初始化的环形缓冲区，中断向其中填入字节
 */
void uart_cmd_parser_init(UART_Handle *hUART, RingBuffer *ringbuf)
{
    uint8_t id = handle_to_id(&UART_BASE_HANDLE, sizeof(UART_HAL_TYPE), hUART->huart);

    uart_instances[id] = (UART_Instance){ .handle  = hUART,
                                          .ringbuf = ringbuf };
    //后面改到UART_Instance中,这里也要修改
    g_in_frame = 0;
    g_rx_index = 0;
}


/*==============================================================================
 *  主循环 tick — 帧组装状态机
 *==============================================================================*/

/**
 * @brief  >>>>>每次调用从ringbuf环形缓冲区读取积压字节并拼帧<<<<
 *
 *         每 tick 只取 1 字节（非 while 循环），180MHz 下多个 tick 间
 *         间隔极短，不影响帧组装时效。
 *
 *         状态机流程：
 *                         ringbuf 空 → 返回
 *                              │
 *                   ringbuf_read() 取一个 byte
 *                              │
 *               ┌──────────────┼──────────────┐
 *               ▼              ▼              ▼
 *         byte == 0xAA    g_in_frame    其他 → 丢弃
 *         && !g_in_frame       │
 *               │         追加到 g_rx_frame
 *         开始新帧：        g_rx_index++
 *         g_rx_index=0          │
 *         g_in_frame=1    byte == 0xFF 0xFF ?
 *                              │
 *                    ┌──── YES ┴─── NO ──→ 继续收
 *                    ▼
 *              uart_cmd_dispatch()
 *              g_in_frame = 0
 *              g_rx_index = 0
 *
 *         溢出保护: index >= FRAME_RX_BUF_SIZE → 丢弃残缺帧，重置
 */
void uart_cmd_parser_tick(void) {
   uint8_t byte;
  for (uint8_t i = 0; i < UART_INSTANCE_MAX; i++) {
    if (uart_instances[i].handle == NULL)
      continue;
    if (ringbuf_read(uart_instances[i].ringbuf, &byte) ==
        0) // 成功读取并存入byte
    {
      if (byte == 0xAA && g_in_frame == 0) // 帧头标志
      {
        g_rx_index = 0;
        g_in_frame = 1;
      }

      if (g_in_frame) // 收帧中
      {
        /* 缓冲区溢出保护：残缺帧超过上限则丢弃，重置等下一帧 */
        if (g_rx_index >= FRAME_RX_BUF_SIZE)
        {
          g_in_frame = 0;
          g_rx_index = 0;
          continue;
        }
        g_rx_frame[g_rx_index] = byte;

        if (byte == FRAME_EOF2_2 && g_rx_index >= 1
            && g_rx_frame[g_rx_index - 1] == FRAME_EOF2_1) // 两字节帧尾 FF FF
        {
            uart_cmd_dispatch(g_rx_frame, g_rx_index - 1); // 去掉最后一个 FF
            g_in_frame = 0;
            g_rx_index = 0;
        }
        else
        {
            g_rx_index++;
        }
      }
    }
  }
}

/*==============================================================================
 *  命令分发
 *==============================================================================*/

/**
 * @brief  帧完整后调用，根据 cmd 字段分发到对应 bridge 函数
 *
 *         帧结构（已去掉帧尾字节）:
 *           [0xAA] [cmd] [id] [参数...]
 *           index0 index1 index2 index3~

 *         参数提取（全部用 memcpy，无需位运算）:
 *           int16_t : memcpy(&val, &frame[3], 2)
 *           uint16_t: memcpy(&val, &frame[3], 2)
 *           float   : memcpy(&val, &frame[3], 4)
 */
static void uart_cmd_dispatch(const uint8_t *frame, uint16_t length)
{
    uint8_t cmd = frame[1];

    switch (cmd)
    {
    /* ======== 无 id 命令 ======== */
    case CMD_NOP:
        uart_cmd_send_status(cmd, 0xFF, STATUS_OK);
        break;

    case CMD_PING:
        uart_cmd_send_status(cmd, 0xFF, STATUS_OK);
        break;

    case CMD_GET_STATUS:
        uart_cmd_send_status(cmd, 0xFF, STATUS_OK);
        break;

    /* ======== 有 id 命令 ======== */
    case CMD_MOTOR_SET_RPM: {
        uint8_t id = frame[2];
        float rpm;
        memcpy(&rpm, &frame[3], 4);
        motor_bridge_set_speed_rpm(id, rpm);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        uart_send_text_fmt("M%d:%dRPM OK\r\n", id, (int)(rpm + 0.5f));
        break;
    }

    case CMD_MOTOR_SET_MPS: {
        uint8_t id = frame[2];
        float mps;
        memcpy(&mps, &frame[3], 4);
        motor_bridge_set_speed_mps(id, mps);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        int cm_s = (int)(mps * 100.0f + 0.5f);
        uart_send_text_fmt("M%d:%dcm/s OK\r\n", id, cm_s);
        break;
    }

    case CMD_MOTOR_BRAKE: {
        uint8_t id = frame[2];
        motor_bridge_brake(id);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        uart_send_text_fmt("M%d:BRAKE OK\r\n", id);
        break;
    }

    case CMD_MOTOR_SET_DEADZ: {
        uint8_t id = frame[2];
        float dead_zone;
        memcpy(&dead_zone, &frame[3], 4);
        motor_bridge_set_dead_zone(id, dead_zone);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        uart_send_text_fmt("M%d:DEADZ %d OK\r\n", id, (int)(dead_zone + 0.5f));
        break;
    }

    /* ---- Servo 命令组 ---- */
    case CMD_SERVO_SET_ANGLE: {
        uint8_t id = frame[2];
        float angle;
        memcpy(&angle, &frame[3], 4);
        servo_bridge_set_angle(id, angle);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;
    }

    case CMD_SERVO_START: {
        uint8_t id = frame[2];
        servo_bridge_start(id);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;
    }

    case CMD_SERVO_STOP: {
        uint8_t id = frame[2];
        servo_bridge_stop(id);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;
    }

    /* ---- PWM 命令组 ---- */
    case CMD_PWM_SET_DUTY: {
        /* TODO: 待 PWM 实例数组 (pwm_ch[]) 定义后接入 */
        uart_cmd_send_status(cmd, 0xFF, STATUS_OK);
        break;
    }

    /* ---- IMU 命令组 ---- */
    case CMD_IMU_GET_DATA:
    case CMD_IMU_CALIBRATE:
        uart_cmd_send_status(cmd, frame[2], STATUS_OK);  /* 先确认收到 */
        imu_uart_handler_dispatch(cmd, frame, length);
        break;

    default:
        uart_cmd_send_status(cmd, 0xFF, STATUS_INVALID_CMD);
        break;
    }
}


/*==============================================================================
 *  响应发送
 *==============================================================================*/

/** @brief 通过已注册的第一个 UART 实例发送响应帧 */
static void uart_cmd_send_status(CmdCode_t cmd, int8_t id, StatusCode_t status)
{
    uint8_t frame[] = {0xAA, cmd, id, status, 0xFF, 0xFF};
    uart_send(uart_instances[0].handle, frame, sizeof(frame));
}


/*==============================================================================
 *  中断回调 — HAL_UART_RxCpltCallback
 *==============================================================================*/

/**
 * @brief  HAL_UART_RxCpltCallback 移至 main.c 实现
 */
/* 在 main.c 中实现 */
