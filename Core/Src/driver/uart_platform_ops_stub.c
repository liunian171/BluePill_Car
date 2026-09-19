/**
 * ============================================================================
 *  uart_platform_ops_stub.c — UART 链路的编译期空壳（链路开关的"另一侧"）
 * ============================================================================
 *
 *  层位：驱动层平台层的占位实现（非真实平台绑定）。与 uart_platform_ops.c
 *  二选一参与编译，由 CMakeLists.txt 依据 Core/Inc/car_config.h 的
 *  CAR_FEATURE_UART 宏取舍（空壳替换方案，详 doc/双链路仲裁设计文档.md §5）。
 *
 *  ▸ 为什么存在 ◂
 *    CAR_FEATURE_UART=0（纯 USB 场测 / 不接 BT04）时，收敛点要落在"平台层"：
 *    真实平台的三个函数是唯一经 HAL 碰 huart2 的通道，空壳化后那条通道从
 *    固件里物理消失，而不是靠上层"记得别调"。符号名与真实实现完全一致
 *    → 组装层 main.c 的 uart_debug.ops 注入点零修改。
 *
 *  ▸ 语义承诺 ◂
 *    1) send/receive/receive_IT 一律 no-op：不碰任何 HAL 句柄
 *    2) 返回值口径不变（0 = 成功），调用方无需分支
 *    3) 真的"不阻塞"：真实实现每笔最坏 1000ms（UART_TIMEOUT_MS），
 *       空壳为 0 —— 对控制节拍只会更安全
 *
 *  ▸ 与组装层接线的关系（两层缺一不可）◂
 *    本文件只管"平台 ops 通道"；主循环的 UART 中断接收与直接发送在组装层
 *    main.c 里由 LINK_UART_ENABLED 门控（见 main.c [P3 双开关] 段）。
 *    两者都关掉才算 UART 链路真正下线。
 *
 *  ▸ 换平台/恢复链路 ◂
 *    本文件不动。恢复 UART = car_config.h 宏改 1 + 重新 cmake configure，
 *    本文件自动退出编译。
 * ============================================================================
 */

#include "uart_platform_ops.h"

/* 空壳模式不绑定任何硬件句柄，显式消费参数避免 -Wunused-parameter */

int8_t uart_stm32_send(void *huart, const uint8_t *data, uint16_t len)
{
    (void)huart;
    (void)data;
    (void)len;
    return 0;
}

int8_t uart_stm32_receive(void *huart, uint8_t *data, uint16_t len)
{
    (void)huart;
    (void)data;
    (void)len;
    return 0;
}

int8_t uart_stm32_receive_IT(void *huart, uint8_t *p_byte, uint16_t len)
{
    (void)huart;
    (void)p_byte;
    (void)len;
    return 0;
}

/* 符号名与真实平台保持一致 → 组装层注入点零修改 */
const UART_PlatformOps_t uart_platform_ops_stm32 = {
    .send       = uart_stm32_send,
    .receive    = uart_stm32_receive,
    .receive_IT = uart_stm32_receive_IT,
};
