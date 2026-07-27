/**
 * @file    imu_uart_handler.h
 * @brief   IMU UART 命令处理器 — C 接口
 */

#ifndef __IMU_UART_HANDLER_H__
#define __IMU_UART_HANDLER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 IMU 滤波器（绑定 scale）
 * @param id       IMU 编号
 * @param accel_lsb_per_g  加速度系数
 * @param gyro_lsb_per_dps 角速度系数
 */
void imu_uart_handler_init(uint8_t id, float accel_lsb_per_g, float gyro_lsb_per_dps);

/**
 * @brief 处理 IMU 命令（由 uart_cmd_dispatch 调用）
 * @param cmd    命令码
 * @param frame  完整帧（含帧头 cmd id 参数）
 * @param length 帧长度
 */
void imu_uart_handler_dispatch(uint8_t cmd, const uint8_t *frame, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
