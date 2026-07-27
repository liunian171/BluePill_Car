/**
 * @file    imu_uart_handler.cpp
 * @brief   IMU UART 命令处理实现
 */

#include "imu_uart_handler.h"
#include "imu_bridge.h"
#include "imu_filter.h"
#include "uart_cmd_parser.h"
#include "uart.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ---------- 状态 ---------- */
static ImuFilter *imu_filters[MAX_IMUS] = {nullptr};
static uint32_t   imu_last_tick[MAX_IMUS] = {0};

/* ---------- 外部 UART 句柄 ---------- */
extern UART_Handle uart_debug;


/* ========================================================================== */

void imu_uart_handler_init(uint8_t id, float accel_lsb_per_g, float gyro_lsb_per_dps)
{
    if (id >= MAX_IMUS) return;

    imu_filters[id] = new ImuFilter();
    imu_filters[id]->set_accel_scale(accel_lsb_per_g);
    imu_filters[id]->set_gyro_scale(gyro_lsb_per_dps);
    imu_last_tick[id] = HAL_GetTick();
}


/* ==========================================================================
 *  发送 IMU 数据响应帧
 *
 *  帧格式: 0xAA | cmd | id | roll(4B) | pitch(4B) | yaw(4B) | 0xFF | 0xFF
 *  共 19 字节
 * ========================================================================== */

static void imu_send_data(uint8_t id)
{
    if (!imu_bridge_ready(id))  return;
    if (!imu_filters[id])       return;

    /* ---- 读原始值 ---- */
    int16_t ax, ay, az, gx, gy, gz;
    if (imu_bridge_read_accel_raw(id, &ax, &ay, &az) != 0 ||
        imu_bridge_read_gyro_raw(id, &gx, &gy, &gz) != 0)
        return;

    ImuFilter *f = imu_filters[id];

    /* ---- 换算 + Mahony 滤波 ---- */
    float af[3], gf[3];
    f->raw_to_accel(ax, ay, az, &af[0], &af[1], &af[2]);
    f->raw_to_gyro(gx, gy, gz, &gf[0], &gf[1], &gf[2]);

    uint32_t now  = HAL_GetTick();
    float    dt   = (float)(now - imu_last_tick[id]) * 0.001f;
    imu_last_tick[id] = now;

    static int first = 1;
    if (first) {
        f->init_from_accel(af[0], af[1], af[2]);
        f->set_mahony_gains(0.5f, 0.0f);
        first = 0;
    }

    if (dt > 0.001f && dt < 1.0f) {
        f->mahony_filter(gf[0], gf[1], gf[2],
                         af[0], af[1], af[2], dt);
    }

    /* ---- 打包响应帧: 0xAA + cmd + id + roll(4B) + pitch(4B) + yaw(4B) + FF + FF = 17B ---- */
    uint8_t  frame[17];
    float    roll = f->get_roll(), pitch = f->get_pitch(), yaw = f->get_yaw();
    frame[0] = 0xAA;
    frame[1] = CMD_IMU_GET_DATA;
    frame[2] = id;
    memcpy(&frame[3],  &roll,  4);
    memcpy(&frame[7],  &pitch, 4);
    memcpy(&frame[11], &yaw,   4);
    frame[15] = 0xFF;
    frame[16] = 0xFF;

    uart_send(&uart_debug, frame, 17);
}


/* ==========================================================================
 *  校准
 * ========================================================================== */

static void imu_calibrate(uint8_t id, uint16_t samples)
{
    if (!imu_bridge_ready(id) || !imu_filters[id] || samples == 0) return;

    const uint16_t max_samples = 256;
    if (samples > max_samples) samples = max_samples;

    int16_t ax[max_samples], ay[max_samples], az[max_samples];
    int16_t gx[max_samples], gy[max_samples], gz[max_samples];

    for (uint16_t i = 0; i < samples; i++) {
        imu_bridge_read_accel_raw(id, &ax[i], &ay[i], &az[i]);
        imu_bridge_read_gyro_raw(id, &gx[i], &gy[i], &gz[i]);
        HAL_Delay(5);
    }

    imu_filters[id]->calibrate_accel_bias(ax, ay, az, samples);
    imu_filters[id]->calibrate_gyro_bias(gx, gy, gz, samples);
}


/* ==========================================================================
 *  分发（由 uart_cmd_dispatch 调用）
 * ========================================================================== */

void imu_uart_handler_dispatch(uint8_t cmd, const uint8_t *frame, uint16_t length)
{
    (void)length;  // 帧长度校验由上层处理

    uint8_t id = frame[2];

    switch (cmd) {
    case CMD_IMU_GET_DATA:
        imu_send_data(id);
        break;

    case CMD_IMU_CALIBRATE: {
        uint16_t samples;
        memcpy(&samples, &frame[3], 2);
        imu_calibrate(id, samples);
        break;
    }
    }
}
