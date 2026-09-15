/**
 * @file    host_attitude_test.c
 * @brief   attitude 感知组件 + imu_filter 纯 C 移植的 PC 测试桩 (C3, 2026-09-15)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc/common -I Core/Inc/driver \
 *       -o build/pc_test_attitude.exe \
 *       test/host_attitude_test.c Core/Src/driver/attitude.c Core/Src/common/imu_filter.c -lm
 *
 * 覆盖：Mahony 数学（纯陀螺积分/静止零角）、校准状态机（50 拍→零偏生效）、
 *       漂移跟踪门限（旋转不误吸 = IMU-1 修复锁）、静置吸收、IDRIFT 语义、
 *       dt 钳位、recalibrate。全场景 accel=(0,0,1g) 恒"静止"（绕竖轴旋转不变，
 *       即 IMU-1 病灶条件，验证门限在此条件下正确拦截）。
 */
#include <stdio.h>
#include <math.h>
#include "driver/attitude.h"
#include "imu_filter.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static int feq(float a, float b, float tol)
{
    float d = a - b;
    if (d < 0) d = -d;
    return d <= tol;
}

/* ---- mock IO：可编程 raw 陀螺/加速度 ---- */
static int16_t m_ax, m_ay, m_az, m_gx, m_gy, m_gz;

static attitude_ret_t mock_accel(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az)
{ (void)id; *ax = m_ax; *ay = m_ay; *az = m_az; return ATTITUDE_OK; }

static attitude_ret_t mock_gyro(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz)
{ (void)id; *gx = m_gx; *gy = m_gy; *gz = m_gz; return ATTITUDE_OK; }

static const attitude_io_t s_io = { .read_accel_raw = mock_accel, .read_gyro_raw = mock_gyro };

#define DT_MS   100u          /* 10Hz 节拍 */
#define GYRO131(x)  ((int16_t)((x) * 131.0f + ((x) >= 0 ? 0.5f : -0.5f)))   /* °/s → raw */

/* 初始化到"校准完成"态：50 拍静稳采样（raw 陀螺 = 传入零偏），accel 水平 */
static void setup_calibrated(uint8_t id, int16_t bias_x, int16_t bias_y, int16_t bias_z, uint32_t *now)
{
    attitude_cfg_t cfg = { 0 };          /* 全默认：50 拍 / 0.05g / 门限 1.5 / 0.02,0.05 */
    attitude_init(id, &cfg, &s_io);
    m_ax = 0; m_ay = 0; m_az = 16384;    /* 水平 1g */
    m_gx = bias_x; m_gy = bias_y; m_gz = bias_z;
    *now = 1000;
    for (int i = 0; i < 60; i++) {       /* 60 拍 > 50：越过完成拍（该拍 init_from_accel） */
        attitude_update(id, *now);
        *now += DT_MS;
    }
}

int main(void)
{
    printf("=== attitude/imu_filter PC 测试桩 (C3) ===\n");

    /* ---- 1. imu_filter 数学：静止水平 → 角度全零；纯陀螺积分线性 ---- */
    {
        imu_filter_t f;
        imu_filter_init(&f);
        float r, p;
        imu_filter_calc_roll_pitch(&f, 0.0f, 0.0f, 1.0f, &r, &p);
        check(feq(r, 0, 0.01f) && feq(p, 0, 0.01f), "水平 accel → roll/pitch = 0");
        imu_filter_init_from_accel(&f, 0.0f, 0.0f, 1.0f);
        check(feq(imu_filter_get_yaw(&f), 0, 0.001f), "init_from_accel → yaw = 0");
        for (int i = 0; i < 10; i++)
            imu_filter_mahony(&f, 0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 1.0f, 0.1f);
        check(feq(imu_filter_get_yaw(&f), 90.0f, 0.5f), "纯陀螺积分 90°/s × 1s → yaw ≈ 90");
        check(feq(imu_filter_get_roll(&f), 0, 0.1f) && feq(imu_filter_get_pitch(&f), 0, 0.1f),
              "绕 Z 旋转不扰动 roll/pitch");
    }

    /* ---- 2. 校准状态机：50 拍零偏生效 ---- */
    {
        uint32_t now;
        setup_calibrated(0, GYRO131(2.0f), 0, 0, &now);   /* X 轴 2°/s 零偏 */
        check(attitude_cal_progress(0) == 100, "60 拍后校准完成 (progress=100)");
        m_gx = GYRO131(2.0f);                              /* 同零偏继续静置 */
        attitude_update(0, now); now += DT_MS;
        check(feq(attitude_get_roll(0), 0, 0.1f), "校准吸收零偏 → 静置 roll 不漂");
    }

    /* ---- 3. 门限核心：旋转不误吸（IMU-1 修复锁）----
     * accel 恒水平 = 病灶条件；gz = 90°/s 持续 10 拍 = 转 90° ---- */
    {
        uint32_t now;
        setup_calibrated(0, 0, 0, 0, &now);
        m_gz = GYRO131(90.0f);
        for (int i = 0; i < 10; i++) { attitude_update(0, now); now += DT_MS; }
        check(feq(attitude_get_yaw(0), 90.0f, 1.0f), "旋转 90° → yaw 满跟（门限拦截吸收）");
        float dx, dy, dz;
        attitude_get_drift(0, &dx, &dy, &dz);
        check(fabsf(dz) < 0.001f, "旋转中 drift_z ≈ 0（不吸收）");
        check(attitude_stable(0) == 1, "旋转中加速度模值恒≈1g（stable=1，病灶条件在）");
    }

    /* ---- 4. 静置吸收：校准后新出现的残余零偏 < 门限 → 被跟踪 ---- */
    {
        uint32_t now;
        setup_calibrated(0, 0, 0, 0, &now);               /* 校准时零偏为 0 */
        m_gz = GYRO131(0.5f);                              /* 校准后漂移出 0.5°/s */
        for (int i = 0; i < 300; i++) { attitude_update(0, now); now += DT_MS; }   /* 30s */
        float dx, dy, dz;
        attitude_get_drift(0, &dx, &dy, &dz);
        check(feq(dz, 0.5f, 0.05f), "静置 30s → drift_z 收敛到 0.5°/s（真零偏被吸收）");
        /* 吸收后 yaw 增量冻结：最后 50 拍增量应 < 1° */
        float yaw0 = attitude_get_yaw(0);
        for (int i = 0; i < 50; i++) { attitude_update(0, now); now += DT_MS; }
        check(fabsf(attitude_get_yaw(0) - yaw0) < 1.0f, "吸收完成后 yaw 增量冻结");
    }

    /* ---- 5. IDRIFT 0 语义：清零 + 不吸收 + 回开 ---- */
    {
        uint32_t now;
        setup_calibrated(0, 0, 0, 0, &now);
        m_gz = GYRO131(90.0f);
        for (int i = 0; i < 5; i++) { attitude_update(0, now); now += DT_MS; }
        (void)attitude_set_drift_enable(0, 0);
        float dx, dy, dz;
        attitude_get_drift(0, &dx, &dy, &dz);
        check(attitude_drift_enabled(0) == 0 && fabsf(dz) < 0.001f, "IDRIFT 0 → 立即清零");
        float yaw0 = attitude_get_yaw(0);
        m_gz = GYRO131(0.5f);                               /* 残余零偏 0.5°/s */
        for (int i = 0; i < 100; i++) { attitude_update(0, now); now += DT_MS; }
        attitude_get_drift(0, &dx, &dy, &dz);
        check(fabsf(dz) < 0.001f, "IDRIFT 0 期间漂移不跟踪（保持 0）");
        check(attitude_get_yaw(0) - yaw0 > 4.0f, "无补偿时残余零偏线性积分（E1b 实测行为）");
        (void)attitude_set_drift_enable(0, 1);
        check(attitude_drift_enabled(0) == 1, "IDRIFT 1 → 恢复");
    }

    /* ---- 6. dt 钳位：长中断后小 dt 渐进恢复 ---- */
    {
        uint32_t now;
        setup_calibrated(0, 0, 0, 0, &now);
        m_gz = GYRO131(90.0f);
        float yaw0 = attitude_get_yaw(0);
        attitude_update(0, now + 5000u);                    /* 5s 断点 */
        float d = attitude_get_yaw(0) - yaw0;
        check(d > 0.0f && d < 2.0f, "断点 5s → dt 钳 0.01s，yaw 增量 <2°");
    }

    /* ---- 7. recalibrate：状态机复位 + 重新校准 ---- */
    {
        uint32_t now;
        setup_calibrated(0, 0, 0, 0, &now);
        (void)attitude_recalibrate(0);
        check(attitude_cal_progress(0) < 100, "recalibrate → 回到校准中");
        m_gz = GYRO131(3.0f);                               /* 新零偏 3°/s */
        for (int i = 0; i < 60; i++) { attitude_update(0, now); now += DT_MS; }
        check(attitude_cal_progress(0) == 100, "重新校准完成");
        m_gz = GYRO131(3.0f);
        attitude_update(0, now); now += DT_MS;
        check(feq(attitude_get_yaw(0), 0, 0.1f), "新零偏被吸收 → yaw 保持 0");
    }

    /* ---- 8. 越界安全侧 ---- */
    {
        check(attitude_ready(ATTITUDE_MAX_CH) == 0, "越界 ready → 0");
        check(attitude_get_yaw(ATTITUDE_MAX_CH) == 0.0f, "越界 get_yaw → 0.0");
        check(attitude_cal_progress(ATTITUDE_MAX_CH) == 0, "越界 progress → 0");
        check(attitude_init(ATTITUDE_MAX_CH, NULL, NULL) == ATTITUDE_ERR_BAD_ARG, "越界 init → BAD_ARG");
        check(attitude_init(0, NULL, &s_io) == ATTITUDE_ERR_BAD_ARG, "cfg NULL → BAD_ARG");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
