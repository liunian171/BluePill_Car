# -*- coding: utf-8 -*-
"""
imu_sim_verify.py — IMU yaw 失真根因 PC 仿真验证 (纯 python, 无第三方依赖)

目的 (对应 doc/IMU调试工具链规划.md §3.2 假说):
  逐行复刻固件流水线 (imu_bridge.cpp 漂移补偿 + imu_filter.cpp Mahony halfez=0),
  在 PC 上复现 "手转 90° 只跟 ~20° 且回落到 0" 的真机症状, 并验证修复方案。

复刻依据 (源码逐行对照, 2026-09-15):
  - IMU 更新周期 100ms (main.c 100ms 显示块, 10Hz)
  - 静止判据: |accel模值 - 1g| < 0.05g (imu_bridge.cpp:202) — 绕竖直轴旋转时恒成立
  - 漂移跟踪: 静止>1s 后 rate=0.02 (1~5s) / 0.05 (>5s), 一阶收敛 (imu_bridge.cpp:208-213)
  - 修正: gf -= drift (imu_bridge.cpp:220-222)
  - Mahony: halfez=0 (imu_filter.cpp:130), Kp=0.5 Ki=0 (imu_bridge.cpp:78)
  - 初始零偏校准: 50 采样取均值 (imu_bridge.cpp:166-177)

场景: 启动静稳校准 5s → 静置 5s → 手转 90° (匀速) → 静置 15s 观察回落
输出: 各场景 跟随角 / 15s 后稳态角 / 回落量
"""
import math

DT = 0.1                     # IMU 更新周期 s (main.c 100ms 块)
CAL_N = 50                   # 初始零偏校准采样数
GYRO_GATE = 1.5              # 修复方案: 陀螺模值门限 (°/s), 三轴均低于此才允许跟踪


class ImuSim:
    """逐行复刻 imu_bridge_update_filter + ImuFilter::mahony_filter (halfez=0)"""

    def __init__(self, drift_enable=True, gyro_gate=None, kp=0.5, ki=0.0):
        self.kp, self.ki = kp, ki
        self.drift_enable = drift_enable
        self.gyro_gate = gyro_gate          # None=现固件逻辑; float=修复门限
        # 滤波器状态 (imu_filter.cpp)
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.int_fb = [0.0, 0.0, 0.0]
        self.rpys = [0.0, 0.0, 0.0]
        # 桥接层状态 (imu_bridge.cpp per-id)
        self.bias = [0.0, 0.0, 0.0]         # 初始校准结果
        self.cal_buf, self.cal_cnt = [], 0
        self.drift = [0.0, 0.0, 0.0]
        self.stable_since = None

    # ---- 初始零偏校准段 (imu_bridge.cpp:166-183) ----
    def feed_cal(self, gx, gy, gz):
        self.cal_buf.append((gx, gy, gz))
        self.cal_cnt += 1
        if self.cal_cnt == CAL_N:
            for i in range(3):
                self.bias[i] = sum(s[i] for s in self.cal_buf) / CAL_N
            self.cal_buf = None
            self.cal_cnt += 1               # = CAL_DONE
        return

    # ---- 正常更新段 (imu_bridge.cpp:149-236) ----
    def update(self, gx, gy, gz, ax, ay, az, stable, dt):
        # 换算物理量 (raw - bias)/scale — 校准后 residual 已含在输入里, 此处直通
        gf = [gx - self.bias[0], gy - self.bias[1], gz - self.bias[2]]
        af = [ax, ay, az]

        # 静止检测 + 漂移跟踪 (imu_bridge.cpp:200-217)
        if self.drift_enable and stable:
            if self.stable_since is None:
                self.stable_since = self.now
            stable_ms = (self.now - self.stable_since) * 1000.0   # 内部秒 → ms (对齐固件 HAL_GetTick 域)
            if stable_ms > 1000:
                gate_ok = True
                if self.gyro_gate is not None:
                    gate_ok = all(abs(g) < self.gyro_gate for g in gf)
                if gate_ok:
                    rate = 0.05 if stable_ms > 5000 else 0.02
                    for i in range(3):
                        self.drift[i] += (gf[i] - self.drift[i]) * rate
        else:
            self.stable_since = None

        gf = [gf[i] - self.drift[i] for i in range(3)]
        self.now += DT
        self.mahony(gf, af, dt)

    # ---- Mahony (imu_filter.cpp:89-208, halfez=0, IMU-only) ----
    def mahony(self, g, a, dt):
        q = self.q
        halfex = halfey = halfez = 0.0
        norm = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
        gain = 1.0
        if norm >= 1e-6:
            if norm < 0.5 or norm > 1.5:   gain = 0.0
            elif norm < 0.8 or norm > 1.2: gain = 0.3
            if gain > 0.0:
                ax, ay, az = (x / norm for x in a)
                halfvx = q[1]*q[3] - q[0]*q[2]
                halfvy = q[0]*q[1] + q[2]*q[3]
                halfvz = q[0]*q[0] - 0.5 + q[3]*q[3]
                halfex = (ay*halfvz - az*halfvy) * gain
                halfey = (az*halfvx - ax*halfvz) * gain
                halfez = 0.0                # IMU-only: yaw 修正清零 (imu_filter.cpp:130)
        if self.ki > 0.0:
            for i, e in enumerate((halfex, halfey, halfez)):
                self.int_fb[i] += self.ki * e * dt
                g[i] += self.int_fb[i]
        gx, gy, gz = g[0] + self.kp * halfex, g[1] + self.kp * halfey, g[2] + self.kp * halfez
        # deg/s → rad/s, 四元数积分 (imu_filter.cpp:167-183)
        d = math.pi / 180.0
        gxr, gyr, gzr = gx * d, gy * d, gz * d
        dq = [0.5*(-q[1]*gxr - q[2]*gyr - q[3]*gzr),
              0.5*( q[0]*gxr + q[2]*gzr - q[3]*gyr),
              0.5*( q[0]*gyr - q[1]*gzr + q[3]*gxr),
              0.5*( q[0]*gzr + q[1]*gyr - q[2]*gxr)]
        for i in range(4):
            q[i] += dq[i] * dt
        # 归一化 (imu_filter.cpp:188-194)
        n = math.sqrt(sum(x * x for x in q))
        if n < 1e-6:
            return
        for i in range(4):
            q[i] /= n
        # 欧拉角 (imu_filter.cpp:204-208)
        q = self.q
        self.rpys[0] = math.degrees(math.atan2(2*(q[0]*q[1] + q[2]*q[3]),
                                               1 - 2*(q[1]**2 + q[2]**2)))
        self.rpys[1] = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2] - q[3]*q[1])))))
        self.rpys[2] = math.degrees(math.atan2(2*(q[0]*q[3] + q[1]*q[2]),
                                               1 - 2*(q[2]**2 + q[3]**2)))


def run_scenario(name, turn_s, drift_enable=True, gyro_gate=None, bias=(0.2, -0.1, 0.3)):
    """静稳校准 5s → 静置 5s → 匀速转 90° (turn_s 秒) → 静置 15s"""
    sim = ImuSim(drift_enable=drift_enable, gyro_gate=gyro_gate)
    sim.now = 0.0
    omega = 90.0 / turn_s                     # °/s
    yaw_end = yaw_final = None
    # 阶段 1: 静稳校准 (50 采样, 陀螺 = 零偏)
    for _ in range(CAL_N):
        sim.feed_cal(bias[0], bias[1], bias[2])
    # 仿真车体: 绕竖直轴旋转 → accel 恒 ≈ (0,0,1), is_stable 恒真 (症状核心)
    # 阶段 2: 静置 5s
    for _ in range(int(5.0 / DT)):
        sim.update(bias[0], bias[1], bias[2], 0, 0, 1, True, DT)
    # 阶段 3: 转 90° (绕竖直轴 → 陀螺 Z 轴)
    for _ in range(int(turn_s / DT)):
        sim.update(bias[0], bias[1], bias[2] + omega, 0, 0, 1, True, DT)
    yaw_end = sim.rpys[2]
    # 阶段 4: 静置 15s 观察回落
    for _ in range(int(15.0 / DT)):
        sim.update(bias[0], bias[1], bias[2], 0, 0, 1, True, DT)
    yaw_final = sim.rpys[2]
    print(f"{name:<34} 跟随 {yaw_end:7.1f}°   静置15s后 {yaw_final:7.1f}°   回落 {yaw_end - yaw_final:6.1f}°")
    return yaw_end, yaw_final


if __name__ == "__main__":
    print("=" * 86)
    print("IMU yaw 失真 PC 仿真验证 — 复刻 imu_bridge 漂移补偿 + Mahony(halfez=0), 10Hz")
    print("场景: 校准5s → 静置5s → 匀速转90° → 静置15s   (旋转时加速度模值恒≈1g=被判静止)")
    print("=" * 86)
    print("--- A. 现固件逻辑复现 (真机症状应为 跟随~20° + 回落到~0) ---")
    run_scenario("A1 慢转 10s (真机顺时针对照)", 10.0)
    run_scenario("A2 较快转 5s (真机逆时针对照)", 5.0)
    run_scenario("A3 慢转 14s (极端慢)", 14.0)
    print("--- B. 实验 E1: IDRIFT 0 (关漂移补偿, 无门限) ---")
    run_scenario("B1 慢转 10s, drift=off", 10.0, drift_enable=False)
    print("--- C. 修复方案: 陀螺模值门限 1.5°/s (E1 证实后落地形态) ---")
    run_scenario("C1 慢转 10s, 门限1.5", 10.0, gyro_gate=GYRO_GATE)
    run_scenario("C2 较快转 5s, 门限1.5", 5.0, gyro_gate=GYRO_GATE)
    run_scenario("C3 极端慢转 14s, 门限1.5", 14.0, gyro_gate=GYRO_GATE)
    print("=" * 86)
    print("判读: A 组复现症状即证实 §3.2 假说; C 组应 跟随≈90°且回落<1° (长静置漂移补偿保留)")
