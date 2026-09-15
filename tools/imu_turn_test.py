# -*- coding: utf-8 -*-
"""
imu_turn_test.py — IMU yaw 90° 转角协议化测试 (IMU-1 专案 E 系列实验工具)

用法:
    envs/default/Scripts/python.exe tools/imu_turn_test.py [port] [方向cw/ccw] [outfile]
    port 缺省读 tools/data/bt_port.txt; outfile 缺省 tools/data/imu_turn_<ts>.csv

流程 (全程 ITEL 5Hz):
    阶段0 静置基线 3s  → 提示手转 90° (匀速, 建议 5~10s) → 阶段1 转角+静置 20s
    自动分析三指标: 跟随角 / 回落量 / 静置漂移(°/min)
    交叉验证: gz 积分角 (陀螺侧) vs yaw 输出角 (融合侧) — 定位"哪一侧丢角"

实验序列 (doc/IMU调试工具链规划.md §4.3):
    E1 基线(默认) → E1b 发 IDRIFT 0 后重跑 → E2 再发 IGAIN 0 0 → E3 发 IRATE 50 重跑
"""
import sys, time, os, serial

def port_default():
    f = os.path.join("tools", "data", "bt_port.txt")
    return open(f).read().strip() if os.path.exists(f) else "COM15"

def collect(ser, seconds):
    rows, t_end = [], time.time() + seconds
    while time.time() < t_end:
        for raw in ser.readlines():
            l = raw.decode(errors="ignore").strip()
            p = l.split(",")
            if len(p) == 9 and l[0].isdigit():
                try:
                    rows.append([int(x) for x in p])
                except ValueError:
                    pass
    return rows

def analyze(rows, label):
    """rows: [tick,gx10,gy10,gz10,roll10,pitch10,yaw10,stable,driftz100]"""
    if len(rows) < 10:
        print(f"  [{label}] 数据不足 ({len(rows)} 行)"); return None
    yaw  = [r[6] / 10.0 for r in rows]                       # °
    gz   = [r[3] / 10.0 for r in rows]                       # °/s
    stab = [r[7] for r in rows]
    n = len(rows)
    # 跟随角 = |yaw| 峰值 (单向转动); 回落 = 峰值 - 末段(静置)均值
    peak_i = max(range(n), key=lambda i: abs(yaw[i]))
    tail = yaw[max(0, n - 8):]                               # 末 ~1.6s
    settled = sum(tail) / len(tail)
    peak, settled_abs = yaw[peak_i], settled
    # 静置漂移: 末段前后 4s 的 yaw 斜率 (°/min), 行距由 tick 差自适应
    if n > 2:
        dts = sorted((rows[i+1][0] - rows[i][0]) for i in range(n - 1))
        med_ms = dts[len(dts) // 2]
        k = max(1, int(4000 / med_ms))                       # ~4s 跨度
    else:
        k = 1
    if n > k + 1:
        drift = (yaw[-1] - yaw[-1 - k]) / ((rows[-1][0] - rows[-1 - k][0]) / 1000.0) * 60.0
    else:
        drift = 0.0
    # 陀螺侧积分角: 静止段 gz ≈ 零偏, 用首个静止段均值扣除; dt 由 tick 差自适应 (IRATE 可变)
    base_i = next((i for i in range(n) if abs(yaw[i]) > 2.0), n // 2)
    base = sum(gz[:max(2, base_i)]) / max(2, base_i)
    dts = [(rows[i+1][0] - rows[i][0]) / 1000.0 for i in range(n - 1)]
    dt = sorted(dts)[len(dts) // 2] if dts else 0.2
    gyro_int = sum((g - base) * dt for g in gz[:peak_i + 1])
    print(f"  [{label}] 跟随角 {peak:7.1f}°  (陀螺侧积分 {gyro_int:7.1f}°)")
    print(f"          静置后 {settled_abs:7.1f}°  回落 {peak - settled_abs:6.1f}°  静置漂移 {drift:5.1f}°/min")
    print(f"          静止标志占比 {100.0 * sum(stab) / n:.0f}%  (旋转中恒为 1 = 漂移补偿误吸收窗口)")
    return {"peak": peak, "settled": settled_abs, "fallback": peak - settled_abs,
            "gyro_int": gyro_int, "drift": drift}

def main():
    port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else port_default()
    outfile = sys.argv[-1] if sys.argv[-1].endswith(".csv") else \
        f"tools/data/imu_turn_{int(time.time())}.csv"
    ser = serial.Serial(port, 9600, timeout=0.1)
    time.sleep(0.5); ser.reset_input_buffer()

    def send(c): ser.write((c + "\n").encode()); time.sleep(0.05)

    send("PING"); time.sleep(0.8)
    if "PONG" not in ser.read(300).decode(errors="ignore"):
        print("FAIL: 无 PONG, 链路不通"); ser.close(); return 1
    print("连接 OK (PONG)")

    print("== 前置: 车体静止水平 (必要时先发 ICAL 重新校准) ==")
    send("ITEL 1")
    print("阶段0: 静置基线 3s ..."); collect(ser, 3)
    print("\n>>> 现在把车**匀速**转 90° (建议 5~10s 转完), 转完保持不动 <<<")
    rows = collect(ser, 20)
    print("阶段2: 静置观察回落 (已含在 20s 采集内)")
    send("ITEL 0")
    ser.close()

    os.makedirs(os.path.dirname(outfile) or ".", exist_ok=True)
    with open(outfile, "w") as f:
        f.write("tick,gx10,gy10,gz10,roll10,pitch10,yaw10,stable,driftz100\n")
        f.write("\n".join(",".join(str(x) for x in r) for r in rows) + "\n")
    print(f"\n数据 -> {outfile}")
    print("== 分析结果 ==")
    r = analyze(rows, "本次")
    print("\n判读 (对照 doc/IMU调试工具链规划.md §3.2.1 仿真):")
    print("  跟随 ~17-33° 且回落到 ~0 → 复现症状 (漂移补偿误吸收, 走 E1b: IDRIFT 0 重跑)")
    print("  跟随 ~90° 回落 <5°       → 正常 (修复生效或未触发条件)")
    print("  陀螺侧积分 ≈ 跟随角      → 丢角在融合/补偿侧; 陀螺侧也小 → 陀螺/采样侧 (E3)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
