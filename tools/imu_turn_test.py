# -*- coding: utf-8 -*-
"""
imu_turn_test.py — IMU yaw 90° 转角协议化测试 (IMU-1 专案 E 系列实验工具, 自助版)

用法:
    envs/default/Scripts/python.exe tools/imu_turn_test.py <COM口> [前置命令] [outfile]
    例:  ... imu_turn_test.py COM7                    # E1 基线 (漂移补偿开)
         ... imu_turn_test.py COM7 "IDRIFT 0"         # E1b (关漂移补偿)
         ... imu_turn_test.py COM7 "IGAIN 0 0|IDRIFT 0"  # E2 纯陀螺积分 (|分隔多条)
         ... imu_turn_test.py COM7 "IRATE 50"          # E3 采样率对照
    前置命令在连接后、ICAL 之前发送 (用 | 分隔多条); 实验完成后手动恢复:
    IDRIFT 1 / IGAIN 50 0 / IRATE 100

流程 (全程 ITEL):
    连接 → [前置命令] → ICAL 重校准(静置 7s) → 基线 5s
    → 提示手转 90°(匀速 8~10s, 平稳别晃) → 采集 30s → 自动分析
三指标: 跟随角 / 回落量 / 静置漂移(°/min) + 陀螺侧 gz 积分交叉验证
"""
import sys, time, os, serial

def port_default():
    f = os.path.join("tools", "data", "bt_port.txt")
    return open(f).read().strip() if os.path.exists(f) else "COM15"

def collect(ser, seconds):
    rows, buf, t_end = [], b"", time.time() + seconds
    while time.time() < t_end:
        buf += ser.read(512)                       # [坑] readlines() 在 BT04 链路会无声挂死, 用 read() 分块
        while b"\n" in buf:
            l, buf = buf.split(b"\n", 1)
            l = l.decode(errors="ignore").strip()
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
    pre_cmd = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].endswith(".csv") else None
    outfile = sys.argv[-1] if sys.argv[-1].endswith(".csv") else \
        f"tools/data/imu_turn_{int(time.time())}.csv"
    ser = serial.Serial(port, 9600, timeout=0.5, write_timeout=2)
    time.sleep(2); ser.reset_input_buffer()

    def send(c):
        ser.write((c + "\n").encode()); time.sleep(0.05)

    send("PING"); time.sleep(0.8)
    if "PONG" not in ser.read(300).decode(errors="ignore"):
        print("FAIL: 无 PONG, 链路不通"); ser.close(); return 1
    print("连接 OK (PONG)")

    if pre_cmd:
        for c in pre_cmd.split("|"):
            send(c.strip()); time.sleep(0.4)
            print(f"前置: {c.strip()} -> {ser.read(200).decode(errors='ignore').strip()}")

    print("== ICAL 重校准: 车体平放保持静止 7s ... ==")
    send("ICAL")
    rows_cal = collect(ser, 7)
    print("== 基线采集 5s (保持静止) ==")
    send("ITEL 1")
    collect(ser, 5)
    print("\n>>> 现在把车**原地匀速**转 90° (8~10s 转完, 平稳别晃), 转完保持不动 <<<")
    rows = collect(ser, 30)
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
